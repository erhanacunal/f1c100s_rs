//! Preemptive thread scheduler for Allwinner F1C100s (ARM926EJ-S)
//!
//! # Architecture
//! - Single-core, priority-based preemptive scheduler
//! - 32 priority levels (0 = highest, 31 = lowest)
//! - Round-robin within equal priorities
//! - Timer-driven preemption via `sched_tick()` (call from timer ISR)
//! - Context switch frame: [cpsr, r0-r12, lr, pc] = 16 words = 64 bytes
//!
//! # Usage
//! ```ignore
//! static mut IDLE_STACK: [u8; 1024] = [0u8; 1024];
//! sched_init();
//! thread_create("idle", idle_fn, core::ptr::null_mut(),
//!               unsafe { &mut IDLE_STACK }, 31, 10);
//! thread_create("main", main_fn, ...);
//! sched_start(); // never returns
//! ```

use core::arch::asm;
use crate::cpu;

// ── Constants ────────────────────────────────────────────────────────────────

pub const MAX_THREADS: usize = 32;
pub const MAX_PRIORITY: usize = 32;
pub const IDLE_PRIORITY: u8 = 31;
pub const NULL_ID: usize = 0xFF;

// IPC error codes stored in Thread::ipc_error
pub const IPC_OK: i32 = 0;
pub const IPC_TIMEOUT: i32 = -1;
pub const IPC_ERROR: i32 = -2;
pub const IPC_WAIT_SENTINEL: i32 = 0x7FFF_FFFF; // "still waiting"

// ── External ASM symbols ─────────────────────────────────────────────────────

extern "C" {
    fn context_switch(from: *mut *mut u8, to: *const *mut u8);
    fn context_switch_to(to: *const *mut u8);
    fn context_switch_interrupt(from: *mut *mut u8, to: *const *mut u8);
}

// ── Thread state ─────────────────────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum ThreadState {
    Dead = 0,
    Ready = 1,
    Running = 2,
    /// Timed sleep (no IPC)
    Sleeping = 3,
    /// IPC blocked, no timeout
    Blocked = 4,
    /// IPC blocked with timeout (sleep_until set)
    BlockedTimeout = 5,
}

// ── Thread handle ─────────────────────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct ThreadHandle(pub usize);

// ── Thread control block ──────────────────────────────────────────────────────

#[repr(C)]
pub struct Thread {
    // MUST be first: context switch asm does STR/LDR at offset 0
    pub(crate) sp: *mut u8,

    pub(crate) state: ThreadState,
    pub(crate) priority: u8,
    pub(crate) id: u8,
    _pad: u8,

    pub(crate) tick: u32,        // remaining time-slice ticks
    pub(crate) init_tick: u32,   // full time-slice length

    pub(crate) sleep_until: u64, // tick count at which to wake

    pub(crate) stack_base: *mut u8,
    pub(crate) stack_size: usize,

    pub name: &'static str,

    // IPC fields
    pub(crate) wait_flags: u32,   // event: flags being waited on
    pub(crate) wait_mode: u8,     // event: 0=OR  1=AND
    pub(crate) ipc_error: i32,    // result after wake-up

    pub(crate) is_ipc_wait: bool, // true if Sleeping state is really an IPC timeout

    // Signal fields
    pub(crate) sig_pending: u32,
    pub(crate) sig_mask: u32,
}

// Safety: single-core, no real parallelism
unsafe impl Send for Thread {}
unsafe impl Sync for Thread {}

// ── Stack initializer ─────────────────────────────────────────────────────────

/// Build the initial SVC stack frame so context_switch_to can start `entry(parameter)`.
/// Returns the initial SP value (points to the cpsr word at the base of the frame).
fn stack_init(
    entry: fn(*mut ()),
    parameter: *mut (),
    stack: &mut [u8],
    exit_fn: unsafe extern "C" fn() -> !,
) -> *mut u8 {
    let top = (stack.as_mut_ptr() as usize + stack.len()) & !7;
    // 16 words = 64 bytes below the top
    let frame = (top - 64) as *mut u32;
    unsafe {
        *frame.add(0) = 0x13u32;           // cpsr: SVC mode, IRQs enabled
        *frame.add(1) = parameter as u32;  // r0 = argument
        *frame.add(2) = 0;                 // r1
        *frame.add(3) = 0;                 // r2
        *frame.add(4) = 0;                 // r3
        *frame.add(5) = 0;                 // r4
        *frame.add(6) = 0;                 // r5
        *frame.add(7) = 0;                 // r6
        *frame.add(8) = 0;                 // r7
        *frame.add(9) = 0;                 // r8
        *frame.add(10) = 0;               // r9
        *frame.add(11) = 0;               // r10
        *frame.add(12) = 0;               // r11
        *frame.add(13) = 0;               // r12
        *frame.add(14) = exit_fn as u32;  // lr  → thread_exit_stub on return
        *frame.add(15) = entry as u32;    // pc  → entry point
    }
    frame as *mut u8
}

// ── Scheduler state ───────────────────────────────────────────────────────────

struct Scheduler {
    threads: [Option<Thread>; MAX_THREADS],
    count: usize,
    current_id: usize,
    /// Bit N = at least one thread at priority N is Ready
    ready_bitmap: u32,
    /// Per-priority round-robin cursor (thread ID of last-run thread)
    rr_cursor: [usize; MAX_PRIORITY],
    tick_count: u64,
    initialized: bool,
    started: bool,
}

impl Scheduler {
    const fn new() -> Self {
        Self {
            threads: [const { None }; MAX_THREADS],
            count: 0,
            current_id: NULL_ID,
            ready_bitmap: 0,
            rr_cursor: [NULL_ID; MAX_PRIORITY],
            tick_count: 0,
            initialized: false,
            started: false,
        }
    }
}

static mut SCHEDULER: Scheduler = Scheduler::new();

// ── Ready-queue bitmap helpers ────────────────────────────────────────────────

unsafe fn set_ready(id: usize, priority: u8) {
    let sched = &mut SCHEDULER;
    let p = priority as usize;
    // Mark the thread ready
    if let Some(t) = &mut sched.threads[id] {
        t.state = ThreadState::Ready;
    }
    sched.ready_bitmap |= 1u32 << p;
}

unsafe fn clear_ready_if_empty(priority: u8) {
    let sched = &mut SCHEDULER;
    let p = priority as usize;
    // Check if any Ready thread exists at this priority
    let any = sched.threads.iter().any(|t| {
        t.as_ref()
            .map(|t| t.priority as usize == p && t.state == ThreadState::Ready)
            .unwrap_or(false)
    });
    if !any {
        sched.ready_bitmap &= !(1u32 << p);
    }
}

/// Pick the next thread to run (highest-priority, round-robin within priority).
unsafe fn pick_next(sched: &mut Scheduler) -> Option<usize> {
    if sched.ready_bitmap == 0 {
        return None;
    }
    let highest_prio = sched.ready_bitmap.trailing_zeros() as usize;
    let cursor = sched.rr_cursor[highest_prio];

    // Find the next Ready thread at this priority after cursor
    let start = if cursor == NULL_ID { 0 } else { (cursor + 1) % MAX_THREADS };
    let mut idx = start;
    for _ in 0..MAX_THREADS {
        if let Some(t) = &sched.threads[idx] {
            if t.priority as usize == highest_prio && t.state == ThreadState::Ready {
                return Some(idx);
            }
        }
        idx = (idx + 1) % MAX_THREADS;
    }
    None
}

// ── Core scheduler logic ──────────────────────────────────────────────────────

/// Perform a context switch from current to next thread.
/// Must be called with interrupts disabled.
unsafe fn do_schedule() {
    let sched = &mut SCHEDULER;

    let Some(next_id) = pick_next(sched) else { return };

    let cur_id = sched.current_id;
    if next_id == cur_id {
        // Same thread: mark running and return
        if let Some(t) = &mut sched.threads[cur_id] {
            t.state = ThreadState::Running;
        }
        return;
    }

    // Update states
    {
        let next = sched.threads[next_id].as_mut().unwrap();
        next.state = ThreadState::Running;
        sched.rr_cursor[next.priority as usize] = next_id;
    }

    let from_sp_ptr: *mut *mut u8;
    let to_sp_ptr: *const *mut u8;

    {
        let from = sched.threads[cur_id].as_mut().unwrap();
        from_sp_ptr = &mut from.sp as *mut *mut u8;
    }
    {
        let to = sched.threads[next_id].as_ref().unwrap();
        to_sp_ptr = &to.sp as *const *mut u8;
    }

    sched.current_id = next_id;
    context_switch(from_sp_ptr, to_sp_ptr);
}

/// Perform context switch from ISR (sets flag for IRQ-exit assembly).
unsafe fn do_schedule_from_isr() {
    let sched = &mut SCHEDULER;

    let Some(next_id) = pick_next(sched) else { return };

    let cur_id = sched.current_id;
    if next_id == cur_id {
        if let Some(t) = &mut sched.threads[cur_id] {
            t.state = ThreadState::Running;
        }
        return;
    }

    {
        let next = sched.threads[next_id].as_mut().unwrap();
        next.state = ThreadState::Running;
        sched.rr_cursor[next.priority as usize] = next_id;
    }

    let from_sp_ptr: *mut *mut u8 = {
        let from = sched.threads[cur_id].as_mut().unwrap();
        &mut from.sp as *mut *mut u8
    };
    let to_sp_ptr: *const *mut u8 = {
        let to = sched.threads[next_id].as_ref().unwrap();
        &to.sp as *const *mut u8
    };

    sched.current_id = next_id;
    context_switch_interrupt(from_sp_ptr, to_sp_ptr);
}

// ── Public API ────────────────────────────────────────────────────────────────

/// Initialize the scheduler. Call once before `thread_create`.
pub fn sched_init() {
    unsafe {
        let sched = &mut SCHEDULER;
        *sched = Scheduler::new();
        sched.initialized = true;
    }
}

/// Create a new thread. Returns a handle on success.
///
/// - `stack`: static byte slice used as the thread's stack (≥ 512 bytes recommended)
/// - `priority`: 0 (highest) to 31 (lowest); use 31 for idle
/// - `tick`: time-slice length in scheduler ticks
pub fn thread_create(
    name: &'static str,
    entry: fn(*mut ()),
    parameter: *mut (),
    stack: &'static mut [u8],
    priority: u8,
    tick: u32,
) -> Option<ThreadHandle> {
    let cpsr = cpu::interrupt_disable();
    let result = unsafe { thread_create_inner(name, entry, parameter, stack, priority, tick) };
    cpu::interrupt_enable(cpsr);
    result
}

unsafe fn thread_create_inner(
    name: &'static str,
    entry: fn(*mut ()),
    parameter: *mut (),
    stack: &'static mut [u8],
    priority: u8,
    tick: u32,
) -> Option<ThreadHandle> {
    let sched = &mut SCHEDULER;
    // Find a free slot
    let id = sched.threads.iter().position(|t| t.is_none())?;

    let sp = stack_init(entry, parameter, stack, thread_exit_stub);
    let priority = priority.min(31);

    sched.threads[id] = Some(Thread {
        sp,
        state: ThreadState::Ready,
        priority,
        id: id as u8,
        _pad: 0,
        tick,
        init_tick: tick,
        sleep_until: 0,
        stack_base: stack.as_mut_ptr(),
        stack_size: stack.len(),
        name,
        wait_flags: 0,
        wait_mode: 0,
        ipc_error: 0,
        is_ipc_wait: false,
        sig_pending: 0,
        sig_mask: 0,
    });

    sched.count += 1;
    sched.ready_bitmap |= 1u32 << priority;
    Some(ThreadHandle(id))
}

/// Start the scheduler. Switches to the highest-priority ready thread. Never returns.
pub fn sched_start() -> ! {
    unsafe {
        let sched = &mut SCHEDULER;
        assert!(sched.initialized, "sched_init() not called");

        let next_id = pick_next(sched).expect("no threads");
        {
            let t = sched.threads[next_id].as_mut().unwrap();
            t.state = ThreadState::Running;
            sched.rr_cursor[t.priority as usize] = next_id;
        }
        sched.current_id = next_id;
        sched.started = true;

        let to_sp_ptr: *const *mut u8 = {
            let t = sched.threads[next_id].as_ref().unwrap();
            &t.sp as *const *mut u8
        };
        context_switch_to(to_sp_ptr);
    }
    loop {}
}

/// Return a handle to the currently running thread.
pub fn thread_current() -> ThreadHandle {
    ThreadHandle(unsafe { SCHEDULER.current_id })
}

/// Voluntarily yield the current time slice.
pub fn thread_yield() {
    let cpsr = cpu::interrupt_disable();
    unsafe {
        let sched = &mut SCHEDULER;
        let cur_id = sched.current_id;
        if let Some(t) = &mut sched.threads[cur_id] {
            if t.state == ThreadState::Running {
                t.state = ThreadState::Ready;
                // Don't reset tick — partial slice stays
            }
        }
        do_schedule();
    }
    cpu::interrupt_enable(cpsr);
}

/// Sleep the current thread for `ticks` scheduler ticks.
pub fn thread_sleep(ticks: u32) {
    let cpsr = cpu::interrupt_disable();
    unsafe {
        let sched = &mut SCHEDULER;
        let cur_id = sched.current_id;
        if let Some(t) = &mut sched.threads[cur_id] {
            t.state = ThreadState::Sleeping;
            t.is_ipc_wait = false;
            t.sleep_until = sched.tick_count + ticks as u64;
            clear_ready_if_empty(t.priority);
        }
        do_schedule();
    }
    cpu::interrupt_enable(cpsr);
}

/// Delete a thread (must not be the current thread).
pub fn thread_delete(handle: ThreadHandle) {
    let cpsr = cpu::interrupt_disable();
    unsafe {
        let sched = &mut SCHEDULER;
        let id = handle.0;
        if let Some(t) = &mut sched.threads[id] {
            let p = t.priority;
            t.state = ThreadState::Dead;
            sched.threads[id] = None;
            sched.count -= 1;
            clear_ready_if_empty(p);
        }
    }
    cpu::interrupt_enable(cpsr);
}

/// Suspend (pause) a thread without deleting it.
pub fn thread_suspend(handle: ThreadHandle) {
    let cpsr = cpu::interrupt_disable();
    unsafe {
        let sched = &mut SCHEDULER;
        let id = handle.0;
        if let Some(t) = &mut sched.threads[id] {
            if t.state == ThreadState::Ready || t.state == ThreadState::Running {
                let p = t.priority;
                t.state = ThreadState::Blocked;
                clear_ready_if_empty(p);
            }
        }
    }
    cpu::interrupt_enable(cpsr);
}

/// Resume a suspended thread.
pub fn thread_resume(handle: ThreadHandle) {
    let cpsr = cpu::interrupt_disable();
    unsafe {
        let sched = &mut SCHEDULER;
        let id = handle.0;
        if let Some(t) = &mut sched.threads[id] {
            if t.state == ThreadState::Blocked && !t.is_ipc_wait {
                set_ready(id, t.priority);
                if sched.started {
                    do_schedule();
                }
            }
        }
    }
    cpu::interrupt_enable(cpsr);
}

/// Called from the timer ISR each tick. Handles time-slice expiry and wakeups.
/// Must be called with interrupts already masked (inside ISR).
pub fn sched_tick() {
    unsafe {
        let sched = &mut SCHEDULER;
        sched.tick_count += 1;
        let now = sched.tick_count;

        // Wake sleeping/timeout-blocked threads
        for id in 0..MAX_THREADS {
            if let Some(t) = &mut sched.threads[id] {
                match t.state {
                    ThreadState::Sleeping | ThreadState::BlockedTimeout => {
                        if t.sleep_until != 0 && now >= t.sleep_until {
                            if t.is_ipc_wait {
                                t.ipc_error = IPC_TIMEOUT;
                                t.is_ipc_wait = false;
                            }
                            t.sleep_until = 0;
                            set_ready(id, t.priority);
                        }
                    }
                    _ => {}
                }
            }
        }

        // Deliver pending signals to current thread
        let cur_id = sched.current_id;
        if cur_id < MAX_THREADS {
            if let Some(t) = &mut sched.threads[cur_id] {
                let deliverable = t.sig_pending & !t.sig_mask;
                if deliverable != 0 {
                    t.sig_pending &= !deliverable;
                    // Signal delivery happens in thread context after tick
                    // (set pending, thread checks on return from ISR)
                    // For now, stored — user calls signal_check() or it's delivered
                    // automatically after scheduler resumes thread.
                    t.sig_pending |= deliverable; // re-set for thread-side delivery
                }
            }
        }

        // Time-slice expiry
        let cur_id = sched.current_id;
        if cur_id < MAX_THREADS {
            if let Some(t) = &mut sched.threads[cur_id] {
                if t.state == ThreadState::Running {
                    if t.tick > 0 {
                        t.tick -= 1;
                    }
                    if t.tick == 0 {
                        t.tick = t.init_tick;
                        t.state = ThreadState::Ready;
                        // Don't clear ready_bitmap — thread stays ready
                        sched.ready_bitmap |= 1u32 << t.priority;
                        do_schedule_from_isr();
                        return;
                    }
                }
            }
        }

        // Check if a higher-priority thread became ready this tick
        if sched.ready_bitmap != 0 {
            let cur_prio = sched.threads[cur_id]
                .as_ref()
                .map(|t| t.priority as u32)
                .unwrap_or(31);
            let best = sched.ready_bitmap.trailing_zeros();
            if best < cur_prio {
                if let Some(t) = &mut sched.threads[cur_id] {
                    if t.state == ThreadState::Running {
                        t.state = ThreadState::Ready;
                        sched.ready_bitmap |= 1u32 << t.priority;
                    }
                }
                do_schedule_from_isr();
            }
        }
    }
}

// ── IPC helpers (pub(crate)) ──────────────────────────────────────────────────

/// Block the current thread on an IPC object. Switches to next thread.
/// Call with interrupts disabled; re-enables them after the switch.
pub(crate) unsafe fn block_current(timeout_ticks: Option<u32>) {
    let sched = &mut SCHEDULER;
    let id = sched.current_id;
    if let Some(t) = &mut sched.threads[id] {
        t.ipc_error = IPC_WAIT_SENTINEL;
        t.is_ipc_wait = true;
        if let Some(ticks) = timeout_ticks {
            t.sleep_until = sched.tick_count + ticks as u64;
            t.state = ThreadState::BlockedTimeout;
        } else {
            t.state = ThreadState::Blocked;
        }
        let p = t.priority;
        clear_ready_if_empty(p);
    }
    do_schedule();
}

/// Wake a thread blocked on IPC (success).
pub(crate) unsafe fn unblock_thread(id: usize) {
    let sched = &mut SCHEDULER;
    if let Some(t) = &mut sched.threads[id] {
        match t.state {
            ThreadState::Blocked | ThreadState::BlockedTimeout | ThreadState::Sleeping
                if t.is_ipc_wait =>
            {
                t.ipc_error = IPC_OK;
                t.is_ipc_wait = false;
                t.sleep_until = 0;
                set_ready(id, t.priority);
            }
            _ => {}
        }
    }
}

/// Return the IPC result for the current thread after unblocking.
pub(crate) fn current_ipc_error() -> i32 {
    unsafe { SCHEDULER.threads[SCHEDULER.current_id].as_ref().map(|t| t.ipc_error).unwrap_or(IPC_ERROR) }
}

/// Get thread ID for an IPC wait queue search.
pub(crate) fn current_id() -> usize {
    unsafe { SCHEDULER.current_id }
}

/// Get the thread's signal pending/mask (for signal module).
pub(crate) unsafe fn thread_sig_refs(id: usize) -> Option<(&'static mut u32, &'static mut u32)> {
    let sched = &mut SCHEDULER;
    sched.threads[id].as_mut().map(|t| {
        let pending: &'static mut u32 = &mut *(&mut t.sig_pending as *mut u32);
        let mask: &'static mut u32 = &mut *(&mut t.sig_mask as *mut u32);
        (pending, mask)
    })
}

// ── Stub called when a thread entry function returns ─────────────────────────

unsafe extern "C" fn thread_exit_stub() -> ! {
    let cpsr = cpu::interrupt_disable();
    let sched = &mut SCHEDULER;
    let id = sched.current_id;
    if let Some(t) = &mut sched.threads[id] {
        let p = t.priority;
        t.state = ThreadState::Dead;
        sched.threads[id] = None;
        sched.count -= 1;
        clear_ready_if_empty(p);
    }
    do_schedule();
    cpu::interrupt_enable(cpsr);
    loop {
        asm!("mcr p15, 0, {0}, c7, c0, 4", in(reg) 0u32);
    }
}

/// Exit the current thread explicitly.
pub fn thread_exit() -> ! {
    unsafe { thread_exit_stub() }
}
