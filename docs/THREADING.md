# Threading & IPC

## Scheduler Overview

The f1c100s provides a **single-core, priority-based preemptive** scheduler:

| Property | Value |
|----------|-------|
| Max threads | 32 (`MAX_THREADS`) |
| Priority levels | 32 (0 = highest, 31 = lowest) |
| Scheduling | Highest-priority first, round-robin within equal priority |
| Preemption source | Timer ISR calls `sched_tick()` each tick |
| Context frame | 16 words = 64 bytes: `[CPSR, R0–R12, LR, PC]` |

### Scheduler State

The `Scheduler` struct (static `SCHEDULER`) holds:

- `threads: [Option<Thread>; 32]` — thread table (slots, not dynamically allocated)
- `ready_bitmap: u32` — bit N set = at least one Ready thread at priority N
- `rr_cursor: [usize; 32]` — per-priority round-robin cursor
- `tick_count: u64` — monotonic tick counter (drives timeouts)
- `current_id` — currently running thread ID

### Thread State Machine

```
Dead ────────── thread_create() ──▶ Ready ── pick_next() ──▶ Running
                                      ▲                          │
                                      │                          │
                                      ├── unblock_thread() ──────┤
                                      │                           │
                                      ├── timeout wakeup ────────┤
                                      │                           │
                                      ├── thread_resume() ───────┤
                                      │                           ▼
                                      │              thread_sleep() / IPC block
                                      │                          │
                                      ◀──────────────────────────┘
                              Sleeping / Blocked / BlockedTimeout
```

**Thread states (`ThreadState`):**

| State | Meaning | Transition trigger |
|-------|---------|-------------------|
| `Dead` (0) | Slot empty or thread exited | `thread_delete()`, `thread_exit()` |
| `Ready` (1) | Waiting for CPU | `thread_create()`, unblock, timeout, preempt |
| `Running` (2) | On CPU | `pick_next()` |
| `Sleeping` (3) | Timed sleep (no IPC) | `thread_sleep()`, → Ready on timeout |
| `Blocked` (4) | IPC wait, no timeout | IPC take/wait call, → Ready on unblock |
| `BlockedTimeout` (5) | IPC wait with timeout | IPC take/wait with `Some(ticks)`, → Ready on timeout or unblock |

## Scheduling Algorithm

### `pick_next()`

1. Find highest bit set in `ready_bitmap` → highest priority with Ready threads
2. Use `rr_cursor` for that priority to find next Ready thread in round-robin
3. Return thread ID (or `None` if nothing ready)

### `sched_tick()` — called from timer ISR

1. Increment `tick_count`
2. Scan all threads: wake any `Sleeping`/`BlockedTimeout` whose `sleep_until` has elapsed
3. Time-slice expiry: decrement current thread's `tick`; if 0, re-queue as Ready
4. Preemption check: if a higher-priority thread became Ready, switch to it
5. Call `do_schedule_from_isr()` if context switch needed

### Context Switch Variants

| Function | Caller | Mechanism |
|----------|--------|-----------|
| `context_switch` | Thread (SVC mode) | Save/restore full frame, `LDM^` return |
| `context_switch_to` | `sched_start()` | Load first thread's frame, `LDM^` return |
| `context_switch_interrupt` | ISR (IRQ mode) | Set `switch_interrupt_flag` + from/to pointers; actual switch happens in IRQ-return epilogue |

## Thread API

```rust
// Core lifecycle
sched_init();                                               // Once, before any threads
thread_create(name, entry, param, stack, priority, tick);   // Create (returns ThreadHandle)
sched_start();                                              // Start scheduler (never returns)

// Runtime
thread_current() -> ThreadHandle;                           // Who am I?
thread_yield();                                             // Voluntarily yield slice
thread_sleep(ticks);                                        // Sleep N ticks
thread_delete(handle);                                      // Destroy another thread
thread_suspend(handle);                                     // Force another thread to stop
thread_resume(handle);                                      // Resume a suspended thread
thread_exit() -> !;                                         // Exit current thread
```

**Stack requirements:** each thread gets a `&'static mut [u8]`. Minimum ~512 bytes; real workloads need 1–8 KB. The scheduler stores a 64-byte frame at the top of the stack.

## IPC Primitives

All six primitives follow the same pattern:
- Blocking calls accept `timeout: Option<u32>` (tick count; `None` = forever)
- Internal state protected by `cpu::interrupt_disable()`/`cpu::interrupt_enable(cpsr)`
- Blocked threads stored as bitmasks (`u32` waiter flags), not linked lists
- Wake-up always picks the lowest-ID waiter (by `trailing_zeros()`)

### 1. SpinLock (`ipc::spinlock`)

Single-core guard: locks by disabling IRQs, unlocks by restoring CPSR.

```rust
use f1c100s::ipc::spinlock::{SpinLock, critical};

let lock = SpinLock::new();
let _guard = lock.lock();           // IRQs disabled here
// ... critical section ...
// Dropping _guard re-enables IRQs

critical(|| {
    // Shorthand: IRQs disabled for this closure
});
```

**Use when:** protecting shared data from ISR/thread races; brief sections only.

### 2. Semaphore (`ipc::semaphore`)

Counting semaphore. Threads block when count reaches 0.

```rust
use f1c100s::ipc::semaphore::Semaphore;

static SEM: Semaphore = Semaphore::new(3);  // Initial count

// In one thread
SEM.take(Some(100)).ok();   // Block up to 100 ticks if no slots

// In another thread (or ISR)
SEM.release().ok();         // Release one slot, wakes blocked waiter
SEM.release_from_isr();     // ISR-safe version (IRQs already off)
```

### 3. Mutex (`ipc::mutex`)

Recursive mutual exclusion with ownership tracking. Only the owning thread can unlock.

```rust
use f1c100s::ipc::mutex::Mutex;

static MUT: Mutex = Mutex::new();

// Thread A
MUT.lock(Some(50)).ok();    // Block up to 50 ticks
// ... exclusive access ...
MUT.unlock().ok();

// Thread B — can also lock (re-entrant if same thread)
MUT.try_lock();             // Non-blocking attempt
MUT.is_locked();            // Check state
MUT.owner();                // Returns Some(ThreadHandle) or None
```

**Key property:** recursive — the owning thread can `lock()` multiple times; each `lock()` needs a matching `unlock()`.

### 4. Event Flags (`ipc::event`)

32-bit flag word. Threads wait for specific bits to be set.

```rust
use f1c100s::ipc::event::{EventFlags, EventMode};

static EV: EventFlags = EventFlags::new();

// Producer (ISR or thread)
EV.set(1 << 3);             // Set bit 3, wakes matching waiters
EV.clear(1 << 3);           // Clear bit 3

// Consumer: wait for ANY of bits 0, 1, 3
let flags = EV.wait(0b1001, EventMode::Or, true, None).unwrap();
// clear_on_exit=true → bits auto-cleared on wake

// Consumer: wait for ALL of bits 4 and 5
EV.wait((1 << 4) | (1 << 5), EventMode::And, false, Some(500));
```

**Modes:**
- `EventMode::Or` — wake when **any** requested bit is set
- `EventMode::And` — wake when **all** requested bits are set

### 5. Message Queue (`ipc::msgqueue`)

Fixed-size byte messages, user-provided backing buffer. Zero heap allocation.

```rust
use f1c100s::ipc::msgqueue::MsgQueue;

static mut BUF: [u8; 512] = [0u8; 512];
let mut q = MsgQueue::new(unsafe { &mut BUF }, 32);  // 32-byte msgs → 16 slots

// Send (blocks if full)
q.send(b"hello, world! (padded to 32B)", None).ok();

// Receive (blocks if empty)
let mut msg = [0u8; 32];
q.recv(&mut msg, Some(200)).ok();

// Non-blocking variants
q.try_send(b"...");
q.try_recv(&mut msg);

// Inspection
q.len(); q.is_empty(); q.is_full(); q.capacity();
```

**Constraints:**
- All messages are exactly `msg_size` bytes — `BadSize` error if mismatched
- Both senders and receivers block when queue is full/empty respectively
- Thread safety via `Send` + `Sync` impls (IRQ disable internally)

### 6. Signals (`ipc::signal`)

Per-thread software signals (0–31), delivered synchronously.

```rust
use f1c100s::ipc::signal;

// Thread A: install handler for SIGUSR1
signal_install(1, |sig| {
    // Handle signal 1
});

// Thread B: send signal to thread A
signal_send(thread_a_handle, 1).ok();

// Thread A: block until a signal arrives
let which = signal_wait(0xFFFF_FFFF, None).unwrap();  // Wait for any signal

// Or: poll without blocking
signal_check();  // Deliver all pending, unmasked signals
```

**Signal mask:** bits set in the mask **block** those signals (they stay pending but don't fire).

## Timer ISR Integration

The scheduler requires a periodic timer ISR. The typical setup:

```rust
static mut IDLE_STACK: [u8; 2048] = [0u8; 2048];
static mut WORKER_STACK: [u8; 4096] = [0u8; 4096];

fn timer_isr(_vector: u32) {
    f1c100s::thread::sched_tick();   // Drive scheduler
    // Clear timer interrupt pending flag
}

#[no_mangle]
pub extern "C" fn rust_main() -> ! {
    f1c100s::mmu::init(&[...]);          // Set up memory map
    f1c100s::interrupt::init();          // Init INTC
    f1c100s::thread::sched_init();       // Init scheduler

    // Create threads
    f1c100s::thread::thread_create("idle", idle_fn, core::ptr::null_mut(),
        unsafe { &mut IDLE_STACK }, 31, 10);
    f1c100s::thread::thread_create("worker", worker_fn, core::ptr::null_mut(),
        unsafe { &mut WORKER_STACK }, 5, 5);

    // Install timer ISR + unmask timer interrupt
    f1c100s::interrupt::install(f1c100s::interrupt::TIMER0_INTERRUPT, timer_isr);
    f1c100s::interrupt::unmask(f1c100s::interrupt::TIMER0_INTERRUPT);

    // Configure timer for periodic ticks (e.g. 1 ms)
    // ... timer::configure(...) ...

    unsafe { f1c100s::cpu::interrupt_enable(f1c100s::cpu::MODE_SVC); }
    f1c100s::thread::sched_start();     // Never returns
}

fn idle_fn(_: *mut ()) {
    loop {}
}

fn worker_fn(_: *mut ()) {
    // Application logic here
    loop {
        f1c100s::thread::thread_sleep(10);
    }
}
```

## Design Notes

- **Single-core, no lock prefix:** all synchronization is IRQ disable/enable. No atomic operations needed (ARMv5TE has no `ldrex`/`strex`).
- **Bitmask waiters:** each IPC object stores blocked threads as a `u32` bitmask (max 32 threads). Wake-up is O(1) via `trailing_zeros()`.
- **IPC timeout:** timeouts use the scheduler's `tick_count`. A thread in `BlockedTimeout` state sets `sleep_until = now + timeout_ticks`; `sched_tick()` wakes it when the counter catches up.
- **ISR context switching:** `context_switch_interrupt` doesn't switch immediately — it sets a flag and pointers. The actual switch happens in the IRQ-return epilogue (in `startup.s`), which checks `switch_interrupt_flag` and performs the switch if set. This avoids corrupting the IRQ-mode stack.
