//! Multi-threading example for f1c100s
//!
//! Demonstrates a producer-consumer pipeline with:
//! - Two worker threads: "sensor" (producer) and "display" (consumer)
//! - Message queue for data flow between workers
//! - Mutex-guarded shared counter (total messages processed)
//! - Semaphore for backpressure (limit unprocessed messages)
//! - Event flags for system state coordination
//! - Signal-based shutdown
//!
//! This is example code — compile into your own binary crate that provides `rust_main`.

#![no_std]
#![no_main]

use core::ptr;
use f1c100s::ipc::semaphore::Semaphore;
use f1c100s::ipc::mutex::Mutex;
use f1c100s::ipc::msgqueue::MsgQueue;
use f1c100s::ipc::event::{EventFlags, EventMode};
use f1c100s::ipc::spinlock::SpinLock;
use f1c100s::ipc::signal;
use f1c100s::thread::{self, ThreadHandle};
use f1c100s::{cpu, interrupt, mmu, timer};

// ── Constants ──────────────────────────────────────────────────────────────

const MSG_SIZE: usize = 16;
const QUEUE_CAPACITY_BYTES: usize = MSG_SIZE * 8; // 8 slots
const TICK_MS: u32 = 1; // Assuming timer is configured for 1 ms ticks
const SYSTEM_READY: u32 = 1 << 0;
const SENSOR_ACTIVE: u32 = 1 << 1;
const DISPLAY_ACTIVE: u32 = 1 << 2;
const SHUTDOWN_REQUESTED: u32 = 1 << 3;

// ── Static resources ───────────────────────────────────────────────────────

/// Message queue backing buffer
static mut QUEUE_BUF: [u8; QUEUE_CAPACITY_BYTES] = [0u8; QUEUE_CAPACITY_BYTES];

/// Global system state via event flags
static SYS_STATE: EventFlags = EventFlags::new();

/// Production slot semaphore: sensor can't send more than N unprocessed messages
static PROD_SLOTS: Semaphore = Semaphore::new(8);

/// Total messages sent (for diagnostics)
static TOTAL_SENT: Mutex = Mutex::new();

/// Total messages received
static TOTAL_RECV: Mutex = Mutex::new();

// ── Thread stacks ──────────────────────────────────────────────────────────

static mut IDLE_STACK: [u8; 2048] = [0u8; 2048];
static mut SENSOR_STACK: [u8; 4096] = [0u8; 4096];
static mut DISPLAY_STACK: [u8; 4096] = [0u8; 4096];
static mut MONITOR_STACK: [u8; 2048] = [0u8; 2048];

// ── Sensor thread (producer) ───────────────────────────────────────────────

/// Data packet sent from sensor to display
#[repr(C)]
#[derive(Clone, Copy)]
struct SensorData {
    counter: u32,
    value: i16,
    _pad: [u8; 6], // pad to 16 bytes
}

impl SensorData {
    fn new(counter: u32, value: i16) -> Self {
        Self { counter, value, _pad: [0u8; 6] }
    }

    fn to_bytes(&self) -> [u8; MSG_SIZE] {
        unsafe { core::mem::transmute(*self) }
    }

    fn from_bytes(bytes: &[u8; MSG_SIZE]) -> Self {
        unsafe { core::mem::transmute(*bytes) }
    }
}

fn sensor_thread(param: *mut ()) {
    // Get the message queue pointer passed as parameter
    let q_ptr = param as *mut MsgQueue;

    // Signal that sensor is active
    SYS_STATE.set(SENSOR_ACTIVE);

    let mut counter: u32 = 0;

    loop {
        // Check for shutdown
        if SYS_STATE.get() & SHUTDOWN_REQUESTED != 0 {
            break;
        }

        // Simulate sensor reading
        let value = read_adc_channel();

        // Take a production slot (blocks if consumer is behind)
        PROD_SLOTS.take(Some(TICK_MS * 100));

        // Build and send message
        let data = SensorData::new(counter, value);
        let bytes = data.to_bytes();

        let q = unsafe { &mut *q_ptr };
        q.send(&bytes, Some(TICK_MS * 50));

        // Update shared counter (mutex-guarded)
        TOTAL_SENT.lock(None).ok();
        // The lock gives us exclusive access — increment happens inside
        TOTAL_SENT.unlock().ok();
        // Note: in practice you'd hold a separate counter; this is illustrative

        counter += 1;

        // Periodic yield to let lower-priority threads run
        if counter % 50 == 0 {
            thread::thread_yield();
        }
    }

    // Shutdown: send poison pill
    let poison = SensorData::new(0xFFFF_FFFF, 0);
    let bytes = poison.to_bytes();
    let q = unsafe { &mut *q_ptr };
    q.send(&bytes, None);
    thread::thread_exit();
}

// ── Display thread (consumer) ──────────────────────────────────────────────

fn display_thread(param: *mut ()) {
    let q_ptr = param as *mut MsgQueue;

    SYS_STATE.set(DISPLAY_ACTIVE);

    let mut recv_count: u32 = 0;

    loop {
        // Wait for data from sensor
        let mut buf = [0u8; MSG_SIZE];
        let q = unsafe { &mut *q_ptr };

        match q.recv(&mut buf, Some(TICK_MS * 200)) {
            Ok(()) => {
                let data = SensorData::from_bytes(&buf);

                // Check poison pill
                if data.counter == 0xFFFF_FFFF {
                    break;
                }

                // Free a production slot
                PROD_SLOTS.release().ok();

                // Process display output
                update_display(data);

                recv_count += 1;

                // Update recv counter
                TOTAL_RECV.lock(None).ok();
                TOTAL_RECV.unlock().ok();

                // Every 100 messages, check for shutdown
                if recv_count % 100 == 0 {
                    if SYS_STATE.get() & SHUTDOWN_REQUESTED != 0 {
                        break;
                    }
                }
            }
            Err(f1c100s::ipc::msgqueue::MsgError::Timeout) => {
                // Timeout: check if we should shut down
                if SYS_STATE.get() & SHUTDOWN_REQUESTED != 0 {
                    break;
                }
            }
            _ => {}
        }
    }

    thread::thread_exit();
}

// ── Monitor thread ─────────────────────────────────────────────────────────

fn monitor_thread(_param: *mut ()) {
    // Wait for the system to be fully initialized
    SYS_STATE.wait(SENSOR_ACTIVE | DISPLAY_ACTIVE, EventMode::And, false, Some(TICK_MS * 500));

    // Install shutdown signal handler
    signal::signal_install(0, |_sig| {
        SYS_STATE.set(SHUTDOWN_REQUESTED);
    });

    loop {
        // Periodic health check: sleep 1 second between checks
        thread::thread_sleep(TICK_MS * 1000);

        // Read current event flags
        let state = SYS_STATE.get();

        // If both workers are down, system is idle
        if state & (SENSOR_ACTIVE | DISPLAY_ACTIVE) == 0 {
            // Workers gone — could restart them or halt
            break;
        }

        // Check semaphore status (look for potential stalls)
        let available = PROD_SLOTS.count();
        if available == 0 {
            // All slots consumed — consumer may be stuck
            // Could trigger diagnostics or recovery
        }
    }

    // Signal main to shut down
    SYS_STATE.set(SHUTDOWN_REQUESTED);
    thread::thread_exit();
}

// ── Idle thread ────────────────────────────────────────────────────────────

fn idle_thread(_param: *mut ()) {
    // The idle thread runs at the lowest priority (31).
    // It spins only when no other thread is ready.
    loop {
        // On real hardware: WFI could go here to save power
        // unsafe { core::arch::asm!("wfi"); }
    }
}

// ── Timer ISR ──────────────────────────────────────────────────────────────

fn timer_isr(_vector: u32) {
    // Drive the scheduler
    thread::sched_tick();

    // Clear timer interrupt
    // (implementation depends on which timer is used)
}

// ── Platform stubs (replace with real hardware calls) ──────────────────────

fn read_adc_channel() -> i16 {
    // In real code: read from ADC or I2C sensor
    // For now, return a dummy value
    42
}

fn update_display(_data: SensorData) {
    // In real code: push to LCD framebuffer
}

// ── Initialization ─────────────────────────────────────────────────────────

fn system_init() {
    // 1. Set up MMU
    // Map the entire address space: DRAM cached, MMIO non-cached
    mmu::init(&[
        mmu::MemDesc::new(0x00000000, 0xFFFF_FFFF, 0x00000000, mmu::RW_NCNB),
        mmu::MemDesc::new(0x80000000, 0x81FF_FFFF, 0x80000000, mmu::RW_CB),
    ]);

    // 2. Initialize interrupt controller
    interrupt::init();

    // 3. Initialize the heap if using alloc
    // unsafe { f1c100s::allocator::init_heap(HEAP_START, HEAP_SIZE); }

    // 4. Initialize scheduler
    thread::sched_init();
}

fn create_threads(q: &'static mut MsgQueue) -> ThreadHandle {
    let q_ptr = q as *mut MsgQueue as *mut ();

    // Create threads (priority 0 = highest, 31 = lowest)
    let sensor_h = thread::thread_create(
        "sensor", sensor_thread, q_ptr,
        unsafe { &mut SENSOR_STACK }, 2, 5,
    ).expect("failed to create sensor thread");

    let _display_h = thread::thread_create(
        "display", display_thread, q_ptr,
        unsafe { &mut DISPLAY_STACK }, 3, 5,
    ).expect("failed to create display thread");

    let _monitor_h = thread::thread_create(
        "monitor", monitor_thread, ptr::null_mut(),
        unsafe { &mut MONITOR_STACK }, 1, 10,
    ).expect("failed to create monitor thread");

    let _idle_h = thread::thread_create(
        "idle", idle_thread, ptr::null_mut(),
        unsafe { &mut IDLE_STACK }, 31, 10,
    ).expect("failed to create idle thread");

    sensor_h
}

fn start_timer() {
    // Configure TIMER0 for periodic interrupts
    // This is platform-specific; typical setup:
    //   - Set timer interval for desired tick rate (e.g., 1 ms)
    //   - Install ISR
    //   - Unmask interrupt
    //   - Start timer
    interrupt::install(interrupt::TIMER0_INTERRUPT, timer_isr);
    interrupt::unmask(interrupt::TIMER0_INTERRUPT);

    // Configure timer hardware (pseudocode):
    // timer::Timer::new(0).configure_interval(TICK_US);
    // timer::Timer::new(0).enable();
}

// ── Main entry ─────────────────────────────────────────────────────────────

#[no_mangle]
pub extern "C" fn rust_main() -> ! {
    // 1. Platform init
    system_init();

    // 2. Create IPC objects
    // Safety: QUEUE_BUF is 'static, initialized at link time
    let message_queue: &'static mut MsgQueue = unsafe {
        MsgQueue::new(&mut QUEUE_BUF, MSG_SIZE)
        // Note: MsgQueue::new returns an owned value.
        // In real code, use a static initializer or unsafe cast to &'static mut.
        // This snippet is illustrative of the pattern.
        // See the project's allocator setup for actual heap-backed objects.
    };

    // 3. Create threads
    let _sensor_thread = create_threads(message_queue);

    // 4. Signal system ready
    SYS_STATE.set(SYSTEM_READY);

    // 5. Start timer (drives scheduler ticks)
    start_timer();

    // 6. Enable interrupts (IRQ, not FIQ)
    unsafe { cpu::interrupt_enable(cpu::MODE_SVC); }

    // 7. Start scheduler — never returns
    //    The highest-priority Ready thread takes over immediately.
    thread::sched_start();

    // Unreachable — sched_start() diverges
}
