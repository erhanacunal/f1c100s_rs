//! Software signaling — per-thread pending signals with installed handlers.
//!
//! # Signal numbers
//! 0–31 are valid. Signal 0 is conventionally "no signal" but may be sent.
//!
//! # Delivery
//! Signals are delivered synchronously: the target thread's pending mask is
//! updated, and the handler runs when the thread calls `signal_check()` or
//! when it wakes from `signal_wait()`.
//!
//! # Usage
//! ```ignore
//! // Install handler in the current thread
//! signal_install(1, |sig| { /* handle SIGUSR1 */ });
//!
//! // Send signal to another thread
//! signal_send(handle, 1);
//!
//! // Wait for any signal with no mask (block until one arrives)
//! if let Ok(sig) = signal_wait(0xFFFF_FFFF, None) { ... }
//! ```

use crate::cpu;
use crate::thread::{self, ThreadHandle, MAX_THREADS};

pub const MAX_SIGNALS: usize = 32;

// ── Global signal handler table ───────────────────────────────────────────────

static mut HANDLERS: [[Option<fn(u32)>; MAX_SIGNALS]; MAX_THREADS] =
    [[None; MAX_SIGNALS]; MAX_THREADS];

// ── Waiter state for signal_wait ──────────────────────────────────────────────

/// Per-thread signal wait mask
static mut SIG_WAIT_MASK: [u32; MAX_THREADS] = [0u32; MAX_THREADS];
/// Bitmask of threads blocked in signal_wait
static mut SIG_WAITERS: u32 = 0;

// ── Public API ────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignalError {
    InvalidSignal,
    Timeout,
}

/// Install a handler for `signo` in the current thread.
/// `signo` must be 0–31.
pub fn signal_install(signo: u32, handler: fn(u32)) -> Result<(), SignalError> {
    if signo >= MAX_SIGNALS as u32 { return Err(SignalError::InvalidSignal); }
    let cpsr = cpu::interrupt_disable();
    let id = thread::current_id();
    unsafe { HANDLERS[id][signo as usize] = Some(handler); }
    cpu::interrupt_enable(cpsr);
    Ok(())
}

/// Remove the handler for `signo` in the current thread.
pub fn signal_uninstall(signo: u32) -> Result<(), SignalError> {
    if signo >= MAX_SIGNALS as u32 { return Err(SignalError::InvalidSignal); }
    let cpsr = cpu::interrupt_disable();
    let id = thread::current_id();
    unsafe { HANDLERS[id][signo as usize] = None; }
    cpu::interrupt_enable(cpsr);
    Ok(())
}

/// Set the signal mask for the current thread.
/// Bits set in `mask` will cause those signals to be **blocked** (not delivered).
/// Returns the previous mask.
pub fn signal_mask(mask: u32) -> u32 {
    let cpsr = cpu::interrupt_disable();
    let id = thread::current_id();
    let old = unsafe {
        thread::thread_sig_refs(id)
            .map(|(_, m)| { let old = *m; *m = mask; old })
            .unwrap_or(0)
    };
    cpu::interrupt_enable(cpsr);
    old
}

/// Send `signo` to `target`. The signal is added to the thread's pending mask.
/// If the thread is waiting in `signal_wait` with a matching mask, it is woken.
pub fn signal_send(target: ThreadHandle, signo: u32) -> Result<(), SignalError> {
    if signo >= MAX_SIGNALS as u32 { return Err(SignalError::InvalidSignal); }
    let cpsr = cpu::interrupt_disable();
    let id = target.0;
    unsafe {
        if let Some((pending, mask)) = thread::thread_sig_refs(id) {
            *pending |= 1u32 << signo;
            // If thread is blocked in signal_wait and this signal passes its wait mask
            let wait_bit = 1u32 << id;
            if SIG_WAITERS & wait_bit != 0 {
                let wait_mask = SIG_WAIT_MASK[id];
                if *pending & !*mask & wait_mask != 0 {
                    // Wake the thread
                    SIG_WAITERS &= !wait_bit;
                    thread::unblock_thread(id);
                }
            }
        }
    }
    cpu::interrupt_enable(cpsr);
    Ok(())
}

/// Send signal from ISR (interrupts already disabled).
pub fn signal_send_from_isr(target: ThreadHandle, signo: u32) -> bool {
    if signo >= MAX_SIGNALS as u32 { return false; }
    let id = target.0;
    unsafe {
        if let Some((pending, mask)) = thread::thread_sig_refs(id) {
            *pending |= 1u32 << signo;
            let wait_bit = 1u32 << id;
            if SIG_WAITERS & wait_bit != 0 {
                let wait_mask = SIG_WAIT_MASK[id];
                if *pending & !*mask & wait_mask != 0 {
                    SIG_WAITERS &= !wait_bit;
                    thread::unblock_thread(id);
                    return true; // reschedule may be needed
                }
            }
        }
    }
    false
}

/// Block the current thread until a pending signal (matching `wait_mask`) arrives.
/// `wait_mask`: bits indicate which signals to wake on.
/// Returns the signal number that woke the thread.
pub fn signal_wait(wait_mask: u32, timeout: Option<u32>) -> Result<u32, SignalError> {
    let cpsr = cpu::interrupt_disable();
    let id = thread::current_id();

    // Check if already pending
    let result = unsafe {
        thread::thread_sig_refs(id).and_then(|(pending, mask)| {
            let deliverable = *pending & !*mask & wait_mask;
            if deliverable != 0 {
                let signo = deliverable.trailing_zeros();
                *pending &= !(1u32 << signo);
                Some(signo)
            } else {
                None
            }
        })
    };

    if let Some(signo) = result {
        cpu::interrupt_enable(cpsr);
        // Deliver via handler if installed
        unsafe { deliver_one(id, signo); }
        return Ok(signo);
    }

    // Register as waiter
    unsafe {
        SIG_WAIT_MASK[id] = wait_mask;
        SIG_WAITERS |= 1u32 << id;
        thread::block_current(timeout);
    }

    let cpsr2 = cpu::interrupt_disable();
    let err = thread::current_ipc_error();
    if err != thread::IPC_OK {
        unsafe { SIG_WAITERS &= !(1u32 << id); }
        cpu::interrupt_enable(cpsr2);
        return Err(SignalError::Timeout);
    }

    // Find which signal woke us
    let signo = unsafe {
        thread::thread_sig_refs(id)
            .and_then(|(pending, mask)| {
                let deliverable = *pending & !*mask & wait_mask;
                if deliverable != 0 {
                    let s = deliverable.trailing_zeros();
                    *pending &= !(1u32 << s);
                    Some(s)
                } else {
                    None
                }
            })
            .unwrap_or(0)
    };

    cpu::interrupt_enable(cpsr2);
    unsafe { deliver_one(id, signo); }
    Ok(signo)
}

/// Check and deliver all pending, unmasked signals for the current thread.
/// Call periodically or after returning from a blocking operation.
pub fn signal_check() {
    let id = thread::current_id();
    loop {
        let cpsr = cpu::interrupt_disable();
        let signo = unsafe {
            thread::thread_sig_refs(id).and_then(|(pending, mask)| {
                let deliverable = *pending & !*mask;
                if deliverable != 0 {
                    let s = deliverable.trailing_zeros();
                    *pending &= !(1u32 << s);
                    Some(s)
                } else {
                    None
                }
            })
        };
        cpu::interrupt_enable(cpsr);
        match signo {
            Some(s) => unsafe { deliver_one(id, s) },
            None => break,
        }
    }
}

unsafe fn deliver_one(id: usize, signo: u32) {
    if let Some(handler) = HANDLERS[id][signo as usize] {
        handler(signo);
    }
}
