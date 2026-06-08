//! Counting semaphore with thread blocking.

use crate::cpu;
use crate::thread;

pub const MAX_THREADS: usize = crate::thread::MAX_THREADS;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SemError {
    Timeout,
    Overflow,
}

/// Counting semaphore. Threads block when count reaches zero.
pub struct Semaphore {
    count: i32,
    /// Bitmask of waiting thread IDs
    waiters: u32,
}

impl Semaphore {
    pub const fn new(initial: i32) -> Self {
        Self { count: initial, waiters: 0 }
    }

    /// Try to take the semaphore without blocking. Returns false if count is 0.
    pub fn try_take(&mut self) -> bool {
        let cpsr = cpu::interrupt_disable();
        let ok = self.count > 0;
        if ok { self.count -= 1; }
        cpu::interrupt_enable(cpsr);
        ok
    }

    /// Take the semaphore, blocking if count is 0.
    /// `timeout`: `None` = wait forever, `Some(ticks)` = timeout in scheduler ticks.
    pub fn take(&mut self, timeout: Option<u32>) -> Result<(), SemError> {
        let cpsr = cpu::interrupt_disable();
        if self.count > 0 {
            self.count -= 1;
            cpu::interrupt_enable(cpsr);
            return Ok(());
        }
        // Block current thread
        let id = thread::current_id();
        self.waiters |= 1u32 << id;
        unsafe { thread::block_current(timeout); }
        // Interrupts re-enabled by scheduler during switch; we're back now
        // Re-disable to check result
        let cpsr2 = cpu::interrupt_disable();
        let err = thread::current_ipc_error();
        if err != thread::IPC_OK {
            self.waiters &= !(1u32 << id);
            cpu::interrupt_enable(cpsr2);
            return Err(SemError::Timeout);
        }
        cpu::interrupt_enable(cpsr2);
        Ok(())
    }

    /// Release the semaphore, waking the highest-priority waiter if any.
    pub fn release(&mut self) -> Result<(), SemError> {
        let cpsr = cpu::interrupt_disable();
        let result = self.release_locked();
        cpu::interrupt_enable(cpsr);
        result
    }

    /// Release from ISR context (interrupts already disabled).
    pub fn release_from_isr(&mut self) -> bool {
        self.release_locked().is_ok()
    }

    fn release_locked(&mut self) -> Result<(), SemError> {
        if self.waiters != 0 {
            // Wake the lowest-ID (or highest-priority) waiter
            let waiter_id = self.waiters.trailing_zeros() as usize;
            self.waiters &= !(1u32 << waiter_id);
            unsafe { thread::unblock_thread(waiter_id); }
        } else {
            self.count += 1;
        }
        Ok(())
    }

    /// Current count value.
    pub fn count(&self) -> i32 {
        self.count
    }

    /// Number of waiting threads.
    pub fn waiter_count(&self) -> u32 {
        self.waiters.count_ones()
    }
}
