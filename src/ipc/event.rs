//! Event flags — 32-bit flag word with thread blocking.
//!
//! A thread can wait for:
//! - **OR mode**: any of the specified bits to be set
//! - **AND mode**: all of the specified bits to be set
//!
//! On wake-up the matched flags are optionally cleared (`clear_on_exit`).

use crate::cpu;
use crate::thread::{self, MAX_THREADS};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventMode {
    /// Wake when ANY of the requested flags are set
    Or,
    /// Wake when ALL of the requested flags are set
    And,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventError {
    Timeout,
}

/// Per-waiter state, stored externally so we can check conditions on set().
struct Waiter {
    flags: u32,
    mode: EventMode,
    clear_on_exit: bool,
}

/// Event flags object. Up to MAX_THREADS threads can wait simultaneously.
pub struct EventFlags {
    flags: u32,
    /// Bitmask of threads currently waiting on this event
    waiters: u32,
    /// Per-thread wait parameters (indexed by thread ID)
    wait_params: [Option<Waiter>; MAX_THREADS],
}

impl EventFlags {
    pub const fn new() -> Self {
        Self {
            flags: 0,
            waiters: 0,
            wait_params: [const { None }; MAX_THREADS],
        }
    }

    /// Set flag bits. Wakes any thread whose wait condition is now satisfied.
    pub fn set(&mut self, flags: u32) {
        let cpsr = cpu::interrupt_disable();
        self.flags |= flags;
        self.check_waiters();
        cpu::interrupt_enable(cpsr);
    }

    /// Clear flag bits.
    pub fn clear(&mut self, flags: u32) {
        let cpsr = cpu::interrupt_disable();
        self.flags &= !flags;
        cpu::interrupt_enable(cpsr);
    }

    /// Read current flags (non-blocking).
    pub fn get(&self) -> u32 {
        self.flags
    }

    /// Wait for flag bits.
    ///
    /// Returns the flags that caused the wakeup (before optional clear).
    pub fn wait(
        &mut self,
        flags: u32,
        mode: EventMode,
        clear_on_exit: bool,
        timeout: Option<u32>,
    ) -> Result<u32, EventError> {
        let cpsr = cpu::interrupt_disable();

        // Check if condition already satisfied
        if self.condition_met(flags, mode) {
            let satisfied = self.flags & flags;
            if clear_on_exit { self.flags &= !flags; }
            cpu::interrupt_enable(cpsr);
            return Ok(satisfied);
        }

        // Register as a waiter
        let id = thread::current_id();
        self.waiters |= 1u32 << id;
        self.wait_params[id] = Some(Waiter { flags, mode, clear_on_exit });

        // Store wait info in the thread's fields (used by check_waiters on set())
        unsafe { thread::block_current(timeout); }

        // Woken up — re-disable to get result
        let cpsr2 = cpu::interrupt_disable();
        let err = thread::current_ipc_error();
        if err != thread::IPC_OK {
            self.waiters &= !(1u32 << id);
            self.wait_params[id] = None;
            cpu::interrupt_enable(cpsr2);
            return Err(EventError::Timeout);
        }

        let satisfied = self.flags & flags;
        if clear_on_exit { self.flags &= !flags; }
        self.wait_params[id] = None;
        cpu::interrupt_enable(cpsr2);
        Ok(satisfied)
    }

    fn condition_met(&self, flags: u32, mode: EventMode) -> bool {
        match mode {
            EventMode::Or  => self.flags & flags != 0,
            EventMode::And => self.flags & flags == flags,
        }
    }

    /// Check all waiters; wake any whose condition is now satisfied.
    fn check_waiters(&mut self) {
        let mut mask = self.waiters;
        while mask != 0 {
            let id = mask.trailing_zeros() as usize;
            mask &= mask - 1;
            if let Some(ref w) = self.wait_params[id] {
                let flags = w.flags;
                let mode = w.mode;
                let clear = w.clear_on_exit;
                if self.condition_met(flags, mode) {
                    if clear { self.flags &= !flags; }
                    self.waiters &= !(1u32 << id);
                    unsafe { thread::unblock_thread(id); }
                }
            }
        }
    }
}
