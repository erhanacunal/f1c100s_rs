//! Mutual exclusion mutex with ownership tracking.

use crate::cpu;
use crate::thread::{self, ThreadHandle};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MutexError {
    Timeout,
    NotOwner,
    AlreadyLocked,
}

const NO_OWNER: usize = 0xFF;

/// A mutual exclusion primitive. Only one thread may hold the mutex at a time.
/// The owning thread may lock recursively (counted).
pub struct Mutex {
    locked: bool,
    owner: usize,   // thread ID of current owner, or NO_OWNER
    nest: u32,      // recursive lock count
    waiters: u32,   // bitmask of waiting thread IDs
}

impl Mutex {
    pub const fn new() -> Self {
        Self { locked: false, owner: NO_OWNER, nest: 0, waiters: 0 }
    }

    /// Try to acquire the mutex without blocking.
    pub fn try_lock(&mut self) -> bool {
        let cpsr = cpu::interrupt_disable();
        let id = thread::current_id();
        let ok = if !self.locked {
            self.locked = true;
            self.owner = id;
            self.nest = 1;
            true
        } else if self.owner == id {
            self.nest += 1;
            true
        } else {
            false
        };
        cpu::interrupt_enable(cpsr);
        ok
    }

    /// Acquire the mutex, blocking if held by another thread.
    pub fn lock(&mut self, timeout: Option<u32>) -> Result<(), MutexError> {
        let cpsr = cpu::interrupt_disable();
        let id = thread::current_id();

        if !self.locked {
            self.locked = true;
            self.owner = id;
            self.nest = 1;
            cpu::interrupt_enable(cpsr);
            return Ok(());
        }
        if self.owner == id {
            self.nest += 1;
            cpu::interrupt_enable(cpsr);
            return Ok(());
        }

        // Block until the mutex is released
        self.waiters |= 1u32 << id;
        unsafe { thread::block_current(timeout); }

        let cpsr2 = cpu::interrupt_disable();
        let err = thread::current_ipc_error();
        if err != thread::IPC_OK {
            self.waiters &= !(1u32 << id);
            cpu::interrupt_enable(cpsr2);
            return Err(MutexError::Timeout);
        }
        cpu::interrupt_enable(cpsr2);
        Ok(())
    }

    /// Release the mutex. Only the owning thread may call this.
    pub fn unlock(&mut self) -> Result<(), MutexError> {
        let cpsr = cpu::interrupt_disable();
        let id = thread::current_id();

        if self.owner != id {
            cpu::interrupt_enable(cpsr);
            return Err(MutexError::NotOwner);
        }
        self.nest -= 1;
        if self.nest > 0 {
            cpu::interrupt_enable(cpsr);
            return Ok(());
        }

        if self.waiters != 0 {
            let waiter_id = self.waiters.trailing_zeros() as usize;
            self.waiters &= !(1u32 << waiter_id);
            self.owner = waiter_id;
            self.nest = 1;
            unsafe { thread::unblock_thread(waiter_id); }
        } else {
            self.locked = false;
            self.owner = NO_OWNER;
            self.nest = 0;
        }
        cpu::interrupt_enable(cpsr);
        Ok(())
    }

    pub fn is_locked(&self) -> bool { self.locked }
    pub fn owner(&self) -> Option<ThreadHandle> {
        if self.owner == NO_OWNER { None } else { Some(ThreadHandle(self.owner)) }
    }
}
