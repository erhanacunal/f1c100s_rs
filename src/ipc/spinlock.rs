//! Spin lock (critical section) — interrupt disable/enable on single-core ARM926.

use crate::cpu;

/// A critical section guard. Interrupts are disabled while this is held.
/// Dropped automatically at end of scope, re-enabling interrupts.
pub struct SpinLockGuard {
    cpsr: u32,
}

impl SpinLockGuard {
    fn new() -> Self {
        Self { cpsr: cpu::interrupt_disable() }
    }
}

impl Drop for SpinLockGuard {
    fn drop(&mut self) {
        cpu::interrupt_enable(self.cpsr);
    }
}

/// A zero-size spin lock. On a single-core system, locking is just masking IRQs.
pub struct SpinLock;

impl SpinLock {
    pub const fn new() -> Self { Self }

    /// Acquire the lock. Returns a guard that releases on drop.
    #[inline]
    pub fn lock(&self) -> SpinLockGuard {
        SpinLockGuard::new()
    }

    /// Execute `f` inside a critical section.
    #[inline]
    pub fn with<R, F: FnOnce() -> R>(&self, f: F) -> R {
        let _guard = self.lock();
        f()
    }
}

/// Execute a closure with interrupts disabled (no lock object needed).
#[inline]
pub fn critical<R, F: FnOnce() -> R>(f: F) -> R {
    let cpsr = cpu::interrupt_disable();
    let result = f();
    cpu::interrupt_enable(cpsr);
    result
}
