//! IPC primitives for the f1c100s multithreading runtime.
//!
//! - [`spinlock`] — Critical-section / interrupt-disable guard
//! - [`semaphore`] — Counting semaphore
//! - [`mutex`] — Mutual exclusion with ownership
//! - [`event`] — 32-bit event flags (OR / AND wait)
//! - [`msgqueue`] — Fixed-size message queue
//! - [`signal`] — Per-thread software signals

pub mod event;
pub mod msgqueue;
pub mod mutex;
pub mod semaphore;
pub mod signal;
pub mod spinlock;

pub use event::{EventError, EventFlags, EventMode};
pub use msgqueue::{MsgError, MsgQueue};
pub use mutex::{Mutex, MutexError};
pub use semaphore::{SemError, Semaphore};
pub use signal::{SignalError, signal_check, signal_install, signal_mask, signal_send,
                  signal_send_from_isr, signal_uninstall, signal_wait};
pub use spinlock::{SpinLock, SpinLockGuard, critical};
