//! Fixed-size message queue with thread blocking.
//!
//! Messages are fixed-size byte arrays. The backing buffer is user-provided
//! (`&'static mut [u8]`) so no heap allocation is required.
//!
//! # Example
//! ```ignore
//! static mut BUF: [u8; 256] = [0u8; 256];
//! let mut q = MsgQueue::new(unsafe { &mut BUF }, 16); // 16-byte messages → 16 slots
//! q.send(b"hello, world!   ", None).ok();
//! let mut msg = [0u8; 16];
//! q.recv(&mut msg, None).ok();
//! ```

use crate::cpu;
use crate::thread;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MsgError {
    Timeout,
    /// Message size doesn't match queue's message size
    BadSize,
    /// Queue buffer is too small to hold any message
    TooSmall,
}

/// A message queue backed by a user-supplied byte buffer.
pub struct MsgQueue {
    buf: *mut u8,
    msg_size: usize,
    capacity: usize,
    head: usize,   // read index (in messages)
    tail: usize,   // write index (in messages)
    count: usize,
    /// Threads waiting to receive (queue empty)
    recv_waiters: u32,
    /// Threads waiting to send (queue full)
    send_waiters: u32,
}

unsafe impl Send for MsgQueue {}
unsafe impl Sync for MsgQueue {}

impl MsgQueue {
    /// Create a new queue. `msg_size` is the fixed size of each message in bytes.
    ///
    /// # Panics
    /// If `msg_size == 0` or the buffer can't hold at least one message.
    pub fn new(buf: &'static mut [u8], msg_size: usize) -> Self {
        assert!(msg_size > 0, "msg_size must be > 0");
        let capacity = buf.len() / msg_size;
        assert!(capacity > 0, "buffer too small");
        Self {
            buf: buf.as_mut_ptr(),
            msg_size,
            capacity,
            head: 0,
            tail: 0,
            count: 0,
            recv_waiters: 0,
            send_waiters: 0,
        }
    }

    /// Send a message. Blocks if the queue is full.
    pub fn send(&mut self, msg: &[u8], timeout: Option<u32>) -> Result<(), MsgError> {
        if msg.len() != self.msg_size {
            return Err(MsgError::BadSize);
        }
        let cpsr = cpu::interrupt_disable();

        if self.count < self.capacity {
            self.enqueue(msg);
            self.wake_recv_waiter();
            cpu::interrupt_enable(cpsr);
            return Ok(());
        }

        // Queue full — block
        let id = thread::current_id();
        self.send_waiters |= 1u32 << id;
        unsafe { thread::block_current(timeout); }

        let cpsr2 = cpu::interrupt_disable();
        let err = thread::current_ipc_error();
        if err != thread::IPC_OK {
            self.send_waiters &= !(1u32 << id);
            cpu::interrupt_enable(cpsr2);
            return Err(MsgError::Timeout);
        }
        // Space was made available by a receive — enqueue now
        self.enqueue(msg);
        self.wake_recv_waiter();
        cpu::interrupt_enable(cpsr2);
        Ok(())
    }

    /// Receive a message. Blocks if the queue is empty.
    pub fn recv(&mut self, buf: &mut [u8], timeout: Option<u32>) -> Result<(), MsgError> {
        if buf.len() != self.msg_size {
            return Err(MsgError::BadSize);
        }
        let cpsr = cpu::interrupt_disable();

        if self.count > 0 {
            self.dequeue(buf);
            self.wake_send_waiter();
            cpu::interrupt_enable(cpsr);
            return Ok(());
        }

        // Queue empty — block
        let id = thread::current_id();
        self.recv_waiters |= 1u32 << id;
        unsafe { thread::block_current(timeout); }

        let cpsr2 = cpu::interrupt_disable();
        let err = thread::current_ipc_error();
        if err != thread::IPC_OK {
            self.recv_waiters &= !(1u32 << id);
            cpu::interrupt_enable(cpsr2);
            return Err(MsgError::Timeout);
        }
        self.dequeue(buf);
        self.wake_send_waiter();
        cpu::interrupt_enable(cpsr2);
        Ok(())
    }

    /// Try to send without blocking. Returns false if full.
    pub fn try_send(&mut self, msg: &[u8]) -> bool {
        if msg.len() != self.msg_size { return false; }
        let cpsr = cpu::interrupt_disable();
        let ok = self.count < self.capacity;
        if ok {
            self.enqueue(msg);
            self.wake_recv_waiter();
        }
        cpu::interrupt_enable(cpsr);
        ok
    }

    /// Try to receive without blocking. Returns false if empty.
    pub fn try_recv(&mut self, buf: &mut [u8]) -> bool {
        if buf.len() != self.msg_size { return false; }
        let cpsr = cpu::interrupt_disable();
        let ok = self.count > 0;
        if ok {
            self.dequeue(buf);
            self.wake_send_waiter();
        }
        cpu::interrupt_enable(cpsr);
        ok
    }

    pub fn len(&self) -> usize { self.count }
    pub fn is_empty(&self) -> bool { self.count == 0 }
    pub fn is_full(&self) -> bool { self.count == self.capacity }
    pub fn capacity(&self) -> usize { self.capacity }

    fn enqueue(&mut self, msg: &[u8]) {
        let dst = self.tail * self.msg_size;
        unsafe {
            core::ptr::copy_nonoverlapping(msg.as_ptr(), self.buf.add(dst), self.msg_size);
        }
        self.tail = (self.tail + 1) % self.capacity;
        self.count += 1;
    }

    fn dequeue(&mut self, buf: &mut [u8]) {
        let src = self.head * self.msg_size;
        unsafe {
            core::ptr::copy_nonoverlapping(self.buf.add(src), buf.as_mut_ptr(), self.msg_size);
        }
        self.head = (self.head + 1) % self.capacity;
        self.count -= 1;
    }

    fn wake_recv_waiter(&mut self) {
        if self.recv_waiters != 0 {
            let id = self.recv_waiters.trailing_zeros() as usize;
            self.recv_waiters &= !(1u32 << id);
            unsafe { thread::unblock_thread(id); }
        }
    }

    fn wake_send_waiter(&mut self) {
        if self.send_waiters != 0 {
            let id = self.send_waiters.trailing_zeros() as usize;
            self.send_waiters &= !(1u32 << id);
            unsafe { thread::unblock_thread(id); }
        }
    }
}
