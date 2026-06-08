//! Thread-safe linked-list heap allocator for bare-metal use.
//!
//! Implements `GlobalAlloc` so it can be used as `#[global_allocator]`.
//! All operations are protected by interrupt disable/enable (single-core safe).
//!
//! # Usage
//! ```ignore
//! use f1c100s::allocator::ALLOCATOR;
//!
//! #[global_allocator]
//! static GLOBAL: f1c100s::allocator::Allocator = f1c100s::allocator::ALLOCATOR;
//!
//! // In rust_main, before any alloc:
//! unsafe { f1c100s::allocator::init_heap(HEAP_START as *mut u8, HEAP_SIZE); }
//! ```

use core::alloc::{GlobalAlloc, Layout};
use core::ptr;
use crate::cpu;

// ── Block header ──────────────────────────────────────────────────────────────

/// Minimum alignment for all allocations (8 bytes for ARM926)
const ALIGN: usize = 8;

/// Header preceding every free block.
/// Allocated blocks only store `size` (reconstructed from raw pointer on free).
#[repr(C)]
struct FreeBlock {
    /// Total size of this free region in bytes, including this header.
    size: usize,
    /// Pointer to next free block (null = end of list).
    next: *mut FreeBlock,
}

/// Header preceding every allocated block (stored just before the user pointer).
#[repr(C)]
struct AllocHeader {
    size: usize, // total bytes from header start (including this header)
}

const FREE_HDR: usize = core::mem::size_of::<FreeBlock>();
const ALLOC_HDR: usize = core::mem::size_of::<AllocHeader>();

// ── Allocator state ───────────────────────────────────────────────────────────

static mut FREE_LIST: *mut FreeBlock = ptr::null_mut();

/// Initialize the heap. Call once from `rust_main` before any allocation.
///
/// # Safety
/// - `start` must be valid, writable, and aligned to 8 bytes
/// - `size` must be at least `FREE_HDR + 16`
/// - Must only be called once
pub unsafe fn init_heap(start: *mut u8, size: usize) {
    let start = align_up(start as usize, ALIGN) as *mut u8;
    let aligned_size = size & !(ALIGN - 1);
    if aligned_size < FREE_HDR + 16 {
        return;
    }
    let block = start as *mut FreeBlock;
    (*block).size = aligned_size;
    (*block).next = ptr::null_mut();
    FREE_LIST = block;
}

// ── Allocation helpers ────────────────────────────────────────────────────────

#[inline]
fn align_up(addr: usize, align: usize) -> usize {
    (addr + align - 1) & !(align - 1)
}

/// Allocate `needed` bytes (raw size including AllocHeader), returning user ptr.
unsafe fn alloc_inner(needed: usize) -> *mut u8 {
    let needed = align_up(needed, ALIGN);
    let mut prev: *mut *mut FreeBlock = &mut FREE_LIST;
    let mut cur = FREE_LIST;

    while !cur.is_null() {
        if (*cur).size >= needed {
            let remaining = (*cur).size - needed;
            if remaining >= FREE_HDR + ALIGN {
                // Split block: carve `needed` from front, keep remainder
                let split = (cur as usize + needed) as *mut FreeBlock;
                (*split).size = remaining;
                (*split).next = (*cur).next;
                *prev = split;
            } else {
                // Use entire block
                *prev = (*cur).next;
            }
            // Write alloc header
            let hdr = cur as *mut AllocHeader;
            (*hdr).size = if remaining >= FREE_HDR + ALIGN { needed } else { (*cur).size };
            return (cur as *mut u8).add(ALLOC_HDR);
        }
        prev = &mut (*cur).next;
        cur = (*cur).next;
    }
    ptr::null_mut()
}

/// Free a user pointer previously returned by `alloc_inner`.
unsafe fn free_inner(ptr: *mut u8) {
    if ptr.is_null() {
        return;
    }
    let hdr = ptr.sub(ALLOC_HDR) as *mut AllocHeader;
    let size = (*hdr).size;
    let block = hdr as *mut FreeBlock;
    (*block).size = size;

    // Insert into free list (sorted by address for coalescing)
    let mut prev: *mut *mut FreeBlock = &mut FREE_LIST;
    let mut cur = FREE_LIST;

    while !cur.is_null() && (cur as usize) < (block as usize) {
        prev = &mut (*cur).next;
        cur = (*cur).next;
    }

    (*block).next = cur;
    *prev = block;

    // Coalesce forward: block → cur
    if !cur.is_null() && (block as usize + (*block).size == cur as usize) {
        (*block).size += (*cur).size;
        (*block).next = (*cur).next;
    }

    // Coalesce backward: *prev → block
    let prev_block = *prev as *mut FreeBlock;
    if !prev_block.is_null() && prev_block != block {
        if prev_block as usize + (*prev_block).size == block as usize {
            (*prev_block).size += (*block).size;
            (*prev_block).next = (*block).next;
        }
    }
}

// ── Public allocator type ─────────────────────────────────────────────────────

#[derive(Clone, Copy)]
pub struct Allocator;

pub static ALLOCATOR: Allocator = Allocator;

unsafe impl GlobalAlloc for Allocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let align = layout.align().max(ALIGN);
        let size = align_up(layout.size() + ALLOC_HDR + align - 1, ALIGN);

        let cpsr = cpu::interrupt_disable();
        let ptr = alloc_inner(size);
        cpu::interrupt_enable(cpsr);

        if ptr.is_null() {
            return ptr::null_mut();
        }
        // Align the returned pointer within the allocated block
        let aligned = align_up(ptr as usize, align);
        // If we needed extra padding, note that the actual block starts earlier;
        // for simplicity we over-allocate to guarantee alignment
        aligned as *mut u8
    }

    unsafe fn dealloc(&self, ptr: *mut u8, _layout: Layout) {
        let cpsr = cpu::interrupt_disable();
        free_inner(ptr);
        cpu::interrupt_enable(cpsr);
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let ptr = self.alloc(layout);
        if !ptr.is_null() {
            ptr::write_bytes(ptr, 0, layout.size());
        }
        ptr
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let new_layout = Layout::from_size_align_unchecked(new_size, layout.align());
        let new_ptr = self.alloc(new_layout);
        if !new_ptr.is_null() {
            let copy_size = layout.size().min(new_size);
            ptr::copy_nonoverlapping(ptr, new_ptr, copy_size);
            self.dealloc(ptr, layout);
        }
        new_ptr
    }
}

/// Returns the total free bytes remaining in the heap (approximate, for diagnostics).
pub fn free_bytes() -> usize {
    let cpsr = cpu::interrupt_disable();
    let mut total = 0usize;
    unsafe {
        let mut cur = FREE_LIST;
        while !cur.is_null() {
            total += (*cur).size;
            cur = (*cur).next;
        }
    }
    cpu::interrupt_enable(cpsr);
    total
}
