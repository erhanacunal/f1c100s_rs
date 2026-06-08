//! # f1c100s - Rust HAL for Allwinner F1C100s (ARM926EJ-S)
//!
//! This crate provides a `#![no_std]` hardware abstraction library for the
//! Allwinner F1C100s system-on-chip, featuring an ARM926EJ-S core (ARMv5TE)
//! with MMU, 16KB I-cache, and 16KB D-cache.
//!
//! ## Modules
//!
//! - [`cpu`] — ARM CPU mode constants, CP15 operations, and low-level hardware init
//! - [`mmu`] — MMU page table management, I/D-cache control, and cache maintenance
//! - [`interrupt`] — INTC controller management (masking, handler installation, dispatch)
//!
//! ## Boot Sequence
//!
//! The startup assembly (`startup.s`) performs:
//! 1. Exception vector table setup
//! 2. CPU critical init (invalidate caches, disable MMU)
//! 3. Copy vectors to 0x00000000 or 0xFFFF0000
//! 4. Watchdog disable
//! 5. IRQ source masking
//! 6. Clock initialization via [`cpu::low_level_init`]
//! 7. Stack setup for each ARM mode
//! 8. BSS section clearing
//! 9. Jump to `rust_main` (to be provided by your binary crate)
//!
//! ## Usage Example
//!
//! ```ignore
//! // In your binary crate's main.rs:
//! #![no_std]
//! #![no_main]
//!
//! use f1c100s::{cpu, mmu, interrupt, mmu::MemDesc, mmu::RW_CB, mmu::RW_NCNB};
//!
//! #[no_mangle]
//! pub extern "C" fn rust_main() -> ! {
//!     // Initialize MMU with memory map
//!     mmu::init(&[
//!         MemDesc::new(0x00000000, 0xFFFF_FFFF, 0x00000000, RW_NCNB),
//!         MemDesc::new(0x80000000, 0x81FF_FFFF, 0x80000000, RW_CB),
//!     ]);
//!
//!     // Initialize interrupt controller
//!     interrupt::init();
//!
//!     // Enable interrupts
//!     unsafe { cpu::interrupt_enable(cpu::MODE_SVC); }
//!
//!     // Your application code...
//!     loop {}
//! }
//!
//! #[panic_handler]
//! fn panic_handler(info: &core::panic::PanicInfo) -> ! {
//!     loop {}
//! }
//! ```
//!
//! ## Linker Script
//!
//! The library includes `link.lds` in the root. Use it with:
//! ```text
//! rustflags = ["-C", "link-arg=-Tlink.lds"]
//! ```

#![no_std]
#![allow(static_mut_refs)]
#![feature(linkage)]

use core::arch::global_asm;

// ARM startup and context switch assembly
global_asm!(include_str!("asm/startup.s"));
global_asm!(include_str!("asm/context.s"));

pub mod allocator;
pub mod clock;
pub mod cpu;
pub mod cst8xx;
pub mod dram;
pub mod gpio;
pub mod i2c;
pub mod interrupt;
pub mod lcd;
pub mod mmu;
pub mod panels;
pub mod pwm;
pub mod soft_spi;
pub mod spi;
pub mod spi_flash;
pub mod thread;
pub mod timer;
pub mod touch;
pub mod uart;
pub mod ipc;


#[no_mangle]
#[linkage = "weak"]
fn rust_main() -> ! {
    loop {}
}
