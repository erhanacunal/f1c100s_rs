# f1c100s

A `#![no_std]`, pure-Rust hardware abstraction library (HAL) for the
**Allwinner F1C100s** system-on-chip.

The F1C100s is a low-cost SoC built around an **ARM926EJ-S** core (ARMv5TE)
with an MMU, 16 KB I-cache, 16 KB D-cache, and integrated DDR/SDR DRAM. This
crate provides bare-metal startup code, peripheral drivers, and an optional
preemptive multi-threading runtime — all written in Rust with no external
runtime dependencies beyond the `log` facade.

## Features

- **Bare-metal boot** — exception vectors, cache/MMU init, clock setup, BSS
  clearing, and per-mode stack setup, all in `startup.s`, before handing off to
  your `rust_main`.
- **Peripheral drivers** — CCU clocks, GPIO, UART, TWI/I²C, hardware & software
  SPI, SPI NOR flash, PWM, timers, INTC, DRAM, and a full LCD display pipeline
  (TCON + DEBE) with capacitive touch.
- **MMU & caches** — L1 section page tables (1 MB granularity), TLB and cache
  maintenance via CP15.
- **Preemptive multi-threading** (optional) — priority-based scheduler with a
  heap allocator and a complete IPC suite, modeled on rt-thread's ARM926 BSP.

## Module overview

### Core / runtime

| Module        | Description                                                        |
| ------------- | ------------------------------------------------------------------ |
| `cpu`         | ARM CPU mode constants, CP15 ops, and low-level hardware init      |
| `mmu`         | L1 page tables, TLB management, I/D-cache control                  |
| `interrupt`   | INTC controller (64 sources, IRQ/FIQ routing, handler dispatch)    |
| `allocator`   | Linked-list first-fit heap implementing `GlobalAlloc`              |
| `thread`      | Priority-based preemptive scheduler (32 levels, up to 32 threads)  |
| `ipc`         | Spinlock, semaphore, mutex, event flags, message queue, signals    |

### Peripherals

| Module        | Description                                                        |
| ------------- | ------------------------------------------------------------------ |
| `clock`       | Clock Control Unit (CCU): PLLs, dividers, bus gating, resets       |
| `gpio`        | GPIO ports A–F, with edge/level interrupts on ports D/E/F          |
| `uart`        | Three 16550-compatible UARTs (64-byte FIFOs)                       |
| `i2c`         | Three TWI/I²C master controllers (standard + fast mode)           |
| `spi`         | Hardware SPI (SPI0 / SPI1)                                         |
| `soft_spi`    | GPIO bit-banged SPI (incl. 9-bit mode for LCD panels)             |
| `spi_flash`   | SPI NOR flash (Winbond W25Qxx, GigaDevice GD25Qxx, …)             |
| `pwm`         | Two 16-bit PWM channels                                            |
| `timer`       | 3 general-purpose timers + watchdog block                         |
| `dram`        | DDR/SDR DRAM controller init with auto-detection                  |
| `lcd`         | Display pipeline: TCON timing + DEBE compositing                  |
| `panels`      | Pre-defined LCD panel descriptors                                 |
| `touch`       | Generic touch input types and coordinate transforms              |
| `cst8xx`      | CST826 / CST820 / CST816S capacitive touch controller driver     |

## Getting started

### Prerequisites

- **Rust nightly** (pinned via `rust-toolchain.toml`) — required for
  `build-std`, `global_asm!`, and the `linkage` feature.
- The **`rust-src`** component: `rustup component add rust-src`.
- An **`arm-none-eabi`** toolchain on your `PATH` (the custom target uses
  `arm-none-eabi-ld` as its linker).

### Build configuration

This repo ships a complete bare-metal build setup:

- `armv5te-none-eabi.json` — custom Tier-3 target (soft-float, strict-align,
  `armv5te`, no atomics, panic = abort).
- `.cargo/config.toml` — sets the default target, enables
  `build-std = ["core", "compiler_builtins"]`, and passes the linker script.
- `link.lds` — the linker script (loaded via `-Tlink.lds`).

### Build

```sh
cargo build --release
```

The target, `build-std`, and linker script are all wired up through
`.cargo/config.toml`, so a plain `cargo build` Just Works.

## Usage

Add the crate as a dependency in your binary crate and provide a `rust_main`
entry point. The startup assembly initializes the hardware and jumps to it.

```rust
#![no_std]
#![no_main]

use f1c100s::{cpu, mmu, interrupt};
use f1c100s::mmu::{MemDesc, RW_CB, RW_NCNB};

#[no_mangle]
pub extern "C" fn rust_main() -> ! {
    // Initialize MMU with a memory map
    mmu::init(&[
        MemDesc::new(0x00000000, 0xFFFF_FFFF, 0x00000000, RW_NCNB),
        MemDesc::new(0x80000000, 0x81FF_FFFF, 0x80000000, RW_CB),
    ]);

    // Bring up the interrupt controller
    interrupt::init();
    unsafe { cpu::interrupt_enable(cpu::MODE_SVC); }

    // Your application code...
    loop {}
}

#[panic_handler]
fn panic_handler(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
```

### Multi-threading

The optional runtime (`thread`, `allocator`, `ipc`) provides a preemptive,
priority-based scheduler. Stack frames match the rt-thread ARM926 BSP layout
(`[cpsr, r0–r12, lr, pc]`, 16 words). Wire `thread::sched_tick()` to your timer
ISR to enable preemption, and use the `ipc` primitives (semaphores, mutexes,
event flags, message queues, signals) for synchronization.

## Project layout

```
.
├── Cargo.toml
├── armv5te-none-eabi.json   # custom bare-metal target
├── build.rs                 # rerun triggers for linker script & startup asm
├── link.lds                 # linker script
├── rust-toolchain.toml      # pins nightly
├── .cargo/config.toml       # target, build-std, link args
└── src
    ├── lib.rs               # crate root, global_asm! includes
    ├── asm/
    │   ├── startup.s        # boot + IRQ handler
    │   └── context.s        # ARMv5TE context switch
    ├── ipc/                 # spinlock, semaphore, mutex, event, msgqueue, signal
    └── *.rs                 # peripheral & runtime modules
```

## License

Licensed under either of [MIT](LICENSE-MIT) or
[Apache-2.0](LICENSE-APACHE) at your option.
