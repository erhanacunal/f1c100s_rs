# f1c100s Architecture

## Overview

`f1c100s` is a `#![no_std]`, pure-Rust hardware abstraction library (HAL) for the
Allwinner **F1C100s** system-on-chip. The F1C100s is built around an **ARM926EJ-S**
core (ARMv5TE architecture) with:

- 16 KB I-cache, 16 KB D-cache
- Integrated MMU (L1 section page tables, 1 MB granularity)
- 32 KB internal SRAM (SRAM A at 0x00000000)
- DDR/SDR DRAM controller (up to 32 MB at 0x80000000)
- ~64 interrupt sources via INTC

## Boot Flow

```
BROM (mask ROM) loads SPL from SPI flash into SRAM
  │
  ▼
_start (0x00000000, startup.s)
  │
  ├─ Save boot params to 0x40
  ├─ Detect FEL mode (magic 0x4c45462e at 0x08 → flag at 0x58)
  ├─ Enter SVC mode, mask all interrupts
  ├─ cpu_init_crit: invalidate caches, disable MMU
  ├─ Copy exception vectors to 0x00000000
  ├─ Disable watchdog
  ├─ Mask all INTC sources
  ├─ Set temp SVC stack (top of SRAM at 0x00008000)
  │
  ▼
low_level_init (cpu.rs)
  ├─ CCU: init PLL_CPU (408 MHz), PLL_PERIPH (600 MHz)
  ├─ Set CPU_CLK, AHB_CLK, APB_CLK dividers
  ├─ Enable all bus clocks, de-assert resets
  │
  ▼
dram::init (dram.rs) — SDR/DDR auto-detect + training
  │
  ├─ If running from SRAM (adr != ldr _start):
  │    copy SPL to DRAM top (SPEEDUP_ADDR = 0x81F80000)
  │    jump there for faster execution
  │
  ├─ FEL? skip flash copy, go direct to _dram_entry
  ├─ Normal: SPI flash → DRAM → jump to _dram_entry
  │
  ▼
_dram_entry: stack setup per mode, BSS clear, → rust_main()
```

## Memory Map

| Region | Physical Address | Size | Attributes | Purpose |
|--------|-----------------|------|-----------|---------|
| SRAM A | 0x00000000 | 32 KB | NCNB | Boot SRAM, exception vectors |
| DRAM | 0x80000000 | 32 MB | RW_CB | Main RAM (code + data + heap) |
| DRAM top | 0x81F80000 | 512 KB | — | Speedup copy of SPL |
| CCU | 0x01C20000 | 4 KB | NCNB | Clock Control Unit |
| INTC | 0x01C20400 | 256 B | NCNB | Interrupt Controller |
| UART0 | 0x01C25000 | — | NCNB | Debug serial |
| Peripherals | 0x01C00000–0x01C2FFFF | — | NCNB | GPIO, SPI, I2C, timers, etc. |

## Clock Tree

```
OSC24M (24 MHz)
  │
  ├─ PLL_CPU ────── CPU_CLK (408 MHz) ── HCLKC div ── HCLKC (CPU/1..4)
  │                   factor: N×K/M                  │
  │                                                 ▼
  ├─ PLL_PERIPH ──── AHB_CLK (200 MHz) ── APB_CLK (AHB/2,4,8)
  │   (600 MHz)       div chain:        ── periph bus clocks
  │                   AHB pre-div
  │                   AHB div (2^n)
  │
  ├─ PLL_AUDIO ───── Audio codecs
  ├─ PLL_VIDEO ───── Display engine (TCON + DEBE)
  ├─ PLL_VE ──────── Video engine
  └─ PLL_DDR ─────── DRAM controller (N×K/M)
```

## MMU Design

ARMv5TE L1 page table with **section descriptors** (1 MB granularity):

- 4096 entries × 1 MB = 4 GB address space
- Table must be 16 KB aligned
- Section descriptor = `[physical_base[31:20]] | AP[11:10] | Domain[8:5] | C[3] B[2] | 1[4] 0[1:0]`

**Memory attributes (pre-composed):**

| Constant | C B | Behavior |
|----------|-----|----------|
| `RW_CB` | 1 1 | Cacheable write-back (DRAM) |
| `RW_CNB` | 1 0 | Cacheable write-through |
| `RW_NCNB` | 0 0 | Non-cacheable, non-bufferable (MMIO) |
| `RW_FAULT` | — | Access generates domain fault |

**Cache line size:** 32 bytes (8 words). Maintenance via CP15 ops.

## Crate Structure

```
f1c100s/
├── Cargo.toml              # #![no_std] lib, only dep: log 0.4
├── build.rs                # Rerun on link.lds + startup.s changes
├── link.lds                # Linker script: .text .rodata .data .bss at 0x80000000
├── armv5te-none-eabi.json  # Custom target: armv5te, soft-float, no atomics
├── rust-toolchain.toml     # Pinned nightly (build-std required)
│
├── src/
│   ├── lib.rs              # Crate root, global_asm!, module declarations
│   ├── asm/
│   │   ├── startup.s       # Boot vectors, SPL init, DRAM copy, → rust_main
│   │   └── context.s       # context_switch / context_switch_to / interrupt switch
│   │
│   ├── cpu.rs              # ARM mode consts, CPSR ops, CP15 cache/TLB/MMU ops
│   ├── mmu.rs              # PageTable, MemDesc, MMU enable/disable, cache ops
│   ├── clock.rs            # CCU: PLLs, dividers, bus gating, reset control
│   ├── interrupt.rs        # INTC: mask/unmask/install/dispatch, 64 sources
│   ├── allocator.rs        # Linked-list first-fit heap (GlobalAlloc impl)
│   │
│   ├── thread.rs           # Priority-based preemptive scheduler
│   ├── ipc/
│   │   ├── mod.rs          # Re-exports all IPC primitives
│   │   ├── spinlock.rs     # IRQ-disable critical section guard
│   │   ├── semaphore.rs    # Counting semaphore with timeout
│   │   ├── mutex.rs        # Recursive mutex with ownership
│   │   ├── event.rs        # 32-bit event flags (OR/AND wait)
│   │   ├── msgqueue.rs     # Fixed-size message queue
│   │   └── signal.rs       # Per-thread software signals (0–31)
│   │
│   ├── gpio.rs             # GPIO ports A–F, edge/level interrupts (D/E/F)
│   ├── uart.rs             # 3 UARTs (16550-compatible, 64-byte FIFO)
│   ├── i2c.rs              # 3 TWI/I²C masters (standard + fast mode)
│   ├── spi.rs              # Hardware SPI0/SPI1
│   ├── soft_spi.rs         # GPIO bit-banged SPI (incl. 9-bit LCD mode)
│   ├── spi_flash.rs        # SPI NOR flash (W25Qxx, GD25Qxx)
│   ├── pwm.rs              # 2-channel 16-bit PWM
│   ├── timer.rs            # 3 GP timers + watchdog
│   ├── dram.rs             # DRAM controller init + auto-detection
│   ├── lcd.rs              # TCON timing + DEBE compositing
│   ├── panels.rs           # Pre-defined LCD panel descriptors
│   ├── touch.rs            # Generic touch types + coordinate transforms
│   └── cst8xx.rs           # CST826/CST820/CST816S touch controller driver
```

## Interrupt System

- **INTC base:** 0x01C20400
- **64 sources** in two groups of 32
- Each source: enable, mask, pending, FIQ-select registers
- `interrupt::install(vector, handler)` — register ISR + unmask
- `interrupt::dispatch()` — called from IRQ exception handler; reads VECTOR register
- Nesting counter (`INTERRUPT_NEST`) tracks re-entrant ISRs
- `IN_ISR` flag for context awareness

**Key interrupt sources:**

| Vector | Peripheral | Vector | Peripheral |
|--------|-----------|--------|-----------|
| 0 | NMI | 13–15 | Timers 0–2 |
| 1–3 | UART 0–2 | 20 | Touch panel |
| 7–9 | TWI/I²C 0–2 | 23–24 | SD/MMC 0–1 |
| 10–11 | SPI 0–1 | 29 | TCON (LCD) |

## Allocation

- First-fit linked-list heap, protected by IRQ disable (single-core safe)
- `#[global_allocator]` via `static GLOBAL: Allocator = ALLOCATOR;`
- Call `init_heap(start, size)` once in `rust_main` before any alloc
- `free_bytes()` for diagnostics

## Build Requirements

1. **Rust nightly** (pinned via `rust-toolchain.toml`)
2. **`rust-src` component:** `rustup component add rust-src`
3. **`arm-none-eabi-` toolchain** on PATH (used as linker via custom target)
4. Build: `cargo build --release` (build-std configured in `.cargo/config.toml`)
