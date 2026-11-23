//! DMA self-test for f1c100s
//!
//! Runs on real hardware and reports on UART2 (PE7 TX / PE8 RX, 115200 8N1):
//! 0. Register-layout probe: xboot (stride 0x40, DDMA IRQ bits 16..23) vs
//!    Linux sun4i-dma suniv (stride 0x20, DDMA IRQ bits 8..15)
//! 1. Small memory copy on a dedicated channel (D0), polled, IRQs off
//! 2. Unaligned byte copy on a normal channel (N0)
//! 3. 300 KiB copy on N1 — forces the driver to split into 128 KiB chunks
//!    and continue from the completion interrupt (IRQs on)
//! 4. `on_done` callback delivery
//! 5. Throughput: DMA vs. CPU copy of 1 MiB, measured with the AVS µs counter
//!
//! This is example code — compile into your own binary crate that provides `rust_main`.

#![no_std]
#![no_main]

use core::fmt::Write;
use f1c100s::dma::{self, Channel};
use f1c100s::mmu::{self, MemDesc, RW_CB, RW_NCNB};
use f1c100s::{cpu, interrupt, timer, uart};

/// Buffers live in DRAM (cacheable) and are cache-line aligned.
#[repr(C, align(32))]
struct Buf<const N: usize>([u8; N]);

const BIG: usize = 300 * 1024;
const MIB: usize = 1024 * 1024;

static mut SRC: Buf<MIB> = Buf([0; MIB]);
static mut DST: Buf<MIB> = Buf([0; MIB]);

/// Incremented by the DMA ISR (no atomics on ARMv5TE; ISR is the only writer)
static mut DONE_CALLS: u32 = 0;

fn on_done(_ch: Channel) {
    unsafe { DONE_CALLS += 1 };
}

fn done_calls() -> u32 {
    unsafe { core::ptr::addr_of!(DONE_CALLS).read_volatile() }
}

/// AVS counter 0 (1 µs ticks, started by `timer::avs_init`)
fn avs_us() -> u32 {
    unsafe { (0x01C2_0C84 as *const u32).read_volatile() }
}

/// Raw DMA controller register access for the layout probe
const DMA_BASE: u32 = 0x01C0_2000;
fn dma_rd(off: u32) -> u32 {
    unsafe { ((DMA_BASE + off) as *const u32).read_volatile() }
}
fn dma_wr(off: u32, v: u32) {
    unsafe { ((DMA_BASE + off) as *mut u32).write_volatile(v) }
}

/// Write a pattern to each SRC-register candidate and read it back.
/// Returns how many of `offs` kept their pattern.
fn probe_regs(out: &mut Out, label: &str, offs: &[u32]) -> usize {
    let mut kept = 0;
    for &off in offs {
        let pat = 0x8000_0000 | (off << 4) | 0x5;
        dma_wr(off, pat);
        let back = dma_rd(off);
        dma_wr(off, 0);
        if back == pat {
            kept += 1;
        }
        let _ = writeln!(out, "  {} 0x{:03X}: wrote {:08X} read {:08X}", label, off, pat, back);
    }
    kept
}

/// Decide the channel stride and DDMA IRQ bit base from the hardware.
fn probe_layout(out: &mut Out) {
    let _ = writeln!(out, "Layout probe (SRC register of N1/N3/D1/D3):");
    // Valid only with stride 0x20: N1, N3, D1, D3 SRC
    let linux = probe_regs(out, "stride 0x20", &[0x124, 0x164, 0x324, 0x364]);
    // Valid only with stride 0x40: N2, N3, D2, D3 SRC (beyond the last 0x20-stride channel)
    let xboot = probe_regs(out, "stride 0x40", &[0x184, 0x1C4, 0x384, 0x3C4]);

    let stride = match (linux, xboot) {
        (4, 0) => "0x20 (Linux) -> CH_STRIDE = 0x20 is correct",
        (0, 4) => "0x40 (xboot) -> set CH_STRIDE = 0x40 in src/dma.rs",
        _ => "INCONCLUSIVE (see raw values above)",
    };
    let _ = writeln!(out, "  => channel stride: {}", stride);

    // INT_CTL is fully writable, so find the DDMA IRQ bits by running a real
    // 64-byte D1 copy (stride 0x20 -> regs at 0x320) and watching INT_STA.
    // CPU IRQs are still off here, so the bits stay pending for us to read.
    let sta = raw_d1_copy();
    let base = match sta {
        // D1 half/full = bits 18/19 (base 16) or 10/11 (base 8); either bit is enough
        x if x & 0x000C_0000 != 0 && x & 0x0000_0C00 == 0 => "16 (xboot) -> DDMA_IRQ_BASE = 16 is correct",
        x if x & 0x0000_0C00 != 0 && x & 0x000C_0000 == 0 => "8 (Linux) -> set DDMA_IRQ_BASE = 8 in src/dma.rs",
        _ => "INCONCLUSIVE",
    };
    let _ = writeln!(out, "  D1 transfer INT_STA: {:08X}", sta);
    let _ = writeln!(out, "  => DDMA IRQ base bit: {}", base);
}

#[repr(C, align(32))]
struct Small([u32; 16]);
static mut PROBE_SRC: Small = Small([0; 16]);
static mut PROBE_DST: Small = Small([0; 16]);

/// Run one 64-byte SDRAM->SDRAM copy on D1 by raw register writes, with every
/// IRQ enable bit set, and return the INT_STA bits it raised (0 on timeout).
fn raw_d1_copy() -> u32 {
    let (s, d) = unsafe {
        let s = &mut *core::ptr::addr_of_mut!(PROBE_SRC.0);
        let d = &mut *core::ptr::addr_of_mut!(PROBE_DST.0);
        for (i, w) in s.iter_mut().enumerate() { *w = 0xD1D1_0000 | i as u32; }
        (s.as_ptr() as u32, d.as_mut_ptr() as u32)
    };
    mmu::clean_dcache(s, 64);
    mmu::clean_invalidate_dcache(d, 64);

    const D1: u32 = 0x320;
    // src/dst DRQ 1 (SDRAM), linear, single, 32-bit
    let cfg = 1 | (2 << 8) | (1 << 16) | (2 << 24);
    dma_wr(0x04, 0xFFFF_FFFF); // clear stale status
    dma_wr(0x00, 0xFFFF_FFFF); // enable every IRQ bit
    dma_wr(D1 + 0x04, s);
    dma_wr(D1 + 0x08, d);
    dma_wr(D1 + 0x0C, 64);
    dma_wr(D1 + 0x18, 0);
    dma_wr(D1, cfg);
    dma_wr(D1, cfg | (1 << 31));

    let mut sta = 0;
    for _ in 0..1_000_000 {
        sta = dma_rd(0x04);
        if sta != 0 && dma_rd(D1) & (3 << 30) == 0 { break; }
    }
    dma_wr(0x00, 0);
    dma_wr(0x04, 0xFFFF_FFFF);
    sta
}

struct Out(uart::Uart);
impl Write for Out {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.0.write_str(s);
        Ok(())
    }
}

fn fill(buf: &mut [u8], seed: u32) {
    let mut x = seed;
    for b in buf.iter_mut() {
        x = x.wrapping_mul(1_103_515_245).wrapping_add(12_345);
        *b = (x >> 16) as u8;
    }
}

fn report(out: &mut Out, name: &str, ok: bool) {
    let _ = writeln!(out, "[{}] {}", if ok { "PASS" } else { "FAIL" }, name);
}

#[no_mangle]
pub extern "C" fn rust_main() -> ! {
    mmu::init(&[
        MemDesc::new(0x0000_0000, 0xFFFF_FFFF, 0x0000_0000, RW_NCNB),
        MemDesc::new(0x8000_0000, 0x81FF_FFFF, 0x8000_0000, RW_CB),
    ]);
    interrupt::init();
    timer::avs_init();
    let mut out = Out(uart::init_uart2());
    let _ = writeln!(out, "\r\nDMA self-test");
    // BROM state saved by startup.s reset (used by cpu::enter_fel)
    let brom = |off: u32| unsafe { ((0x40 + off) as *const u32).read_volatile() };
    let _ = writeln!(
        out,
        "BROM state @0x40: sp={:08X} lr={:08X} cpsr={:08X} cp15={:08X}  FEL flag={}",
        brom(0), brom(4), brom(8), brom(12), brom(0x18)
    );
    // BROM INTC setup captured by startup.s on FEL boot (restored on 'f')
    extern "C" {
        static brom_intc_save: [u32; 13];
    }
    let s = unsafe { core::ptr::addr_of!(brom_intc_save).read_volatile() };
    let _ = writeln!(
        out,
        "BROM INTC: base={:08X} en={:08X}/{:08X} mask={:08X}/{:08X}",
        s[0], s[1], s[2], s[3], s[4]
    );

    dma::init();
    probe_layout(&mut out);

    let (src, dst) = unsafe {
        (&mut *core::ptr::addr_of_mut!(SRC.0), &mut *core::ptr::addr_of_mut!(DST.0))
    };

    // 1. D0, 4 KiB, interrupts still disabled → wait() services the channel
    fill(&mut src[..4096], 1);
    dst[..4096].fill(0);
    let ok = dma::memcpy(Channel::D0, &mut dst[..4096], &src[..4096]).is_ok()
        && dst[..4096] == src[..4096];
    report(&mut out, "D0 4 KiB copy (polled)", ok);

    // 2. N0, unaligned 1001 bytes → byte-wide transfers
    fill(&mut src[..1100], 2);
    dst[..1100].fill(0);
    let ok = dma::memcpy(Channel::N0, &mut dst[3..1004], &src[1..1002]).is_ok()
        && dst[3..1004] == src[1..1002]
        && dst[..3] == [0, 0, 0]
        && dst[1004] == 0;
    report(&mut out, "N0 unaligned 1001 B copy", ok);

    // Turn interrupts on for the rest
    cpu::interrupt_enable(cpu::MODE_SVC);

    // 3. N1, 300 KiB → 3 chunks (128 + 128 + 44 KiB) chained from the ISR
    fill(&mut src[..BIG], 3);
    dst[..BIG].fill(0);
    let ok = dma::memcpy(Channel::N1, &mut dst[..BIG], &src[..BIG]).is_ok()
        && dst[..BIG] == src[..BIG];
    report(&mut out, "N1 300 KiB chunked copy (IRQ)", ok);

    // 4. Callback delivery
    let t = dma::Transfer::new(
        dma::Endpoint::memory(src.as_ptr() as u32),
        dma::Endpoint::memory(dst.as_mut_ptr() as u32),
        64 * 1024,
    )
    .on_done(on_done);
    let before = done_calls();
    unsafe { dma::start(Channel::D1, &t).unwrap() };
    dma::wait(Channel::D1);
    report(&mut out, "on_done callback", done_calls() == before + 1);

    // 5. Throughput
    let t0 = avs_us();
    dma::memcpy(Channel::D2, &mut dst[..], &src[..]).unwrap();
    let t_dma = avs_us().wrapping_sub(t0);
    let t0 = avs_us();
    dst.copy_from_slice(src);
    let t_cpu = avs_us().wrapping_sub(t0);
    let _ = writeln!(out, "1 MiB copy: DMA {} us, CPU {} us", t_dma, t_cpu);

    let _ = writeln!(out, "Keys: [f] FEL mode  [r] reset");
    loop {
        out.0.poll_dev_keys();
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
