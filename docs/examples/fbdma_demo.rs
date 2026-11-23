//! Framebuffer DMA (fbdma) self-test for f1c100s
//!
//! Runs on memory-only 480×480 surfaces (no panel needed) and reports on
//! UART2 (PE7 TX / PE8 RX, 115200 8N1). Every DMA operation is repeated by
//! the CPU on a reference buffer and the two buffers are compared byte for
//! byte, which checks the pixels inside the rectangle and the untouched
//! pixels around it:
//! 1. full-screen ARGB8888 fill (one contiguous transfer) + DMA/CPU timing
//! 2. rectangle fill at odd coordinates (row chaining)
//! 3. RGB565 fill at an odd x (16-bit beats)
//! 4. blit between two surfaces
//! 5. scroll down inside one surface (overlapping, bottom-up rows)
//! 6. async fill: CPU keeps running, completion callback fires
//! 7. same-row overlapping blit to the right is rejected
//!
//! Then waits for console keys: [f] FEL mode, [r] reset.

#![no_std]
#![no_main]

use core::fmt::Write;
use f1c100s::dma::{self, Channel};
use f1c100s::fbdma::{self, Rect, Surface};
use f1c100s::lcd::ColorMode;
use f1c100s::mmu::{self, MemDesc, RW_CB, RW_NCNB};
use f1c100s::{cpu, interrupt, timer, uart};

const W: u32 = 480;
const H: u32 = 480;
const FB_BYTES: usize = (W * H * 4) as usize;
const CH: Channel = Channel::D0;

#[repr(C, align(32))]
struct Fb([u8; FB_BYTES]);

static mut FB_A: Fb = Fb([0; FB_BYTES]);
static mut FB_B: Fb = Fb([0; FB_BYTES]);
static mut REF: Fb = Fb([0; FB_BYTES]);

static mut DONE: u32 = 0;
fn on_done(_ch: Channel) {
    unsafe { DONE += 1 };
}
fn done() -> u32 {
    unsafe { core::ptr::addr_of!(DONE).read_volatile() }
}

/// AVS counter 0 (1 µs ticks, started by `timer::avs_init`)
fn avs_us() -> u32 {
    unsafe { (0x01C2_0C84 as *const u32).read_volatile() }
}

struct Out(uart::Uart);
impl Write for Out {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.0.write_str(s);
        Ok(())
    }
}

fn report(out: &mut Out, name: &str, ok: bool) {
    let _ = writeln!(out, "[{}] {}", if ok { "PASS" } else { "FAIL" }, name);
}

/// Compare, printing the first differing offset and 16 bytes of each
fn same(out: &mut Out, got: &[u8], want: &[u8]) -> bool {
    match got.iter().zip(want).position(|(x, y)| x != y) {
        None => true,
        Some(i) => {
            let i = i & !15;
            let _ = writeln!(out, "  first diff at +{:#x} (row {}, px {})", i, i / (W as usize * 4), (i % (W as usize * 4)) / 4);
            let _ = writeln!(out, "  got  {:02X?}", &got[i..(i + 16).min(got.len())]);
            let _ = writeln!(out, "  want {:02X?}", &want[i..(i + 16).min(want.len())]);
            false
        }
    }
}

fn pattern(buf: &mut [u8], seed: u32) {
    let mut x = seed;
    for b in buf.iter_mut() {
        x = x.wrapping_mul(1_103_515_245).wrapping_add(12_345);
        *b = (x >> 16) as u8;
    }
}

// ── CPU reference implementations ─────────────────────────────────────────

fn cpu_fill(buf: &mut [u8], s: &Surface, r: Rect, color: u32) {
    let bpp = s.bytes_pp as usize;
    let px = color.to_le_bytes();
    for y in r.y..r.y + r.h {
        let row = (y * s.stride + r.x * s.bytes_pp) as usize;
        for i in 0..r.w as usize {
            buf[row + i * bpp..row + (i + 1) * bpp].copy_from_slice(&px[..bpp]);
        }
    }
}

/// Copy between two different buffers
fn cpu_blit(dst: &mut [u8], d: &Surface, dx: u32, dy: u32, src: &[u8], s: &Surface, r: Rect) {
    let n = (r.w * s.bytes_pp) as usize;
    for i in 0..r.h {
        let so = ((r.y + i) * s.stride + r.x * s.bytes_pp) as usize;
        let doff = ((dy + i) * d.stride + dx * d.bytes_pp) as usize;
        dst[doff..doff + n].copy_from_slice(&src[so..so + n]);
    }
}

/// Copy inside one buffer, memmove semantics (rows bottom-up when moving down)
fn cpu_scroll(buf: &mut [u8], s: &Surface, dy: u32, r: Rect) {
    let n = (r.w * s.bytes_pp) as usize;
    let rows: &mut dyn Iterator<Item = u32> = if dy > r.y { &mut (0..r.h).rev() } else { &mut (0..r.h) };
    for i in rows {
        let so = ((r.y + i) * s.stride + r.x * s.bytes_pp) as usize;
        let doff = ((dy + i) * s.stride + r.x * s.bytes_pp) as usize;
        buf.copy_within(so..so + n, doff);
    }
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
    let _ = writeln!(out, "\r\nfbdma self-test");
    dma::init();
    cpu::interrupt_enable(cpu::MODE_SVC);

    let (a, b, r) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(FB_A.0),
            &mut *core::ptr::addr_of_mut!(FB_B.0),
            &mut *core::ptr::addr_of_mut!(REF.0),
        )
    };
    let sa = unsafe { Surface::new(a.as_ptr(), W, H, ColorMode::Argb8888) };
    let sb = unsafe { Surface::new(b.as_ptr(), W, H, ColorMode::Argb8888) };
    let sa16 = unsafe { Surface::new(a.as_ptr(), W, H, ColorMode::Rgb565) };

    // 1. Full-screen fill + timing
    pattern(a, 1);
    let t0 = avs_us();
    let res = fbdma::fill_rect(CH, &sa, sa.rect(), 0xFF20_4060);
    let t_dma = avs_us().wrapping_sub(t0);
    let t0 = avs_us();
    cpu_fill(r, &sa, sa.rect(), 0xFF20_4060);
    let t_cpu_bytes = avs_us().wrapping_sub(t0);
    let t0 = avs_us();
    {
        let words = unsafe { core::slice::from_raw_parts_mut(r.as_mut_ptr() as *mut u32, FB_BYTES / 4) };
        words.fill(0xFF20_4060);
    }
    let t_cpu = avs_us().wrapping_sub(t0);
    let ok = res.is_ok() && same(&mut out, a, r);
    report(&mut out, "full-screen fill 480x480 ARGB", ok);
    let _ = writeln!(out, "  DMA {} us, CPU u32 fill {} us (per-pixel {} us)", t_dma, t_cpu, t_cpu_bytes);

    // 2. Rectangle fill at odd coordinates
    pattern(a, 2);
    r.copy_from_slice(a);
    let rect = Rect::new(37, 53, 101, 77);
    let res = fbdma::fill_rect(CH, &sa, rect, 0x80FF_00FF);
    cpu_fill(r, &sa, rect, 0x80FF_00FF);
    let ok = res.is_ok() && same(&mut out, a, r);
    report(&mut out, "rect fill (37,53) 101x77", ok);

    // 3. RGB565 fill, odd x -> 16-bit beats
    pattern(a, 3);
    r.copy_from_slice(a);
    let rect = Rect::new(5, 9, 33, 21);
    let res = fbdma::fill_rect(CH, &sa16, rect, 0xF81F);
    cpu_fill(r, &sa16, rect, 0xF81F);
    report(&mut out, "RGB565 fill at odd x (16-bit beats)", res.is_ok() && a[..] == r[..]);

    // 4. Blit between surfaces
    pattern(a, 4);
    pattern(b, 5);
    r.copy_from_slice(b);
    let src = Rect::new(13, 7, 200, 150);
    let res = fbdma::blit(CH, &sb, 251, 300, &sa, src);
    cpu_blit(r, &sb, 251, 300, a, &sa, src);
    report(&mut out, "blit 200x150 A(13,7) -> B(251,300)", res.is_ok() && b[..] == r[..]);

    // 5. Scroll down by 10 rows inside one surface (overlapping)
    pattern(a, 6);
    r.copy_from_slice(a);
    let src = Rect::new(0, 0, W, H - 10);
    let t0 = avs_us();
    let res = fbdma::blit(CH, &sa, 0, 10, &sa, src);
    let t_scroll = avs_us().wrapping_sub(t0);
    cpu_scroll(r, &sa, 10, src);
    report(&mut out, "scroll down 10 rows (bottom-up)", res.is_ok() && a[..] == r[..]);
    let _ = writeln!(out, "  scroll {} us", t_scroll);

    // 6. Async fill: count CPU loops while the DMA works
    pattern(a, 7);
    r.copy_from_slice(a);
    let rect = Rect::new(100, 100, 300, 300);
    let before = done();
    let started = unsafe { fbdma::fill_rect_async(CH, &sa, rect, 0xFF00_FF00, Some(on_done)) };
    let mut spins: u32 = 0;
    while fbdma::is_busy(CH) {
        spins += 1;
    }
    let res = fbdma::wait(CH);
    cpu_fill(r, &sa, rect, 0xFF00_FF00);
    let ok = started.is_ok() && res.is_ok() && done() == before + 1 && same(&mut out, a, r);
    report(&mut out, "async fill + callback", ok);
    let _ = writeln!(out, "  CPU spun {} times while DMA filled 300 rows", spins);

    // 7. Overlap rejection
    let res = fbdma::blit(CH, &sa, 20, 0, &sa, Rect::new(0, 0, 100, 10));
    report(&mut out, "same-row overlap to the right rejected", res == Err(fbdma::Error::Overlap));

    out.0.write_str("Keys: [f] FEL mode  [r] reset\r\n");
    loop {
        out.0.poll_dev_keys();
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
