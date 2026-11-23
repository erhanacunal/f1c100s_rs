//! fbdma on the TL021WVC04 480×480 panel (Lichee Pi Nano + panel board)
//!
//! Everything on screen is drawn with DMA (`fbdma`, channel D0); the CPU only
//! decides what to draw. Scenes, repeated forever:
//! 1. full-screen colour fills: red, green, blue, white, black
//! 2. 8×8 grid of rainbow tiles (64 rectangle fills)
//! 3. endless vertical scroll: the screen moves down 4 rows per frame
//!    (overlapping blit) and a new stripe is filled in at the top
//! 4. a 96×96 sprite bouncing around: erase the old spot, blit the new one
//!
//! UART2 (115200 8N1) prints DMA time per frame for each scene.
//! Keys: [f] FEL mode, [r] reset (checked every frame).

#![no_std]
#![no_main]

use core::fmt::Write;
use f1c100s::dma::{self, Channel};
use f1c100s::fbdma::{self, Rect, Surface};
use f1c100s::lcd::{self, ColorMode};
use f1c100s::mmu::{self, MemDesc, RW_CB, RW_NCNB};
use f1c100s::pwm::{self, PwmConfig};
use f1c100s::{cpu, interrupt, panels, timer, uart};

const W: u32 = 480;
const H: u32 = 480;
const FB_BYTES: usize = (W * H * 4) as usize;
const CH: Channel = Channel::D0;

const SPR: u32 = 96;
const SPR_BYTES: usize = (SPR * SPR * 4) as usize;
const BG: u32 = 0xFF10_1828;

#[repr(C, align(32))]
struct Buf<const N: usize>([u8; N]);

static mut FB: Buf<FB_BYTES> = Buf([0; FB_BYTES]);
static mut SPRITE: Buf<SPR_BYTES> = Buf([0; SPR_BYTES]);

/// AVS counter 0 in µs. `timer::delay_*` resets it, so only time code
/// without delays in between.
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

/// Fully saturated colour on a 0..=1535 hue wheel, as 0xFFRRGGBB
fn hue(h: u32) -> u32 {
    let h = h % 1536;
    let x = h % 256;
    let (r, g, b) = match h / 256 {
        0 => (255, x, 0),
        1 => (255 - x, 255, 0),
        2 => (0, 255, x),
        3 => (0, 255 - x, 255),
        4 => (x, 0, 255),
        _ => (255, 0, 255 - x),
    };
    0xFF00_0000 | (r << 16) | (g << 8) | b
}

fn ok(r: Result<(), fbdma::Error>, out: &mut Out, what: &str) {
    if let Err(e) = r {
        let _ = writeln!(out, "[ERR] {}: {:?}", what, e);
    }
}

/// Wait for the next frame (and check the console keys)
fn frame(u: &uart::Uart) {
    u.poll_dev_keys();
    lcd::wait_for_vsync();
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
    let _ = writeln!(out, "\r\nfbdma panel demo");
    dma::init();
    cpu::interrupt_enable(cpu::MODE_SVC);

    let fb_ptr = unsafe { core::ptr::addr_of_mut!(FB.0) as *mut u8 };
    let fb = unsafe { Surface::new(fb_ptr, W, H, ColorMode::Argb8888) };
    let spr = unsafe { Surface::new(core::ptr::addr_of!(SPRITE.0) as *const u8, SPR, SPR, ColorMode::Argb8888) };
    let screen = fb.rect();

    // Black screen before the panel turns on
    ok(fbdma::fill_rect(CH, &fb, screen, 0xFF00_0000), &mut out, "clear");

    let backlight = pwm::pwm0_pe12();
    backlight
        .configure(&PwmConfig {
            period_ns: 1_000_000,
            mode: pwm::PwmMode::Continuous,
            polarity: pwm::PwmPolarity::ActiveHigh,
            pulse_ns: 800_000,
        })
        .ok();
    backlight.enable();

    panels::tl021wvc04_reset();
    lcd::init_all(&panels::TL021WVC04, fb_ptr, ColorMode::Argb8888);
    let _ = writeln!(out, "panel up; keys: [f] FEL mode  [r] reset");

    // Sprite: concentric squares, drawn once with fills
    for i in 0..6u32 {
        let m = i * 8;
        ok(
            fbdma::fill_rect(CH, &spr, Rect::new(m, m, SPR - 2 * m, SPR - 2 * m), hue(i * 256)),
            &mut out,
            "sprite",
        );
    }

    let mut round: u32 = 0;
    loop {
        // 1. Full-screen fills
        for &c in &[0xFFFF_0000, 0xFF00_FF00, 0xFF00_00FF, 0xFFFF_FFFF, 0xFF00_0000] {
            frame(&out.0);
            let t0 = avs_us();
            ok(fbdma::fill_rect(CH, &fb, screen, c), &mut out, "fill");
            let t = avs_us().wrapping_sub(t0);
            if c == 0xFFFF_0000 {
                let _ = writeln!(out, "round {}: full fill {} us", round, t);
            }
            for _ in 0..30 {
                frame(&out.0);
            }
        }

        // 2. Rainbow tile grid
        let tile = W / 8;
        let t0 = avs_us();
        for ty in 0..8 {
            for tx in 0..8 {
                let r = Rect::new(tx * tile + 2, ty * tile + 2, tile - 4, tile - 4);
                ok(fbdma::fill_rect(CH, &fb, r, hue((tx + ty) * 96 + round * 200)), &mut out, "tile");
            }
        }
        let _ = writeln!(out, "  64 tiles {} us", avs_us().wrapping_sub(t0));
        for _ in 0..60 {
            frame(&out.0);
        }

        // 3. Endless scroll down, 4 rows per frame, new stripe at the top
        const STEP: u32 = 4;
        let mut worst = 0;
        for f in 0..240u32 {
            frame(&out.0);
            let t0 = avs_us();
            ok(fbdma::blit(CH, &fb, 0, STEP, &fb, Rect::new(0, 0, W, H - STEP)), &mut out, "scroll");
            let c = if (f / 6) % 2 == 0 { hue(f * 24) } else { 0xFF00_0000 };
            ok(fbdma::fill_rect(CH, &fb, Rect::new(0, 0, W, STEP), c), &mut out, "stripe");
            worst = worst.max(avs_us().wrapping_sub(t0));
        }
        let _ = writeln!(out, "  scroll frame worst {} us", worst);

        // 4. Bouncing sprite
        ok(fbdma::fill_rect(CH, &fb, screen, BG), &mut out, "bg");
        let (mut x, mut y, mut vx, mut vy) = (40i32, 120i32, 5i32, 3i32);
        let mut worst = 0;
        for _ in 0..600 {
            frame(&out.0);
            let t0 = avs_us();
            let old = Rect::new(x as u32, y as u32, SPR, SPR);
            x += vx;
            y += vy;
            if x < 0 || x > (W - SPR) as i32 { vx = -vx; x += 2 * vx; }
            if y < 0 || y > (H - SPR) as i32 { vy = -vy; y += 2 * vy; }
            ok(fbdma::fill_rect(CH, &fb, old, BG), &mut out, "erase");
            ok(fbdma::blit(CH, &fb, x as u32, y as u32, &spr, spr.rect()), &mut out, "sprite");
            worst = worst.max(avs_us().wrapping_sub(t0));
        }
        let _ = writeln!(out, "  sprite frame worst {} us", worst);

        round += 1;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
