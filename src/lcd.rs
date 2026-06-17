//! LCD driver for Allwinner F1C100s
//!
//! The display pipeline consists of three hardware blocks:
//! - **TCON** (Timing Controller, `0x01C0C000`) — generates HSYNC/VSYNC/DE/CLK
//! - **DEBE** (Display Engine Backend, `0x01E60000`) — layer compositing, output
//! - **DEFE** (Display Engine Frontend, `0x01E00000`) — input scaling (bypassed)
//!
//! Supported interfaces: parallel RGB, serial RGB, CPU 8080
//! Supported color modes: RGB565, RGB888, ARGB8888, palette modes

use crate::{clock, gpio};
use crate::gpio::{Port, DriveLevel, PullMode};

// ── Base Addresses ──────────────────────────────────────────────────────

const TCON_BASE: u32 = 0x01C0C000;
const DEBE_BASE: u32 = 0x01E60000;

// ── MMIO Helpers ────────────────────────────────────────────────────────

unsafe fn read(addr: u32) -> u32 { (addr as *const u32).read_volatile() }
unsafe fn write(addr: u32, val: u32) { (addr as *mut u32).write_volatile(val); }
unsafe fn set_bits(addr: u32, mask: u32) { write(addr, read(addr) | mask); }
unsafe fn clear_bits(addr: u32, mask: u32) { write(addr, read(addr) & !mask); }

// ── Panel Descriptor ────────────────────────────────────────────────────

/// Parallel bus interface mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusMode {
    ParallelRgb,
    SerialRgb,
    SerialYuv,
    Cpu8080,
}

/// 8080 bus data width mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bus8080Mode {
    Mode18Bit256k = 0,
    Mode16Bit0 = 1,
    Mode16Bit1 = 2,
    Mode16Bit2 = 3,
    Mode16Bit3 = 4,
    Mode9Bit = 5,
    Mode8Bit256k = 6,
    Mode8Bit65k = 7,
}

/// LCD panel timing and configuration descriptor
#[derive(Clone)]
pub struct Panel {
    pub name: &'static str,
    pub width: u32,
    pub height: u32,
    /// Data bus width in bits (e.g. 8, 16, 18, 24)
    pub bus_width: u32,
    /// Parallel/serial/8080 interface mode
    pub bus_mode: BusMode,
    /// 8080 sub-mode (only used when bus_mode == Cpu8080)
    pub bus_8080_type: Bus8080Mode,
    /// Target pixel clock in Hz
    pub pixel_clock_hz: u32,
    pub h_front_porch: u32,
    pub h_back_porch: u32,
    pub h_sync_len: u32,
    pub v_front_porch: u32,
    pub v_back_porch: u32,
    pub v_sync_len: u32,
    /// Bits per pixel going over the bus (e.g. 16, 18, 24)
    pub bus_bits_per_pixel: u32,
    /// Invert HSYNC polarity
    pub h_sync_inv: bool,
    /// Invert VSYNC polarity
    pub v_sync_inv: bool,
    /// Invert data enable polarity
    pub data_enable_inv: bool,
    /// Invert pixel clock polarity
    pub clock_inv: bool,
    /// Optional panel-specific initialization callback.
    /// Called after TCON+DEBE setup but before output enable.
    /// Used for panels that need SPI/I2C init sequences (e.g. TL021WVC04).
    pub panel_init: Option<fn()>,
}

impl Panel {
    /// Helper to create a typical parallel RGB panel descriptor
    #[allow(clippy::too_many_arguments)]
    pub const fn rgb_parallel(
        name: &'static str,
        width: u32,
        height: u32,
        bus_width: u32,
        pixel_clock_hz: u32,
        h_front_porch: u32, h_back_porch: u32, h_sync_len: u32,
        v_front_porch: u32, v_back_porch: u32, v_sync_len: u32,
        bus_bits_per_pixel: u32,
        h_sync_inv: bool, v_sync_inv: bool,
        data_enable_inv: bool, clock_inv: bool,
    ) -> Self {
        Self {
            name, width, height, bus_width,
            bus_mode: BusMode::ParallelRgb,
            bus_8080_type: Bus8080Mode::Mode18Bit256k,
            pixel_clock_hz,
            h_front_porch, h_back_porch, h_sync_len,
            v_front_porch, v_back_porch, v_sync_len,
            bus_bits_per_pixel,
            h_sync_inv, v_sync_inv,
            data_enable_inv, clock_inv,
            panel_init: None,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  TCON — Timing Controller
// ═══════════════════════════════════════════════════════════════════════

/// TCON register offsets (relative to TCON_BASE)
pub mod tcon_reg {
    pub const CTRL: u32          = 0x00;
    pub const INT0: u32          = 0x04;
    pub const INT1: u32          = 0x08;
    pub const FRM_CTRL: u32      = 0x10;
    pub const FRM_SEED: u32      = 0x14;  // 6 × u32 seed table
    pub const FRM_TABLE: u32     = 0x2C;  // 4 × u32 dither table

    pub const TCON0_CTRL: u32        = 0x40;
    pub const TCON0_DCLK: u32        = 0x44;
    pub const TCON0_TIMING_ACT: u32  = 0x48;
    pub const TCON0_TIMING_H: u32    = 0x4C;
    pub const TCON0_TIMING_V: u32    = 0x50;
    pub const TCON0_TIMING_SYNC: u32 = 0x54;
    pub const TCON0_HV_INTF: u32     = 0x58;
    pub const TCON0_CPU_INTF: u32    = 0x60;
    pub const TCON0_CPU_WR_DAT: u32  = 0x64;
    pub const TCON0_CPU_RD_DAT0: u32 = 0x68;
    pub const TCON0_CPU_RD_DAT1: u32 = 0x6C;
    pub const TCON0_IO_POLARITY: u32 = 0x88;
    pub const TCON0_IO_TRISTATE: u32 = 0x8C;

    pub const TCON1_CTRL: u32         = 0x90;
    pub const TCON1_TIMING_SRC: u32   = 0x94;
    pub const TCON1_TIMING_SCALE: u32 = 0x98;
    pub const TCON1_TIMING_OUT: u32   = 0x9C;
    pub const TCON1_TIMING_H: u32     = 0xA0;
    pub const TCON1_TIMING_V: u32     = 0xA4;
    pub const TCON1_TIMING_SYNC: u32  = 0xA8;
    pub const TCON1_IO_POLARITY: u32  = 0xF0;
    pub const TCON1_IO_TRISTATE: u32  = 0xF4;
    pub const DEBUG: u32              = 0xFC;
}

// ═══════════════════════════════════════════════════════════════════════
//  DEBE — Display Engine Backend
// ═══════════════════════════════════════════════════════════════════════

/// DEBE register offsets (relative to DEBE_BASE)
pub mod debe_reg {
    pub const MODE: u32        = 0x0800;
    pub const BACKCOLOR: u32   = 0x0804;
    pub const REGBUF_CTRL: u32 = 0x0870;
    pub const CKEY_MAX: u32    = 0x0880;
    pub const CKEY_MIN: u32    = 0x0884;
    pub const CKEY_CFG: u32    = 0x0888;
    pub const PALETTE: u32     = 0x1000;

    // Per-layer offsets (layer 0–3)
    pub const LAY_SIZE: u32   = 0x0810;  // + layer*0x10
    pub const LAY_POS: u32    = 0x0820;  // + layer*0x10
    pub const LAY_STRIDE: u32 = 0x0840;  // + layer*0x04
    pub const LAY_ADDR_L: u32 = 0x0850;  // + layer*0x04
    pub const LAY_ADDR_H: u32 = 0x0860;  // + layer*0x04
    pub const LAY_ATTR0: u32  = 0x0890;  // + layer*0x10
    pub const LAY_ATTR1: u32  = 0x08A0;  // + layer*0x10

    pub fn lay_size(layer: u8) -> u32 { LAY_SIZE + (layer as u32) * 0x04 }
    pub fn lay_pos(layer: u8) -> u32 { LAY_POS + (layer as u32) * 0x04 }
    pub fn lay_stride(layer: u8) -> u32 { LAY_STRIDE + (layer as u32) * 0x04 }
    pub fn lay_addr_l(layer: u8) -> u32 { LAY_ADDR_L + (layer as u32) * 0x04 }
    pub fn lay_addr_h(layer: u8) -> u32 { LAY_ADDR_H + (layer as u32) * 0x04 }
    pub fn lay_attr0(layer: u8) -> u32 { LAY_ATTR0 + (layer as u32) * 0x04 }
    pub fn lay_attr1(layer: u8) -> u32 { LAY_ATTR1 + (layer as u32) * 0x04 }
}

// ── DEBE Color Modes ────────────────────────────────────────────────────

/// Bits-per-pixel
const BPP_16: u32 = 16 << 8;
const BPP_32: u32 = 32 << 8;

/// DEBE color mode enum
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColorMode {
    Rgb565 = 5,        // 16 bpp RGB 5-6-5
    Rgb888 = 9,        // 32 bpp XRGB 8-8-8
    Argb8888 = 10,     // 32 bpp ARGB 8-8-8-8
}

impl ColorMode {
    fn reg_value(self) -> u32 {
        match self {
            ColorMode::Rgb565  => 5  | BPP_16,
            ColorMode::Rgb888  => 9  | BPP_32,
            ColorMode::Argb8888 => 10 | BPP_32,
        }
    }

    pub fn bpp(self) -> u32 {
        match self {
            ColorMode::Rgb565  => 16,
            ColorMode::Rgb888 | ColorMode::Argb8888 => 32,
        }
    }
}

/// Register update mode
#[derive(Debug, Clone, Copy)]
pub enum UpdateMode {
    Auto = 0,
    Manual = 3,
}

// ── DEBE Operations ─────────────────────────────────────────────────────

/// Set background color (ARGB)
pub fn debe_set_bg_color(color: u32) {
    unsafe { write(DEBE_BASE + debe_reg::BACKCOLOR, color); }
}

/// Enable a layer
pub fn debe_layer_enable(layer: u8) {
    assert!(layer < 4);
    unsafe { set_bits(DEBE_BASE + debe_reg::MODE, 1 << (layer as u32 + 8)); }
}

/// Disable a layer
pub fn debe_layer_disable(layer: u8) {
    assert!(layer < 4);
    unsafe { clear_bits(DEBE_BASE + debe_reg::MODE, 1 << (layer as u32 + 8)); }
}

/// Set layer position (top-left corner)
pub fn debe_layer_set_pos(layer: u8, x: u16, y: u16) {
    assert!(layer < 4);
    unsafe {
        write(DEBE_BASE + debe_reg::lay_pos(layer), ((y as u32) << 16) | (x as u32));
    }
}

/// Set layer size in pixels and update stride
pub fn debe_layer_set_size(layer: u8, w: u16, h: u16, bpp: u32) {
    assert!(layer < 4);
    unsafe {
        write(DEBE_BASE + debe_reg::lay_size(layer), (((h as u32) - 1) << 16) | ((w as u32) - 1));
        write(DEBE_BASE + debe_reg::lay_stride(layer), (w as u32) * bpp);
    }
}

/// Set framebuffer address for a layer
pub fn debe_layer_set_addr(layer: u8, buf: *const u8) {
    assert!(layer < 4);
    let addr = buf as u32;
    unsafe {
        write(DEBE_BASE + debe_reg::lay_addr_l(layer), addr << 3);
        write(DEBE_BASE + debe_reg::lay_addr_h(layer), addr >> 29);
    }
}

/// Configure layer color mode
pub fn debe_layer_set_mode(layer: u8, mode: ColorMode) {
    assert!(layer < 4);
    let reg_val = mode.reg_value();
    unsafe {
        // Disable palette
        clear_bits(DEBE_BASE + debe_reg::lay_attr0(layer), 1 << 22);
        // Set color mode in ATTR1
        let val = read(DEBE_BASE + debe_reg::lay_attr1(layer)) & !(0x0F << 8);
        write(DEBE_BASE + debe_reg::lay_attr1(layer), val | ((reg_val & 0x0F) << 8));
    }
}

/// Set global alpha for a layer (0 = fully transparent, 255 = fully opaque)
pub fn debe_layer_set_alpha(layer: u8, alpha: u8) {
    assert!(layer < 4);
    unsafe {
        let addr = DEBE_BASE + debe_reg::lay_attr0(layer);
        let val = read(addr) & !(0xFF << 24);
        write(addr, val | ((alpha as u32) << 24));
        if alpha != 0 {
            set_bits(addr, 1 << 0);   // enable alpha blending
        } else {
            clear_bits(addr, 1 << 0);
        }
    }
}

/// Commit register changes (for manual update mode)
pub fn debe_load(mode: UpdateMode) {
    unsafe { write(DEBE_BASE + debe_reg::REGBUF_CTRL, mode as u32); }
}

/// Initialize a single layer with size, framebuffer, color mode
pub fn debe_layer_init(
    layer: u8,
    width: u32,
    height: u32,
    fb: *const u8,
    mode: ColorMode,
    enable: bool,
) {
    assert!(layer < 4);
    debe_layer_set_size(layer, width as u16, height as u16, mode.bpp());
    debe_layer_set_pos(layer, 0, 0);
    debe_layer_set_addr(layer, fb);
    debe_layer_set_mode(layer, mode);
    debe_layer_set_alpha(layer, 255);
    if enable {
        debe_layer_enable(layer);
    } else {
        debe_layer_disable(layer);
    }
}

// ── TCON Initialisation ─────────────────────────────────────────────────

/// Configure LCD GPIO pins on port D (function 1)
fn lcd_gpio_init() {
    // PD1–PD8: LCD data lines, drive level 0, no pull
    for pin in 1..=8u8 {
        gpio::set_function(Port::D, pin, gpio::function::FUNC1);
        gpio::set_drive_level(Port::D, pin, DriveLevel::Level0);
        gpio::set_pull_mode(Port::D, pin, PullMode::Disable);
    }
    // PD10–PD15: more data lines
    for pin in 10..=15u8 {
        gpio::set_function(Port::D, pin, gpio::function::FUNC1);
        gpio::set_drive_level(Port::D, pin, DriveLevel::Level0);
        gpio::set_pull_mode(Port::D, pin, PullMode::Disable);
    }
    // PD16–PD19: data lines, drive level 1
    for pin in 16..=19u8 {
        gpio::set_function(Port::D, pin, gpio::function::FUNC1);
        gpio::set_drive_level(Port::D, pin, DriveLevel::Level1);
        gpio::set_pull_mode(Port::D, pin, PullMode::Disable);
    }
    // PD20–PD21: DE/CLK lines, drive level 1, pull up
    for pin in 20..=21u8 {
        gpio::set_function(Port::D, pin, gpio::function::FUNC1);
        gpio::set_drive_level(Port::D, pin, DriveLevel::Level1);
        gpio::set_pull_mode(Port::D, pin, PullMode::Up);
    }
}

/// Configure the TCON0 timing registers for an LCD panel
fn tcon0_init(panel: &Panel, tcon_clk_hz: u32) {
    let width = panel.width;
    let height = panel.height;

    unsafe {
        // Set blanking control (vertical total blank lines in bits [4:0])
        let v_total_blank = panel.v_front_porch + panel.v_back_porch + panel.v_sync_len;
        write(TCON_BASE + tcon_reg::TCON0_CTRL, (v_total_blank & 0x1F) << 4);

        // Pixel clock divisor: DCLK = tcon_clk / divisor
        let divisor = tcon_clk_hz / panel.pixel_clock_hz;
        write(TCON_BASE + tcon_reg::TCON0_DCLK, (0xF << 28) | divisor);

        // Active area
        write(TCON_BASE + tcon_reg::TCON0_TIMING_ACT,
              ((width - 1) << 16) | (height - 1));

        // Horizontal timing: total, back porch
        let h_bp = panel.h_sync_len + panel.h_back_porch;
        let h_total = width + panel.h_front_porch + h_bp;
        write(TCON_BASE + tcon_reg::TCON0_TIMING_H,
              ((h_total - 1) << 16) | (h_bp - 1));

        // Vertical timing: total×2, back porch
        let v_bp = panel.v_sync_len + panel.v_back_porch;
        let v_total = height + panel.v_front_porch + v_bp;
        write(TCON_BASE + tcon_reg::TCON0_TIMING_V,
              ((v_total * 2) << 16) | (v_bp - 1));

        // Sync pulse widths
        write(TCON_BASE + tcon_reg::TCON0_TIMING_SYNC,
              ((panel.h_sync_len - 1) << 16) | (panel.v_sync_len - 1));

        // Interface mode
        match panel.bus_mode {
            BusMode::Cpu8080 => {
                set_bits(TCON_BASE + tcon_reg::TCON0_CTRL, 1 << 24);
                write(TCON_BASE + tcon_reg::TCON0_HV_INTF, 0);
                write(TCON_BASE + tcon_reg::TCON0_CPU_INTF,
                      ((panel.bus_8080_type as u32) << 29) | (1 << 26));
            }
            BusMode::SerialRgb => {
                clear_bits(TCON_BASE + tcon_reg::TCON0_CTRL, 1 << 24);
                write(TCON_BASE + tcon_reg::TCON0_HV_INTF, 1 << 31);
                write(TCON_BASE + tcon_reg::TCON0_CPU_INTF, 0);
            }
            BusMode::SerialYuv => {
                clear_bits(TCON_BASE + tcon_reg::TCON0_CTRL, 1 << 24);
                write(TCON_BASE + tcon_reg::TCON0_HV_INTF, (1 << 31) | (1 << 30));
                write(TCON_BASE + tcon_reg::TCON0_CPU_INTF, 0);
            }
            BusMode::ParallelRgb => {
                clear_bits(TCON_BASE + tcon_reg::TCON0_CTRL, 1 << 24);
                write(TCON_BASE + tcon_reg::TCON0_HV_INTF, 0);
                write(TCON_BASE + tcon_reg::TCON0_CPU_INTF, 0);
            }
        }

        // Dithering for 16/18 bpp panels to improve color depth
        let bpp = panel.bus_bits_per_pixel;
        if bpp == 18 || bpp == 16 {
            // Frame seed (6 × u32)
            for i in 0..6u32 {
                write(TCON_BASE + tcon_reg::FRM_SEED + i * 4, 0x1111_1111);
            }
            // Frame dither table (4 × u32)
            write(TCON_BASE + tcon_reg::FRM_TABLE + 0, 0x0101_0000);
            write(TCON_BASE + tcon_reg::FRM_TABLE + 4, 0x1515_1111);
            write(TCON_BASE + tcon_reg::FRM_TABLE + 8, 0x5757_5555);
            write(TCON_BASE + tcon_reg::FRM_TABLE + 12, 0x7F7F_7777);

            write(TCON_BASE + tcon_reg::FRM_CTRL, (panel.bus_width << 4) | (1 << 31));
        }

        // I/O polarity
        let mut polarity = 1 << 28;  // io3
        if panel.h_sync_inv    { polarity |= 1 << 25; }  // io1
        if panel.v_sync_inv    { polarity |= 1 << 24; }  // io0
        if panel.data_enable_inv { polarity |= 1 << 27; }  // io4
        if panel.clock_inv     { polarity |= 1 << 26; }  // io2
        write(TCON_BASE + tcon_reg::TCON0_IO_POLARITY, polarity);
        write(TCON_BASE + tcon_reg::TCON0_IO_TRISTATE, 0);
    }
}

// ── DEBE Init ───────────────────────────────────────────────────────────

fn debe_init() {
    unsafe {
        // Start DEBE in manual update mode
        write(DEBE_BASE + debe_reg::MODE, 1 << 1);

        // Init all 4 layers
        for i in 0u8..4 {
            write(DEBE_BASE + debe_reg::lay_attr0(i),
                  ((i as u32) << 10) | (((i & 1) as u32) << 15));
            write(DEBE_BASE + debe_reg::lay_attr1(i), 0);
        }
    }
}

// ── TCON De-init ────────────────────────────────────────────────────────

fn tcon_deinit() {
    unsafe {
        write(DEBE_BASE + tcon_reg::CTRL, 0);    // Note: reference writes to DEBE + TCON offset
        write(DEBE_BASE + tcon_reg::INT0, 0);
        write(DEBE_BASE + tcon_reg::TCON0_DCLK, 0xF << 28);
        write(DEBE_BASE + tcon_reg::TCON0_IO_TRISTATE, 0xFFFF_FFFF);
        write(DEBE_BASE + tcon_reg::TCON1_IO_TRISTATE, 0xFFFF_FFFF);
    }
}

// ── Clock Initialisation ────────────────────────────────────────────────

fn tcon_clk_enable() {
    unsafe { set_bits(clock::CCU_BASE + 0x118, 1 << 31); }
    clock::bus_gate_enable(clock::BusGate::Lcd);
}

fn debe_clk_enable() {
    unsafe {
        set_bits(clock::CCU_BASE + 0x100, 1 << 26);   // DRAM gate
        set_bits(clock::CCU_BASE + 0x104, 1 << 31);   // BE clock
    }
    clock::bus_gate_enable(clock::BusGate::Debe);
}

fn defe_clk_enable() {
    unsafe {
        set_bits(clock::CCU_BASE + 0x100, 1 << 24);   // DRAM gate
        set_bits(clock::CCU_BASE + 0x10C, 1 << 31);   // FE clock
    }
    clock::bus_gate_enable(clock::BusGate::Defe);
}

// ═══════════════════════════════════════════════════════════════════════
//  Public API
// ═══════════════════════════════════════════════════════════════════════

/// Enable the DEBE + TCON output (starts scanning out to LCD)
pub fn output_enable() {
    unsafe {
        set_bits(TCON_BASE + tcon_reg::TCON0_CTRL, 1 << 31);
        set_bits(TCON_BASE + tcon_reg::CTRL, 1 << 31);
        set_bits(DEBE_BASE + debe_reg::MODE, 1 << 0);
    }
}

/// Disable the DEBE + TCON output
pub fn output_disable() {
    unsafe {
        clear_bits(TCON_BASE + tcon_reg::CTRL, 1 << 31);
        clear_bits(DEBE_BASE + debe_reg::MODE, 1 << 0);
    }
}

/// Block until the TCON0 timing controller next enters vertical blanking.
///
/// DEBE layer-enable / framebuffer-address registers take effect the instant
/// they are written. Writing them while the panel is actively scanning out
/// tears the frame (top half = old buffer, bottom half = new). Calling this
/// first parks the swap in the vertical-blanking gap between frames, so the
/// change is never visible mid-scanout.
///
/// Polls the TCON0 vertical-blank interrupt *flag* — the hardware sets it at the
/// start of each vblank regardless of whether the interrupt is enabled, so no
/// IRQ wiring is needed. The flag is cleared first, then awaited, so the call
/// returns at the *next* vblank edge rather than on a stale flag.
///
/// The wait is bounded by the free-running AVS millisecond counter: if the flag
/// never appears (e.g. output disabled, or the wrong channel), it degrades to an
/// immediate flip after a few frames instead of stalling indefinitely.
pub fn wait_for_vsync() {
    // sun4i/suniv TCON vblank interrupt flag in TCON_INT0 is `BIT(15 - pipe)`.
    // The RGB panel is driven by TCON0 (pipe 0) → bit 15. (Bit 14 is the TCON1
    // channel, which is unused here and never fires — polling it just times out,
    // which stalls every frame.)
    const TCON0_VB_FLAG: u32 = 1 << 15;
    // AVS counter 1 (timer block 0x01C2_0C00 + 0x88): free-running, ~0.5 ms per
    // tick, not reset by `delay_us`. Used purely as a watchdog on the poll.
    const AVS_CNT1: u32 = 0x01C2_0C88;
    // ~50 ms — comfortably longer than one frame at any sane refresh, so a
    // healthy panel always exits on the flag well before this trips.
    const TIMEOUT_TICKS: u32 = 100;
    unsafe {
        // Clear pending flags (TCON interrupts are left disabled).
        write(TCON_BASE + tcon_reg::INT0, 0);
        let start = read(AVS_CNT1);
        while read(TCON_BASE + tcon_reg::INT0) & TCON0_VB_FLAG == 0 {
            if read(AVS_CNT1).wrapping_sub(start) >= TIMEOUT_TICKS {
                break;
            }
        }
    }
}

/// Flip the framebuffer: assign a new buffer to layer 0 and return
/// the previous buffer address. Caller must clean D-cache before flipping.
pub fn flip_framebuffer(layer: u8, new_fb: *const u8) -> *const u8 {
    assert!(layer < 4);
    unsafe {
        let prev = (read(DEBE_BASE + debe_reg::lay_addr_l(layer)) >> 3) as *const u8;
        debe_layer_set_addr(layer, new_fb);
        prev
    }
}

/// Write a byte in CPU 8080 mode (command or data)
pub fn write_8080(data: u16, is_command: bool) {
    unsafe {
        // Wait for FIFO not full
        while read(TCON_BASE + tcon_reg::TCON0_CPU_INTF) & 0x00C0_0000 != 0 {}
        if is_command {
            clear_bits(TCON_BASE + tcon_reg::TCON0_CPU_INTF, 1 << 25);
        } else {
            set_bits(TCON_BASE + tcon_reg::TCON0_CPU_INTF, 1 << 25);
        }
        while read(TCON_BASE + tcon_reg::TCON0_CPU_INTF) & 0x00C0_0000 != 0 {}

        // Reformat data for the 8080 interface
        let reg = ((data as u32 & 0xFC00) << 8) | ((data as u32 & 0x0300) << 6)
                | ((data as u32 & 0x00E0) << 5) | ((data as u32 & 0x001F) << 3);
        write(TCON_BASE + tcon_reg::TCON0_CPU_WR_DAT, reg);
    }
}

/// Set CPU 8080 auto-write mode (streams framebuffer automatically)
pub fn write_8080_auto_mode(enabled: bool) {
    unsafe {
        if enabled {
            set_bits(TCON_BASE + tcon_reg::TCON0_CPU_INTF, 1 << 28);
        } else {
            clear_bits(TCON_BASE + tcon_reg::TCON0_CPU_INTF, 1 << 28);
        }
    }
}

// ── Full LCD Initialisation ─────────────────────────────────────────────

/// Fully initialise the LCD subsystem for a given panel.
///
/// This configures TCON timing, enables clocks, and sets up the DEBE.
/// After calling this, call [`start_display`] with your framebuffer.
pub fn init(panel: &Panel) {
    let tcon_clk = clock::video_pll_hz();

    // Assert reset on all 3 blocks
    clock::sw_reset_assert(clock::BusGate::Defe);
    clock::sw_reset_assert(clock::BusGate::Debe);
    clock::sw_reset_assert(clock::BusGate::Lcd);

    // Enable clocks
    defe_clk_enable();
    debe_clk_enable();
    tcon_clk_enable();

    // De-assert reset
    clock::sw_reset_deassert(clock::BusGate::Defe);
    clock::sw_reset_deassert(clock::BusGate::Debe);
    clock::sw_reset_deassert(clock::BusGate::Lcd);

    // Zero DEBE registers 0x0800–0x1000
    unsafe {
        let mut addr = DEBE_BASE + 0x0800;
        while addr < DEBE_BASE + 0x1000 {
            write(addr, 0);
            addr += 4;
        }
    }

    tcon_deinit();
    debe_init();
    tcon0_init(panel, tcon_clk);
}

/// Start display output with the given framebuffer on layer 0.
///
/// Call `init()` first, then this, then `output_enable()`.
pub fn start_display(framebuffer: *mut u8, panel: &Panel, color_mode: ColorMode) {
    debe_set_bg_color(0);
    debe_layer_init(0, panel.width, panel.height, framebuffer, color_mode, true);
    debe_load(UpdateMode::Manual);
    output_enable();
    debe_set_bg_color(0);
    debe_load(UpdateMode::Auto);
}

/// Convenience: full LCD bring-up with GPIO init.
///
/// This calls `lcd_gpio_init()`, `init()`, then `start_display()`.
/// If the panel has a `panel_init` callback, it is called after `init()`
/// and before `start_display()`.
pub fn init_all(panel: &Panel, framebuffer: *mut u8, color_mode: ColorMode) {
    lcd_gpio_init();
    init(panel);
    if let Some(init_fn) = panel.panel_init {
        init_fn();
    }
    start_display(framebuffer, panel, color_mode);
}
