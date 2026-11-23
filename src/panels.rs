//! Pre-defined LCD panel descriptors for Allwinner F1C100s
//!
//! Each panel describes the physical LCD's timing, interface and optional
//! initialization callback. Use with [`lcd::init_all`] or [`lcd::init`].
//!
//! ## Adding a new panel
//! ```ignore
//! use f1c100s::lcd::Panel;
//! pub const MY_PANEL: Panel = Panel { ... };
//! ```

use crate::gpio::{self, DriveLevel, Port};
use crate::lcd::{Panel, BusMode, Bus8080Mode};
use crate::soft_spi::{SoftSpi, SoftSpiConfig, SpiMode};
use crate::timer::delay_ms;

// ── Default Panels ───────────────────────────────────────────────────────

/// Default 800×480 8-bit parallel RGB panel.
/// Good starting point for most 5-inch LCDs. No panel init required.
pub const DEFAULT: Panel = Panel {
    name: "default",
    width: 800,
    height: 480,
    bus_width: 8,
    bus_mode: BusMode::ParallelRgb,
    bus_8080_type: Bus8080Mode::Mode18Bit256k,
    pixel_clock_hz: 33_000_000,
    h_front_porch: 40,
    h_back_porch: 40,
    h_sync_len: 48,
    v_front_porch: 13,
    v_back_porch: 29,
    v_sync_len: 3,
    bus_bits_per_pixel: 8,
    h_sync_inv: false,
    v_sync_inv: false,
    data_enable_inv: false,
    clock_inv: false,
    panel_init: None,
};

/// TL021WVC04 480×480 parallel 18-bit RGB panel.
///
/// Needs a 9-bit software SPI initialization sequence before display
/// output; `panel_init` defaults to [`tl021wvc04_init`], which uses the
/// board wiring (SCK PE0, MOSI PE1, CS PE6). Pulse the reset line first.
///
/// ## Typical usage
/// ```ignore
/// panels::tl021wvc04_reset();
/// lcd::init_all(&panels::TL021WVC04, fb, lcd::ColorMode::Argb8888);
/// ```
pub const TL021WVC04: Panel = Panel {
    name: "tl021wvc04",
    width: 480,
    height: 480,
    bus_width: 18,  // 6 bits per color channel (R6 G6 B6)
    bus_mode: BusMode::ParallelRgb,
    bus_8080_type: Bus8080Mode::Mode18Bit256k,
    pixel_clock_hz: 16_000_000,
    h_front_porch: 24,
    h_back_porch: 18,
    h_sync_len: 6,
    v_front_porch: 2,
    v_back_porch: 11,
    v_sync_len: 4,
    bus_bits_per_pixel: 32,
    h_sync_inv: false,
    v_sync_inv: false,
    data_enable_inv: false,
    clock_inv: false,
    panel_init: Some(tl021wvc04_init),
};

/// Pulse the TL021WVC04 reset line (PE5): high, low, high, 100 ms each.
/// Needs `timer::avs_init` (for `delay_ms`).
pub fn tl021wvc04_reset() {
    gpio::set_function(Port::E, 5, gpio::function::OUTPUT);
    gpio::set_drive_level(Port::E, 5, DriveLevel::Level2);
    gpio::set_value(Port::E, 5, true);
    delay_ms(100);
    gpio::set_value(Port::E, 5, false);
    delay_ms(100);
    gpio::set_value(Port::E, 5, true);
}

/// TL021WVC04 (ST7701S-class controller) register setup over 9-bit software
/// SPI: SCK PE0, MOSI PE1, CS PE6. Ends with sleep-out, display-on and an
/// 18-bit RGB interface. Needs `timer::avs_init` (for `delay_ms`).
pub fn tl021wvc04_init() {
    let spi = SoftSpi::new(SoftSpiConfig {
        sck: (Port::E, 0),
        mosi: (Port::E, 1),
        miso: (Port::E, 1),
        delay_us: 100,
        mode: SpiMode::Mode0,
        cs: Some((Port::E, 6)),
    });
    spi.init();
    let cmd = |c: u8, data: &[u8]| {
        spi.send_9bit(c as u16, 0);
        for &d in data {
            spi.send_9bit(d as u16 | 1 << 8, 0);
        }
    };

    cmd(0xFF, &[0x77, 0x01, 0x00, 0x00, 0x10]);
    cmd(0xC0, &[0x3B, 0x00]);
    cmd(0xC1, &[0x0B, 0x02]);
    cmd(0xC2, &[0x00, 0x02]);
    cmd(0xCC, &[0x10]);
    cmd(0xCD, &[0x08]);
    cmd(0xB0, &[0x02, 0x13, 0x1B, 0x0D, 0x10, 0x05, 0x08, 0x07, 0x07, 0x24, 0x04, 0x11, 0x0E, 0x2C, 0x33, 0x1D]);
    cmd(0xB1, &[0x05, 0x13, 0x1B, 0x0D, 0x11, 0x05, 0x08, 0x07, 0x07, 0x24, 0x04, 0x11, 0x0E, 0x2C, 0x33, 0x1D]);
    cmd(0xFF, &[0x77, 0x01, 0x00, 0x00, 0x11]);
    cmd(0xB0, &[0x5D]);
    cmd(0xB1, &[0x43]);
    cmd(0xB2, &[0x81]);
    cmd(0xB3, &[0x80]);
    cmd(0xB5, &[0x43]);
    cmd(0xB7, &[0x85]);
    cmd(0xB8, &[0x20]);
    cmd(0xC1, &[0x78]);
    cmd(0xC2, &[0x78]);
    cmd(0xD0, &[0x88]);
    cmd(0xE0, &[0x00, 0x00, 0x02]);
    cmd(0xE1, &[0x03, 0xA0, 0x00, 0x00, 0x04, 0xA0, 0x00, 0x00, 0x00, 0x20, 0x20]);
    cmd(0xE2, &[0x00; 13]);
    cmd(0xE3, &[0x00, 0x00, 0x11, 0x00]);
    cmd(0xE4, &[0x22, 0x00]);
    cmd(0xE5, &[0x05, 0xEC, 0xA0, 0xA0, 0x07, 0xEE, 0xA0, 0xA0, 0, 0, 0, 0, 0, 0, 0, 0]);
    cmd(0xE6, &[0x00, 0x00, 0x11, 0x00]);
    cmd(0xE7, &[0x22, 0x00]);
    cmd(0xE8, &[0x06, 0xED, 0xA0, 0xA0, 0x08, 0xEF, 0xA0, 0xA0, 0, 0, 0, 0, 0, 0, 0, 0]);
    cmd(0xEB, &[0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00]);
    cmd(0xED, &[0xFF, 0xFF, 0xFF, 0xBA, 0x0A, 0xBF, 0x45, 0xFF, 0xFF, 0x54, 0xFB, 0xA0, 0xAB, 0xFF, 0xFF, 0xFF]);
    cmd(0xEF, &[0x10, 0x0D, 0x04, 0x08, 0x3F, 0x1F]);
    cmd(0xFF, &[0x77, 0x01, 0x00, 0x00, 0x13]);
    cmd(0xEF, &[0x08]);
    cmd(0xFF, &[0x77, 0x01, 0x00, 0x00, 0x00]);
    cmd(0x11, &[]); // sleep out
    delay_ms(120);
    cmd(0x29, &[]); // display on
    cmd(0x36, &[0x00]);
    delay_ms(50);
    cmd(0x3A, &[0x60]); // 18-bit pixels
}
