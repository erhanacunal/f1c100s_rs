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

use crate::lcd::{Panel, BusMode, Bus8080Mode};

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
/// Requires a software SPI initialization sequence before display output
/// can be enabled. Attach your SPI init function via `panel_init`.
///
/// ## Typical usage
/// ```ignore
/// let mut panel = panels::TL021WVC04;
/// panel.panel_init = Some(my_spi_init_fn);
/// lcd::init_all(&panel, fb, lcd::ColorMode::Rgb565);
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
    panel_init: None, // caller sets this to their SPI init function
};
