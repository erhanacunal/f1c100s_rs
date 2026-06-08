//! Generic touch input types and helpers for Allwinner F1C100s
//!
//! Provides common touch abstractions (event types, point data,
//! coordinate transformation) shared by touch controller drivers.
//!
//! For a specific touch controller, see [`crate::cst8xx`].

use crate::gpio::{self, Port, PullMode};

// ── Touch Event Types ─────────────────────────────────────────────────────

/// Touch event type reported by the controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TouchEvent {
    /// No touch / released
    #[default]
    None,
    /// Finger pressed down
    Down,
    /// Finger lifted up
    Up,
    /// Finger moved (held and dragged)
    Move,
}

impl TouchEvent {
    /// Convert CST8xx raw event code (bits [7:6] of XH register).
    /// 0 = Down, 1 = Up, 2 = Move, other = None.
    pub fn from_cst8xx(raw: u8) -> Self {
        match raw {
            0 => TouchEvent::Down,
            1 => TouchEvent::Up,
            2 => TouchEvent::Move,
            _ => TouchEvent::None,
        }
    }
}

// ── Touch Point ────────────────────────────────────────────────────────────

/// A single touch point with coordinates and event.
#[derive(Debug, Clone, Copy, Default)]
pub struct TouchPoint {
    /// X coordinate (after transformation)
    pub x: u16,
    /// Y coordinate (after transformation)
    pub y: u16,
    /// Touch event type
    pub event: TouchEvent,
    /// Touch ID (for multi-touch tracking, controller-dependent)
    pub id: u8,
}

// ── Coordinate Transform ──────────────────────────────────────────────────

/// Coordinate transformation flags
pub mod transform {
    /// Reverse (mirror) the Y axis
    pub const Y_REVERSE: u8 = 1 << 0;
    /// Reverse (mirror) the X axis
    pub const X_REVERSE: u8 = 1 << 1;
    /// Swap X and Y axes
    pub const XY_EXCHANGE: u8 = 1 << 2;
}

/// Apply coordinate transformation in-place.
///
/// - `flags`: bitmask of [`transform`] constants
/// - `range_x`, `range_y`: panel dimensions in pixels (e.g. 480×480)
pub fn coord_transform(x: &mut u16, y: &mut u16, range_x: u16, range_y: u16, flags: u8) {
    if flags & transform::XY_EXCHANGE != 0 {
        let xbuf = *y;
        let ybuf = *x;

        *x = if flags & transform::X_REVERSE != 0 {
            range_y - xbuf
        } else {
            xbuf
        };

        *y = if flags & transform::Y_REVERSE != 0 {
            range_x - ybuf
        } else {
            ybuf
        };
    } else {
        if flags & transform::X_REVERSE != 0 {
            *x = range_x - *x;
        }
        if flags & transform::Y_REVERSE != 0 {
            *y = range_y - *y;
        }
    }
}

// ── GPIO Interrupt ────────────────────────────────────────────────────────

/// Configuration for a touch controller interrupt pin.
#[derive(Debug, Clone)]
pub struct TouchIntPin {
    pub port: Port,
    pub pin: u8,
    pub pin_func: u8,
}

/// Default touch interrupt pin: PE9, function 5 (common on many boards)
pub const INT_PIN_PE9: TouchIntPin = TouchIntPin {
    port: Port::E,
    pin: 9,
    pin_func: gpio::function::FUNC5,
};

/// Configure the GPIO pin for touch interrupt (falling-edge trigger).
pub fn setup_int_pin(pin: &TouchIntPin) {
    gpio::set_function(pin.port, pin.pin, pin.pin_func);
    gpio::set_pull_mode(pin.port, pin.pin, PullMode::Disable);
    gpio::set_irq_type(pin.port, pin.pin, gpio::IrqType::NegativeEdge);
}

/// Enable the touch interrupt pin (re-enable after processing a touch).
pub fn int_enable(pin: &TouchIntPin) {
    gpio::irq_enable(pin.port, pin.pin);
}

/// Disable the touch interrupt pin (call before reading touch data).
pub fn int_disable(pin: &TouchIntPin) {
    gpio::irq_disable(pin.port, pin.pin);
}
