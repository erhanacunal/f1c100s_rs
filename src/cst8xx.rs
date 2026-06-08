//! CST8xx Capacitive Touch Controller driver
//!
//! Driver for the CST826 / CST820 / CST816S family of I²C touch controllers
//! commonly found on small LCD panels (e.g. TL021WVC04).
//!
//! Communicates over I²C at 7-bit address `0x15`, reporting up to 5
//! simultaneous touch points with X/Y coordinates and event type.

use crate::i2c::Twi;
use crate::touch::{TouchEvent, TouchPoint, coord_transform};

// ── Constants ─────────────────────────────────────────────────────────────

/// CST8xx I²C 7-bit slave address
pub const ADDR: u8 = 0x15;

/// Maximum number of simultaneous touches
pub const MAX_TOUCHES: usize = 5;

// ── Register Map ──────────────────────────────────────────────────────────

#[allow(dead_code)]
mod reg {
    pub const WORK_MODE: u8 = 0x00;
    pub const TOUCH_NUM: u8 = 0x02;
    pub const TOUCH_DATA: u8 = 0x03;
    pub const TOUCH1_XH: u8 = 0x03;
    pub const TOUCH1_XL: u8 = 0x04;
    pub const TOUCH1_YH: u8 = 0x05;
    pub const TOUCH1_YL: u8 = 0x06;
    pub const TOUCH2_XH: u8 = 0x09;
    pub const TOUCH2_XL: u8 = 0x10;
    pub const TOUCH2_YH: u8 = 0x11;
    pub const TOUCH2_YL: u8 = 0x12;
    pub const SLEEP: u8 = 0xA5;
    pub const FW_VERSION_LO: u8 = 0xA6;
    pub const FW_VERSION_HI: u8 = 0xA7;
    pub const MODULE_ID: u8 = 0xA8;
    pub const PROJECT_NAME: u8 = 0xA9;
    pub const CHIP_TYPE_LO: u8 = 0xAA;
    pub const CHIP_TYPE_HI: u8 = 0xAB;
    pub const CHECKSUM_LO: u8 = 0xAC;
    pub const CHECKSUM_HI: u8 = 0xAD;
    pub const IRQ_CTL: u8 = 0xFA;
    pub const MOTION_MASK: u8 = 0xEC;
    pub const DISABLE_AUTOSLEEP: u8 = 0xFE;
    pub const GESTURE_ID: u8 = 0xD3;
}

// ── IRQ Control Bits ──────────────────────────────────────────────────────

const IRQ_EN_TOUCH: u8 = 1 << 6;
#[allow(dead_code)]
const IRQ_EN_CHANGE: u8 = 1 << 5;

// ── Driver ────────────────────────────────────────────────────────────────

/// CST8xx touch controller driver
pub struct Cst8xx<'a> {
    twi: &'a Twi,
    range_x: u16,
    range_y: u16,
    transform_flags: u8,
}

impl<'a> Cst8xx<'a> {
    /// Create a new CST8xx driver instance using the given I²C bus.
    pub fn new(twi: &'a Twi) -> Self {
        Self {
            twi,
            range_x: 480,
            range_y: 480,
            transform_flags: 0,
        }
    }

    /// Set the display range for coordinate mapping.
    pub fn set_range(&mut self, x: u16, y: u16) {
        self.range_x = x;
        self.range_y = y;
    }

    /// Set coordinate transform flags (see [`touch::transform`]).
    pub fn set_transform(&mut self, flags: u8) {
        self.transform_flags = flags;
    }

    // ── I²C Helpers ────────────────────────────────────────────────────

    fn read_reg(&self, reg: u8) -> Result<u8, ()> {
        let mut buf = [0u8; 1];
        self.twi.write_read(ADDR, reg, &mut buf).map_err(|_| ())?;
        Ok(buf[0])
    }

    fn write_reg(&self, reg: u8, data: u8) -> Result<(), ()> {
        let buf = [reg, data];
        self.twi.write(ADDR, None, &buf).map_err(|_| ())?;
        Ok(())
    }

    fn read_regs(&self, reg: u8, buf: &mut [u8]) -> Result<(), ()> {
        self.twi.write_read(ADDR, reg, buf).map_err(|_| ())?;
        Ok(())
    }

    // ── Public API ─────────────────────────────────────────────────────

    /// Probe whether a CST8xx chip is present on the bus.
    pub fn probe(&self) -> bool {
        let hi = self.read_reg(reg::CHIP_TYPE_HI);
        let lo = self.read_reg(reg::CHIP_TYPE_LO);
        hi.is_ok() && lo.is_ok()
    }

    /// Initialize the CST8xx chip: enable touch IRQ, disable motion mask,
    /// disable auto-sleep.
    pub fn init(&self) -> Result<(), ()> {
        self.write_reg(reg::IRQ_CTL, IRQ_EN_TOUCH)?;
        self.write_reg(reg::MOTION_MASK, 0)?;
        self.write_reg(reg::DISABLE_AUTOSLEEP, 1)?;
        Ok(())
    }

    /// Put the chip into deep sleep.
    pub fn sleep(&self) -> Result<(), ()> {
        self.write_reg(reg::SLEEP, 0x03)
    }

    /// Read the chip type identifier (2 bytes).
    pub fn chip_type(&self) -> Result<u16, ()> {
        let hi = self.read_reg(reg::CHIP_TYPE_HI)?;
        let lo = self.read_reg(reg::CHIP_TYPE_LO)?;
        Ok(u16::from(hi) << 8 | u16::from(lo))
    }

    /// Read the firmware version.
    pub fn fw_version(&self) -> Result<u16, ()> {
        let hi = self.read_reg(reg::FW_VERSION_HI)?;
        let lo = self.read_reg(reg::FW_VERSION_LO)?;
        Ok(u16::from(hi) << 8 | u16::from(lo))
    }

    /// Get the number of currently detected touch points.
    pub fn touch_count(&self) -> Result<u8, ()> {
        self.read_reg(reg::TOUCH_NUM)
    }

    /// Read a single touch point (the first detected touch).
    ///
    /// Returns `None` if no touch is active.
    /// Applies coordinate transformation based on configured flags.
    pub fn read_point(&self) -> Result<Option<TouchPoint>, ()> {
        let count = self.touch_count()?;
        if count == 0 {
            return Ok(None);
        }

        let mut raw = [0u8; 6];
        self.read_regs(reg::TOUCH_DATA, &mut raw)?;

        let mut x = u16::from(raw[0] & 0x0F) << 8 | u16::from(raw[1]);
        let mut y = u16::from(raw[2] & 0x0F) << 8 | u16::from(raw[3]);
        let event = TouchEvent::from_cst8xx(raw[0] >> 6);
        let id = raw[2] >> 4;

        coord_transform(&mut x, &mut y, self.range_x, self.range_y, self.transform_flags);

        Ok(Some(TouchPoint { x, y, event, id }))
    }

    /// Read all touch points (up to [`MAX_TOUCHES`]).
    ///
    /// Returns the array and count. Applies coordinate transformation to each.
    pub fn read_points(&self) -> Result<([TouchPoint; MAX_TOUCHES], usize), ()> {
        let count = self.touch_count()? as usize;
        if count == 0 {
            return Ok(([TouchPoint::default(); MAX_TOUCHES], 0));
        }

        let n = count.min(MAX_TOUCHES);
        let size = n * 6;
        let mut raw = [0u8; MAX_TOUCHES * 6];
        self.read_regs(reg::TOUCH_DATA, &mut raw[..size])?;

        let mut points = [TouchPoint::default(); MAX_TOUCHES];
        for i in 0..n {
            let off = i * 6;
            let mut x = u16::from(raw[off] & 0x0F) << 8 | u16::from(raw[off + 1]);
            let mut y = u16::from(raw[off + 2] & 0x0F) << 8 | u16::from(raw[off + 3]);
            let event = TouchEvent::from_cst8xx(raw[off] >> 6);
            let id = raw[off + 2] >> 4;

            coord_transform(&mut x, &mut y, self.range_x, self.range_y, self.transform_flags);
            points[i] = TouchPoint { x, y, event, id };
        }

        Ok((points, n))
    }

    /// Check if a touch is pending (shorthand for `touch_count() > 0`).
    pub fn is_touched(&self) -> Result<bool, ()> {
        Ok(self.touch_count()? > 0)
    }
}
