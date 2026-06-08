//! PWM (Pulse Width Modulation) driver for Allwinner F1C100s
//!
//! The F1C100s provides two PWM channels with:
//! - 16-bit period and duty cycle resolution
//! - Configurable prescaler (from 1 to 72000)
//! - Continuous or single-pulse modes
//! - Polarity control (active high/low)
//! - Bypass mode (direct 24 MHz output)
//!
//! ## Clock Derivation
//! PWM_CLK = 24MHz / prescaler
//! Period = (PRD + 1) / PWM_CLK  where PRD is a 16-bit value (0..65535)
//! Duty   = (DTY) / PWM_CLK      where DTY is a 16-bit value (0..65535)
//!
//! ## Base Address
//! PWM_BASE: `0x01C21000`
//!
//! ## Pin Mapping
//! - PWM0: PA2 (func2) or PE12 (func3)
//! - PWM1: PE6 (func2) or PF5 (func3)

use crate::gpio::{self, Port, PullMode};

// ── Base Address ──────────────────────────────────────────────────────────

const PWM_BASE: u32 = 0x01C21000;

// ── Register Offsets ──────────────────────────────────────────────────────

const PWM_CTRL_REG: u32 = 0x00;
const PWM_CH0_PRD: u32 = 0x04;
const PWM_CH1_PRD: u32 = 0x08;

// ── CTRL Register Bit-Field Helpers ──────────────────────────────────────

/// Each channel occupies 15 bits in the CTRL register.
const CH_OFFSET: u32 = 15;

/// Prescaler field mask (lower 4 bits)
const PRESCAL_MASK: u32 = 0xF;

/// Write a channel-specific bit field into a CTRL register value
const fn ctrl_bit(bit: u32, ch: u32) -> u32 {
    bit << (ch * CH_OFFSET)
}

const PWM_EN: u32 = 1 << 4;
const PWM_ACT_STATE: u32 = 1 << 5;
const PWM_CLK_GATING: u32 = 1 << 6;
const PWM_MODE: u32 = 1 << 7;
#[allow(dead_code)]
const PWM_PULSE: u32 = 1 << 8;
#[allow(dead_code)]
const PWM_BYPASS: u32 = 1 << 9;

/// Ready bit per channel (for checking if period update took effect)
#[allow(dead_code)]
const RDY_BASE: u32 = 28;
#[allow(dead_code)]
const fn pwm_rdy(ch: u32) -> u32 {
    1 << (RDY_BASE + ch)
}

// ── Period/Duty Register Helpers ─────────────────────────────────────────

const PRD_MASK: u32 = 0xFFFF;
const DTY_MASK: u32 = 0xFFFF;

/// Pack period-1 and duty into a CHx register value
const fn prd_duty_pack(prd: u32, dty: u32) -> u32 {
    ((prd - 1) & PRD_MASK) << 16 | (dty & DTY_MASK)
}

// ── Prescaler Table ──────────────────────────────────────────────────────

/// Prescaler divisor table indexed by the 4-bit prescaler field.
/// Value 15 is special: treated as divide-by-1.
const PRESCALER_TABLE: [u32; 16] = [
    120,   // 0
    180,   // 1
    240,   // 2
    360,   // 3
    480,   // 4
    0,     // 5 - invalid
    0,     // 6 - invalid
    0,     // 7 - invalid
    12000, // 8
    24000, // 9
    36000, // 10
    48000, // 11
    72000, // 12
    0,     // 13 - invalid
    0,     // 14 - invalid
    1,     // 15 - ÷1 (special)
];

/// 24 MHz source clock
const PWM_SRC_HZ: u32 = 24_000_000;

/// Nanoseconds per second
const NSEC_PER_SEC: u64 = 1_000_000_000;

// ── MMIO Helpers ──────────────────────────────────────────────────────────

#[inline]
unsafe fn pwm_read(offset: u32) -> u32 {
    ((PWM_BASE + offset) as *const u32).read_volatile()
}

#[inline]
unsafe fn pwm_write(offset: u32, val: u32) {
    ((PWM_BASE + offset) as *mut u32).write_volatile(val);
}

// ── PWM Channel ───────────────────────────────────────────────────────────

/// PWM channel identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PwmChannel {
    Ch0 = 0,
    Ch1 = 1,
}

/// PWM polarity / active state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PwmPolarity {
    /// Active high (normal PWM)
    ActiveHigh = 0,
    /// Active low (inverted)
    ActiveLow = 1,
}

/// PWM operating mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PwmMode {
    /// Continuous PWM output
    Continuous = 0,
    /// Single pulse then stop
    SinglePulse = 1,
}

/// PWM configuration
#[derive(Debug, Clone)]
pub struct PwmConfig {
    /// Period in nanoseconds
    pub period_ns: u32,
    /// Duty cycle active time in nanoseconds (must be ≤ period_ns)
    pub pulse_ns: u32,
    /// Polarity / active state
    pub polarity: PwmPolarity,
    /// Operating mode
    pub mode: PwmMode,
}

impl Default for PwmConfig {
    fn default() -> Self {
        Self {
            period_ns: 1_000_000, // 1 ms → 1 kHz
            pulse_ns: 500_000,    // 50% duty
            polarity: PwmPolarity::ActiveHigh,
            mode: PwmMode::Continuous,
        }
    }
}

/// Pin-mux configuration for a PWM channel
#[derive(Debug, Clone)]
pub struct PwmPins {
    pub port: Port,
    pub pin: u8,
    pub pin_func: u8,
}

/// Preset pin configurations
pub mod pins {
    use super::*;

    /// PWM0 on PA2 (func2)
    pub const PWM0_PA2: PwmPins = PwmPins {
        port: Port::A,
        pin: 2,
        pin_func: gpio::function::FUNC2,
    };

    /// PWM0 on PE12 (func3)
    pub const PWM0_PE12: PwmPins = PwmPins {
        port: Port::E,
        pin: 12,
        pin_func: gpio::function::FUNC3,
    };

    /// PWM1 on PE6 (func2)
    pub const PWM1_PE6: PwmPins = PwmPins {
        port: Port::E,
        pin: 6,
        pin_func: gpio::function::FUNC2,
    };

    /// PWM1 on PF5 (func3)
    pub const PWM1_PF5: PwmPins = PwmPins {
        port: Port::F,
        pin: 5,
        pin_func: gpio::function::FUNC3,
    };
}

/// A configured PWM channel instance
pub struct Pwm {
    ch: u32,
}

impl Pwm {
    /// Create a new PWM instance for the given channel.
    ///
    /// # Safety
    /// Caller must ensure no other code is accessing the same channel concurrently.
    pub unsafe fn new(ch: PwmChannel) -> Self {
        Self { ch: ch as u32 }
    }

    /// Initialize the PWM channel: configure GPIO pin mux, disable pull,
    /// and disable the PWM output (safe startup state).
    pub fn init(&self, pins: &PwmPins) {
        gpio::set_function(pins.port, pins.pin, pins.pin_func);
        gpio::set_pull_mode(pins.port, pins.pin, PullMode::Disable);
        self.disable();
    }

    // ── Enable / Disable ───────────────────────────────────────────────

    /// Disable the PWM channel (gate clock + disable output)
    pub fn disable(&self) {
        unsafe {
            let val = pwm_read(PWM_CTRL_REG);
            let val = val & !ctrl_bit(PWM_CLK_GATING, self.ch);
            let val = val & !ctrl_bit(PWM_EN, self.ch);
            pwm_write(PWM_CTRL_REG, val);
        }
    }

    /// Enable the PWM channel (enable clock + output)
    pub fn enable(&self) {
        unsafe {
            let val = pwm_read(PWM_CTRL_REG);
            let val = val | ctrl_bit(PWM_CLK_GATING, self.ch);
            let val = val | ctrl_bit(PWM_EN, self.ch);
            pwm_write(PWM_CTRL_REG, val);
        }
    }

    /// Check whether the PWM channel is currently enabled
    pub fn is_enabled(&self) -> bool {
        unsafe {
            let val = pwm_read(PWM_CTRL_REG);
            let mask = ctrl_bit(PWM_CLK_GATING | PWM_EN, self.ch);
            (val & mask) == mask
        }
    }

    // ── Read Current Configuration ─────────────────────────────────────

    /// Read the current configuration (period, pulse, polarity)
    pub fn get_config(&self) -> PwmConfig {
        unsafe {
            let ctrl = pwm_read(PWM_CTRL_REG);
            let prescal_idx = (ctrl >> (self.ch * CH_OFFSET)) & PRESCAL_MASK;

            let prescaler = if prescal_idx == PRESCAL_MASK {
                1 // Special case: ÷1
            } else {
                PRESCALER_TABLE[prescal_idx as usize]
            };

            let polarity = if ctrl & ctrl_bit(PWM_ACT_STATE, self.ch) != 0 {
                PwmPolarity::ActiveHigh
            } else {
                PwmPolarity::ActiveLow
            };

            let prd_reg = pwm_read(if self.ch == 0 { PWM_CH0_PRD } else { PWM_CH1_PRD });
            let prd = ((prd_reg >> 16) & PRD_MASK) + 1;
            let dty = prd_reg & DTY_MASK;

            // Convert to nanoseconds
            let clk_hz = PWM_SRC_HZ / prescaler;
            let period_ns = ((prd as u64) * NSEC_PER_SEC / (clk_hz as u64)) as u32;
            let pulse_ns = ((dty as u64) * NSEC_PER_SEC / (clk_hz as u64)) as u32;

            PwmConfig {
                period_ns,
                pulse_ns,
                polarity,
                mode: PwmMode::Continuous, // Mode not tracked separately
            }
        }
    }

    // ── Configure ──────────────────────────────────────────────────────

    /// Apply a new PWM configuration.
    ///
    /// This recalculates prescaler, period, and duty values, writes them
    /// to hardware, and updates polarity.
    ///
    /// The PWM must be disabled before calling this if the prescaler changes;
    /// otherwise the method gates the clock automatically during prescaler
    /// updates.
    pub fn configure(&self, config: &PwmConfig) -> Result<(), PwmError> {
        if config.pulse_ns > config.period_ns {
            return Err(PwmError::PulseExceedsPeriod);
        }

        unsafe {
            let (prescal_idx, prd, dty) =
                self.calculate(config.period_ns, config.pulse_ns)?;

            let mut ctrl = pwm_read(PWM_CTRL_REG);
            let old_prescal = (ctrl >> (self.ch * CH_OFFSET)) & PRESCAL_MASK;

            // If prescaler changed, gate the clock first
            if old_prescal != prescal_idx {
                ctrl &= !ctrl_bit(PWM_CLK_GATING, self.ch);
                pwm_write(PWM_CTRL_REG, ctrl);

                // Update prescaler
                ctrl &= !ctrl_bit(PRESCAL_MASK, self.ch);
                ctrl |= ctrl_bit(prescal_idx, self.ch);
            }

            // Write period/duty
            let prd_offset = if self.ch == 0 { PWM_CH0_PRD } else { PWM_CH1_PRD };
            pwm_write(prd_offset, prd_duty_pack(prd, dty));

            // Set polarity
            match config.polarity {
                PwmPolarity::ActiveHigh => ctrl &= !ctrl_bit(PWM_ACT_STATE, self.ch),
                PwmPolarity::ActiveLow => ctrl |= ctrl_bit(PWM_ACT_STATE, self.ch),
            }

            // Set mode (continuous vs single-pulse)
            match config.mode {
                PwmMode::Continuous => ctrl &= !ctrl_bit(PWM_MODE, self.ch),
                PwmMode::SinglePulse => ctrl |= ctrl_bit(PWM_MODE, self.ch),
            }

            pwm_write(PWM_CTRL_REG, ctrl);
        }

        Ok(())
    }

    // ── Calculatation ──────────────────────────────────────────────────

    /// Calculate prescaler index, period, and duty values for given
    /// period_ns and pulse_ns.
    unsafe fn calculate(
        &self,
        period_ns: u32,
        pulse_ns: u32,
    ) -> Result<(u32, u32, u32), PwmError> {
        // Total cycles at 24MHz for the requested period
        let clk_24m = PWM_SRC_HZ as u64;
        let div = (clk_24m * period_ns as u64) / NSEC_PER_SEC;

        if div == 0 {
            return Err(PwmError::PeriodTooSmall);
        }

        // Search for a prescaler that keeps entire_cycles ≤ 65536
        let mut entire_cycles: u64 = div;
        let mut prescal_idx: u32 = PRESCAL_MASK; // Start with ÷1

        for idx in 0..16u32 {
            let pval = PRESCALER_TABLE[idx as usize];
            if pval == 0 {
                continue;
            }
            let scaled = div / pval as u64;
            if scaled <= 65536 {
                entire_cycles = scaled;
                prescal_idx = idx;
                break;
            }
            // Also try with an extra divider (the C code iterates prescale 0..256)
            // We do a simpler scan: divide by increasing multiples
            let mut extra: u64 = 1;
            while extra <= 256 {
                let scaled = div / (pval as u64 * extra);
                if scaled <= 65536 {
                    entire_cycles = scaled;
                    prescal_idx = idx;
                    break;
                }
                extra += 1;
            }
            if entire_cycles <= 65536 {
                break;
            }
        }

        if entire_cycles > 65536 {
            return Err(PwmError::PeriodTooLarge);
        }
        if entire_cycles == 0 {
            entire_cycles = 1;
        }

        let prd = entire_cycles as u32;

        // Calculate duty cycles proportional to pulse/period
        let active_cycles = (entire_cycles * pulse_ns as u64 / period_ns as u64) as u32;

        Ok((prescal_idx, prd, active_cycles))
    }

    /// Simple interface: set frequency (Hz) and duty cycle (0.0 to 1.0)
    pub fn set_freq_duty(&self, freq_hz: u32, duty: f32) -> Result<(), PwmError> {
        if duty < 0.0 || duty > 1.0 {
            return Err(PwmError::InvalidDuty);
        }
        let period_ns = NSEC_PER_SEC as u32 / freq_hz;
        let pulse_ns = (period_ns as f32 * duty) as u32;
        self.configure(&PwmConfig {
            period_ns,
            pulse_ns,
            polarity: PwmPolarity::ActiveHigh,
            mode: PwmMode::Continuous,
        })
    }

    /// Set period in nanoseconds and duty ratio (0.0 to 1.0)
    pub fn set_period_duty(&self, period_ns: u32, duty: f32) -> Result<(), PwmError> {
        if duty < 0.0 || duty > 1.0 {
            return Err(PwmError::InvalidDuty);
        }
        let pulse_ns = (period_ns as f32 * duty) as u32;
        self.configure(&PwmConfig {
            period_ns,
            pulse_ns,
            polarity: PwmPolarity::ActiveHigh,
            mode: PwmMode::Continuous,
        })
    }
}

// Pwm instances hold raw base addresses and are inherently tied to the
// hardware. They are not meant to be shared across threads.

// ── PWM Errors ────────────────────────────────────────────────────────────

#[derive(Debug)]
pub enum PwmError {
    /// Pulse (active) time exceeds period
    PulseExceedsPeriod,
    /// Requested period is too short to achieve
    PeriodTooSmall,
    /// Requested period is too long to achieve (exceeds 16-bit counter)
    PeriodTooLarge,
    /// Duty cycle is out of range (must be 0.0 to 1.0)
    InvalidDuty,
}

// ── Factory Functions ─────────────────────────────────────────────────────

/// Create and initialize PWM channel 0 on PA2 (func2)
pub fn pwm0_pa2() -> Pwm {
    let pwm = unsafe { Pwm::new(PwmChannel::Ch0) };
    pwm.init(&pins::PWM0_PA2);
    pwm
}

/// Create and initialize PWM channel 0 on PE12 (func3)
pub fn pwm0_pe12() -> Pwm {
    let pwm = unsafe { Pwm::new(PwmChannel::Ch0) };
    pwm.init(&pins::PWM0_PE12);
    pwm
}

/// Create and initialize PWM channel 1 on PE6 (func2)
pub fn pwm1_pe6() -> Pwm {
    let pwm = unsafe { Pwm::new(PwmChannel::Ch1) };
    pwm.init(&pins::PWM1_PE6);
    pwm
}

/// Create and initialize PWM channel 1 on PF5 (func3)
pub fn pwm1_pf5() -> Pwm {
    let pwm = unsafe { Pwm::new(PwmChannel::Ch1) };
    pwm.init(&pins::PWM1_PF5);
    pwm
}
