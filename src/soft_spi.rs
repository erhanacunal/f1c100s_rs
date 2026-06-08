//! Software SPI (bit-banging) driver for Allwinner F1C100s
//!
//! Provides a GPIO-based SPI implementation for cases where hardware SPI
//! controllers are unavailable, or for driving SPI devices with non-standard
//! requirements (e.g. 9-bit data width for LCD panels).
//!
//! Supports:
//! - Standard 3-wire (SCK, MOSI, MISO) or 4-wire (SCK, MOSI, MISO, CS)
//! - 3-wire half-duplex mode (MOSI == MISO shared pin)
//! - Configurable clock speed via delay
//! - SPI modes 0–3
//! - CS control (manual, via a dedicated GPIO pin)

use crate::gpio::{self, Port, PullMode};
use crate::timer;

// ── Soft SPI Configuration ────────────────────────────────────────────────

/// Software SPI bus configuration
#[derive(Debug, Clone)]
pub struct SoftSpiConfig {
    /// SCK pin: (port, pin)
    pub sck: (Port, u8),
    /// MOSI pin: (port, pin)
    pub mosi: (Port, u8),
    /// MISO pin: (port, pin). Set equal to MOSI for 3-wire mode.
    pub miso: (Port, u8),
    /// Half-clock delay in microseconds (determines SCK frequency)
    /// F_SCK ≈ 1 / (2 × delay_us × 1e-6)
    pub delay_us: u32,
    /// SPI mode
    pub mode: SpiMode,
    /// CS pin (optional — set to `None` for external CS control)
    pub cs: Option<(Port, u8)>,
}

/// SPI mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiMode {
    Mode0 = 0,
    Mode1 = 1,
    Mode2 = 2,
    Mode3 = 3,
}

impl SoftSpiConfig {
    /// Create a config for ~100 kHz SCK (delay 5 µs half-cycle), Mode0
    pub const fn new(
        sck: (Port, u8),
        mosi: (Port, u8),
        miso: (Port, u8),
        delay_us: u32,
    ) -> Self {
        Self {
            sck, mosi, miso, delay_us,
            mode: SpiMode::Mode0,
            cs: None,
        }
    }

    /// Set CS pin
    pub fn with_cs(mut self, cs: (Port, u8)) -> Self {
        self.cs = Some(cs);
        self
    }

    /// Set SPI mode
    pub fn with_mode(mut self, mode: SpiMode) -> Self {
        self.mode = mode;
        self
    }

    /// True if MOSI and MISO share the same pin (3-wire mode)
    fn is_3wire(&self) -> bool {
        self.mosi.0 as u8 == self.miso.0 as u8 && self.mosi.1 == self.miso.1
    }
}

// ── Soft SPI Instance ─────────────────────────────────────────────────────

/// Software SPI bus instance
pub struct SoftSpi {
    cfg: SoftSpiConfig,
}

impl SoftSpi {
    /// Create a new software SPI bus with the given configuration.
    /// Call `init()` to configure GPIO pins.
    pub fn new(cfg: SoftSpiConfig) -> Self {
        Self { cfg }
    }

    /// Initialize GPIO pins and set default levels.
    pub fn init(&self) {
        let (sck_port, sck_pin) = self.cfg.sck;
        let (mosi_port, mosi_pin) = self.cfg.mosi;
        let (miso_port, miso_pin) = self.cfg.miso;

        // SCK output, initial level depends on mode
        let sck_idle = match self.cfg.mode {
            SpiMode::Mode0 | SpiMode::Mode1 => false,  // CPOL=0
            SpiMode::Mode2 | SpiMode::Mode3 => true,   // CPOL=1
        };

        gpio::set_function(sck_port, sck_pin, gpio::function::OUTPUT);
        gpio::set_value(sck_port, sck_pin, sck_idle);
        gpio::set_pull_mode(sck_port, sck_pin, PullMode::Disable);

        // MOSI output, high idle
        gpio::set_function(mosi_port, mosi_pin, gpio::function::OUTPUT);
        gpio::set_value(mosi_port, mosi_pin, true);
        gpio::set_pull_mode(mosi_port, mosi_pin, PullMode::Disable);

        if !self.cfg.is_3wire() {
            // MISO input with pull-up
            gpio::set_function(miso_port, miso_pin, gpio::function::INPUT);
            gpio::set_pull_mode(miso_port, miso_pin, PullMode::Up);
        }

        // CS output, high (inactive)
        if let Some((cs_port, cs_pin)) = self.cfg.cs {
            gpio::set_function(cs_port, cs_pin, gpio::function::OUTPUT);
            gpio::set_value(cs_port, cs_pin, true);
            gpio::set_pull_mode(cs_port, cs_pin, PullMode::Disable);
        }
    }

    // ── GPIO helpers ───────────────────────────────────────────────────

    #[inline]
    fn sck_write(&self, val: bool) {
        gpio::set_value(self.cfg.sck.0, self.cfg.sck.1, val);
    }

    #[inline]
    fn mosi_write(&self, val: bool) {
        gpio::set_value(self.cfg.mosi.0, self.cfg.mosi.1, val);
    }

    #[inline]
    fn miso_read(&self) -> bool {
        gpio::get_value(self.cfg.miso.0, self.cfg.miso.1)
    }

    #[inline]
    fn mosi_read(&self) -> bool {
        gpio::get_value(self.cfg.mosi.0, self.cfg.mosi.1)
    }

    #[inline]
    fn cs_write(&self, val: bool) {
        if let Some((port, pin)) = self.cfg.cs {
            gpio::set_value(port, pin, val);
        }
    }

    fn delay_half(&self) {
        timer::delay_us(self.cfg.delay_us);
    }

    /// Switch MOSI pin to input (for 3-wire reads)
    fn mosi_input_mode(&self) {
        gpio::set_function(self.cfg.mosi.0, self.cfg.mosi.1, gpio::function::INPUT);
    }

    /// Switch MOSI pin back to output
    fn mosi_output_mode(&self) {
        gpio::set_function(self.cfg.mosi.0, self.cfg.mosi.1, gpio::function::OUTPUT);
    }

    // ── Transfer ───────────────────────────────────────────────────────

    /// Transfer a single byte, returns the received byte.
    fn transfer_byte(&self, byte: u8) -> u8 {
        let cpha = matches!(self.cfg.mode, SpiMode::Mode1 | SpiMode::Mode3);
        let cpol = matches!(self.cfg.mode, SpiMode::Mode2 | SpiMode::Mode3);

        let mut rx: u8 = 0;

        for i in 0..8 {
            let bit = 7 - i;

            // Set MOSI before clock edge for CPHA=0
            if !cpha {
                self.mosi_write((byte >> bit) & 1 != 0);
                self.delay_half();
            }

            // Toggle SCK (first edge)
            self.sck_write(!cpol);
            self.delay_half();

            // Sample MISO
            if self.miso_read() {
                rx |= 1 << bit;
            }

            // Set MOSI after clock edge for CPHA=1
            if cpha {
                self.mosi_write((byte >> bit) & 1 != 0);
            }

            // Toggle SCK back
            self.sck_write(cpol);
            self.delay_half();
        }

        rx
    }

    /// Transfer a single byte in 3-wire mode (MOSI used for both TX and RX)
    fn transfer_byte_3wire(&self, byte: u8) -> u8 {
        let cpha = matches!(self.cfg.mode, SpiMode::Mode1 | SpiMode::Mode3);
        let cpol = matches!(self.cfg.mode, SpiMode::Mode2 | SpiMode::Mode3);
        let mut rx: u8 = 0;

        // Ensure MOSI is output to start
        self.mosi_output_mode();

        for i in 0..8 {
            let bit = 7 - i;

            if !cpha {
                self.mosi_write((byte >> bit) & 1 != 0);
                self.delay_half();
            }

            self.sck_write(!cpol);

            if cpha {
                self.delay_half();
                // Switch MOSI to input to read
                self.mosi_input_mode();
                self.delay_half();
                // Sample
                if self.mosi_read() {
                    rx |= 1 << bit;
                }
                // Switch back to output
                self.mosi_output_mode();
            } else {
                self.delay_half();
                // Switch to input briefly to read
                self.mosi_input_mode();
                if self.mosi_read() {
                    rx |= 1 << bit;
                }
                self.mosi_output_mode();
            }

            self.sck_write(cpol);

            if cpha {
                self.mosi_write((byte >> bit) & 1 != 0);
            }
            self.delay_half();
        }

        rx
    }

    // ── Public API ─────────────────────────────────────────────────────

    /// Transfer data: sends `tx` and simultaneously receives into `rx`.
    /// If `tx` is shorter than `rx`, remaining TX bytes are 0x00.
    /// If `rx` is shorter than `tx`, extra received bytes are discarded.
    pub fn transfer(&self, tx: &[u8], rx: &mut [u8]) {
        let len = tx.len().max(rx.len());
        let is_3w = self.cfg.is_3wire();

        for i in 0..len {
            let tx_byte = if i < tx.len() { tx[i] } else { 0x00 };
            let rx_byte = if is_3w {
                self.transfer_byte_3wire(tx_byte)
            } else {
                self.transfer_byte(tx_byte)
            };
            if i < rx.len() {
                rx[i] = rx_byte;
            }
        }
    }

    /// Send data only (ignores MISO)
    pub fn send(&self, tx: &[u8]) {
        for &byte in tx {
            if self.cfg.is_3wire() {
                self.transfer_byte_3wire(byte);
            } else {
                self.transfer_byte(byte);
            }
        }
    }

    /// Receive data (sends 0x00 on MOSI)
    pub fn receive(&self, rx: &mut [u8]) {
        for byte in rx.iter_mut() {
            *byte = if self.cfg.is_3wire() {
                self.transfer_byte_3wire(0x00)
            } else {
                self.transfer_byte(0x00)
            };
        }
    }

    /// Send 9-bit data (command: bit8=0, data: bit8=1).
    /// This is specifically for LCD panels like TL021WVC04 that use
    /// 9-bit SPI where the 9th bit distinguishes command from data.
    pub fn send_9bit(&self, value: u16, delay_after: u32) {
        // bit8=1 → data (D/C HIGH), bit8=0 → command (D/C LOW)
        let dc_bit = (value & 0x100) != 0;
        let cpha = matches!(self.cfg.mode, SpiMode::Mode1 | SpiMode::Mode3);
        let cpol = matches!(self.cfg.mode, SpiMode::Mode2 | SpiMode::Mode3);

        self.cs_write(false);

        // Send the 9th bit (D/C) first
        if !cpha {
            self.mosi_write(dc_bit); // low=command, high=data
            self.delay_half();
        }

        self.sck_write(!cpol);
        self.delay_half();

        if cpha {
            self.mosi_write(dc_bit);
        }

        self.sck_write(cpol);
        self.delay_half();

        // Then send 8 data bits using the standard byte transfer
        if self.cfg.is_3wire() {
            self.transfer_byte_3wire(value as u8);
        } else {
            self.transfer_byte(value as u8);
        }

        self.cs_write(true);

        if delay_after > 0 {
            timer::delay_us(delay_after);
        }
    }

    // ── CS control ─────────────────────────────────────────────────────

    /// Assert chip select (active low)
    pub fn cs_assert(&self) {
        self.cs_write(false);
    }

    /// De-assert chip select
    pub fn cs_deassert(&self) {
        self.cs_write(true);
    }

    /// Assert CS, run a closure, then de-assert CS
    pub fn with_cs<F: FnOnce()>(&self, f: F) {
        self.cs_assert();
        f();
        self.cs_deassert();
    }
}

