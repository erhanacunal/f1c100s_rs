//! Hardware SPI driver for Allwinner F1C100s
//!
//! The F1C100s provides two SPI controllers:
//! - SPI0 at `0x01C05000` (pins: PC0=CS, PC1=CLK, PC2=MISO, PC3=MOSI, func1)
//! - SPI1 at `0x01C06000` (pins: PA0=CS, PA1=CLK, PA2=MISO, PA3=MOSI, func5)
//!
//! Each controller features:
//! - 64-byte TX/RX FIFOs with configurable trigger levels
//! - Master/slave mode (we support master only)
//! - 4 hardware chip-select lines (SS0–SS3)
//! - Clock divider from AHB bus (typically 200 MHz) down to kHz range
//! - SPI modes 0/1/2/3, MSB/LSB first, full/half duplex
//! - Max bus clock: 30 MHz
//!
//! ## Clock Calculation
//! SPI_CLK = AHB_CLK / (2 × (DIV + 1))  for CDR2 mode (DIV ≤ 511)
//! SPI_CLK = AHB_CLK / (2 × 2^(DIV+1))  for CDR1 mode (DIV > 511)

use crate::clock;
use crate::gpio::{self, Port, PullMode, DriveLevel};

// ── Base Addresses ────────────────────────────────────────────────────────

pub const SPI0_BASE: u32 = 0x01C05000;
pub const SPI1_BASE: u32 = 0x01C06000;

// ── Register Offsets ──────────────────────────────────────────────────────

#[allow(dead_code)]
mod reg {
    pub const CTRL: u32 = 0x04;   // Global Control
    pub const TCTRL: u32 = 0x08;  // Transfer Control
    pub const IER: u32 = 0x10;    // Interrupt Control
    pub const STA: u32 = 0x14;    // Interrupt Status
    pub const FCTL: u32 = 0x18;   // FIFO Control
    pub const FST: u32 = 0x1C;    // FIFO Status
    pub const WAIT: u32 = 0x20;   // Wait Clock Counter
    pub const CCTR: u32 = 0x24;   // Clock Rate Control
    pub const BC: u32 = 0x30;     // Burst Control
    pub const TC: u32 = 0x34;     // Transmit Counter
    pub const BCC: u32 = 0x38;    // Burst Control (alt)
    pub const TXD: u32 = 0x200;   // TX Data (8-bit access)
    pub const RXD: u32 = 0x300;   // RX Data (8-bit access)
}

// ── Control Register Bits ─────────────────────────────────────────────────

#[allow(dead_code)]
mod ctrl {
    pub const RST: u32 = 1 << 31;
    pub const TP_EN: u32 = 1 << 7;
    pub const MODE_MASTER: u32 = 1 << 1;
    pub const EN: u32 = 1 << 0;
}

#[allow(dead_code)]
mod tctrl {
    pub const XCH: u32 = 1 << 31;       // Start exchange
    pub const RPSM: u32 = 1 << 10;      // Rapid mode
    pub const SDC: u32 = 1 << 11;       // Sample delay control
    pub const FBS_LSB: u32 = 1 << 12;   // First bit LSB
    pub const DHB_HALF: u32 = 1 << 8;   // Half-duplex
    pub const SS_LEVEL: u32 = 1 << 7;   // CS level (1=high)
    pub const SS_OWNER_SW: u32 = 1 << 6; // Software CS control
    pub const SS_SEL_SHIFT: u32 = 4;
    pub const SS_CTL: u32 = 1 << 3;     // Auto CS de-assert control
    pub const SPOL: u32 = 1 << 2;       // CS idle polarity
    pub const CPOL: u32 = 1 << 1;       // Clock polarity (1=low when idle)
    pub const CPHA: u32 = 1 << 0;       // Clock phase (1=sample on trailing edge)
}

#[allow(dead_code)]
mod fctl {
    pub const TF_RST: u32 = 1 << 31;    // TX FIFO reset
    pub const RF_RST: u32 = 1 << 15;    // RX FIFO reset
    pub const TX_TRIG_SHIFT: u32 = 16;
    pub const RX_TRIG_SHIFT: u32 = 0;
}

mod fst {
    pub const TF_CNT_SHIFT: u32 = 16;
    pub const TF_CNT_MASK: u32 = 0xFF;
    pub const RF_CNT_SHIFT: u32 = 0;
    pub const RF_CNT_MASK: u32 = 0xFF;
}

#[allow(dead_code)]
mod cctr {
    pub const DRS: u32 = 1 << 12;       // Divider rate select (0=CDR1, 1=CDR2)
    pub const CDR1_SHIFT: u32 = 8;
    pub const CDR2_SHIFT: u32 = 0;
}

#[allow(dead_code)]
mod sta {
    pub const TC: u32 = 1 << 12;        // Transfer complete
    pub const TX_READY: u32 = 1 << 4;
    pub const RX_READY: u32 = 1 << 0;
}

// ── Constants ─────────────────────────────────────────────────────────────

const FIFO_SIZE: u32 = 64;
const BUS_MAX_CLK: u32 = 30_000_000;
/// Default AHB clock (200 MHz after standard init)
const DEFAULT_AHB_HZ: u32 = 200_000_000;

// ── SPI Mode ──────────────────────────────────────────────────────────────

/// SPI mode (CPOL, CPHA combination)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiMode {
    /// CPOL=0, CPHA=0: clock low idle, sample on leading edge
    Mode0 = 0,
    /// CPOL=0, CPHA=1: clock low idle, sample on trailing edge
    Mode1 = 1,
    /// CPOL=1, CPHA=0: clock high idle, sample on leading edge
    Mode2 = 2,
    /// CPOL=1, CPHA=1: clock high idle, sample on trailing edge
    Mode3 = 3,
}

/// Bit order
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BitOrder {
    /// Most significant bit first
    MsbFirst,
    /// Least significant bit first
    LsbFirst,
}

/// SPI chip select (SS0–SS3)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChipSelect {
    Ss0 = 0,
    Ss1 = 1,
    Ss2 = 2,
    Ss3 = 3,
}

// ── MMIO Helpers ──────────────────────────────────────────────────────────

#[inline]
unsafe fn spi_read(base: u32, offset: u32) -> u32 {
    ((base + offset) as *const u32).read_volatile()
}

#[inline]
unsafe fn spi_write(base: u32, offset: u32, val: u32) {
    ((base + offset) as *mut u32).write_volatile(val);
}

/// 8-bit write to TXD (byte access per HAL_REG_8BIT)
#[inline]
unsafe fn spi_write_byte(base: u32, byte: u8) {
    ((base + reg::TXD) as *mut u8).write_volatile(byte);
}

/// 8-bit read from RXD
#[inline]
unsafe fn spi_read_byte(base: u32) -> u8 {
    ((base + reg::RXD) as *const u8).read_volatile()
}

// ── SPI Instance ──────────────────────────────────────────────────────────

/// Pin-mux configuration for one SPI bus
#[derive(Debug, Clone)]
pub struct SpiPins {
    pub clk_port: Port,
    pub clk_pin: u8,
    pub mosi_port: Port,
    pub mosi_pin: u8,
    pub miso_port: Port,
    pub miso_pin: u8,
    pub cs_port: Port,
    pub cs_pin: u8,
    pub pin_func: u8,
}

/// A configured hardware SPI bus
pub struct Spi {
    base: u32,
    gate: clock::BusGate,
    ahb_hz: u32,
}

impl Spi {
    /// Create a new SPI instance.
    ///
    /// # Safety
    /// Caller must ensure the base address is valid for this CPU.
    pub unsafe fn new(base: u32, gate: clock::BusGate, ahb_hz: u32) -> Self {
        Self { base, gate, ahb_hz }
    }

    /// Initialize the SPI bus: configure pins, enable clock gate, release reset.
    pub fn init(&self, pins: &SpiPins) {
        // Configure pins
        gpio::set_function(pins.clk_port, pins.clk_pin, pins.pin_func);
        gpio::set_pull_mode(pins.clk_port, pins.clk_pin, PullMode::Disable);
        gpio::set_drive_level(pins.clk_port, pins.clk_pin, DriveLevel::Level0);

        gpio::set_function(pins.mosi_port, pins.mosi_pin, pins.pin_func);
        gpio::set_pull_mode(pins.mosi_port, pins.mosi_pin, PullMode::Disable);
        gpio::set_drive_level(pins.mosi_port, pins.mosi_pin, DriveLevel::Level0);

        gpio::set_function(pins.miso_port, pins.miso_pin, pins.pin_func);
        gpio::set_pull_mode(pins.miso_port, pins.miso_pin, PullMode::Disable);
        gpio::set_drive_level(pins.miso_port, pins.miso_pin, DriveLevel::Level0);

        gpio::set_function(pins.cs_port, pins.cs_pin, pins.pin_func);
        gpio::set_pull_mode(pins.cs_port, pins.cs_pin, PullMode::Disable);
        gpio::set_drive_level(pins.cs_port, pins.cs_pin, DriveLevel::Level0);

        // Enable clock gate and release reset
        clock::bus_clk_init(self.gate);
    }

    // ── Configuration ──────────────────────────────────────────────────

    /// Fully configure the SPI controller and enable it.
    ///
    /// Resets FIFOs, sets mode, bit order, chip select, and clock speed.
    pub fn configure(
        &self,
        mode: SpiMode,
        order: BitOrder,
        cs: ChipSelect,
        max_hz: u32,
    ) {
        unsafe {
            // Disable, then reset
            self.disable();
            self.reset_ctrl();
            self.reset_fifos();

            // Master mode, full duplex
            let mut tctrl_val = 0u32;

            // Set SPI mode (CPOL, CPHA)
            match mode {
                SpiMode::Mode0 => {} // both 0
                SpiMode::Mode1 => { tctrl_val |= tctrl::CPHA; }
                SpiMode::Mode2 => { tctrl_val |= tctrl::CPOL; }
                SpiMode::Mode3 => { tctrl_val |= tctrl::CPOL | tctrl::CPHA; }
            }

            // Bit order
            if order == BitOrder::LsbFirst {
                tctrl_val |= tctrl::FBS_LSB;
            }

            // Software CS control, select CS line
            tctrl_val |= tctrl::SS_OWNER_SW;
            tctrl_val |= (cs as u32) << tctrl::SS_SEL_SHIFT;

            spi_write(self.base, reg::TCTRL, tctrl_val);

            // Set clock divider (source: AHB)
            self.set_clock(max_hz);

            // Set master mode and enable
            spi_write(self.base, reg::CTRL, ctrl::MODE_MASTER | ctrl::EN);
        }
    }

    /// Set the SPI clock divider to achieve ≤ `max_hz`.
    unsafe fn set_clock(&self, max_hz: u32) {
        let src = self.ahb_hz;
        let limit = max_hz.min(BUS_MAX_CLK);
        let mut div = (src + limit - 1) / limit; // ceil division

        if div < 1 {
            div = 1;
        }

        // SPI_CLK = AHB / (2 * (DIV+1)) when DRS=1 (CDR2 mode, div ≤ 2×256=512)
        // SPI_CLK = AHB / (2 * 2^(DIV+1)) when DRS=0 (CDR1 mode, div > 512)
        if div > 2 * 256 {
            // Use CDR1 mode: find n such that 2 * 2^(n+1) ≈ div
            // Desired: div_effective = (src + limit - 1) / limit
            // In CDR1: SPI_CLK = AHB / (2 * 2^(n+1)) → div_eff = 2 * 2^(n+1)
            // We need 2 * 2^(n+1) ≥ desired_div
            let mut n: u32 = 0;
            let mut div_eff: u32 = 2;
            while div_eff < div && n < 15 {
                div_eff *= 2;
                n += 1;
            }
            // Clear DRS, set CDR1
            let cctr_val = (n & 0x0F) << cctr::CDR1_SHIFT;
            spi_write(self.base, reg::CCTR, cctr_val);
        } else {
            // Use CDR2 mode (DRS=1)
            let n = ((div + 1) / 2).saturating_sub(1);
            let cctr_val = cctr::DRS | ((n & 0xFF) << cctr::CDR2_SHIFT);
            spi_write(self.base, reg::CCTR, cctr_val);
        }
    }

    // ── Low-level Control ──────────────────────────────────────────────

    unsafe fn disable(&self) {
        let val = spi_read(self.base, reg::CTRL) & !ctrl::EN;
        spi_write(self.base, reg::CTRL, val);
    }

    unsafe fn reset_ctrl(&self) {
        spi_write(self.base, reg::CTRL, ctrl::RST);
    }

    unsafe fn reset_fifos(&self) {
        // Reset TX FIFO
        spi_write(self.base, reg::FCTL, fctl::TF_RST);
        while spi_read(self.base, reg::FCTL) & fctl::TF_RST != 0 {
            core::hint::spin_loop();
        }
        // Reset RX FIFO
        spi_write(self.base, reg::FCTL, fctl::RF_RST);
        while spi_read(self.base, reg::FCTL) & fctl::RF_RST != 0 {
            core::hint::spin_loop();
        }
    }

    unsafe fn tx_fifo_count(&self) -> u8 {
        ((spi_read(self.base, reg::FST) >> fst::TF_CNT_SHIFT) & fst::TF_CNT_MASK) as u8
    }

    unsafe fn rx_fifo_count(&self) -> u8 {
        ((spi_read(self.base, reg::FST) >> fst::RF_CNT_SHIFT) & fst::RF_CNT_MASK) as u8
    }

    /// Assert chip select (active low by default)
    unsafe fn cs_assert(&self) {
        let val = spi_read(self.base, reg::TCTRL) & !tctrl::SS_LEVEL;
        spi_write(self.base, reg::TCTRL, val);
    }

    /// De-assert chip select
    unsafe fn cs_deassert(&self) {
        let val = spi_read(self.base, reg::TCTRL) | tctrl::SS_LEVEL;
        spi_write(self.base, reg::TCTRL, val);
    }

    // ── Transfer ───────────────────────────────────────────────────────

    /// Perform a full-duplex SPI transfer.
    ///
    /// Sends `tx` and simultaneously receives into `rx`.
    /// If `tx` is shorter than `rx`, remaining bytes are sent as 0xFF.
    /// If `rx` is shorter than `tx`, extra received bytes are discarded.
    pub fn transfer(&self, tx: &[u8], rx: &mut [u8]) -> Result<(), SpiError> {
        let len = tx.len().max(rx.len());
        if len == 0 {
            return Ok(());
        }

        unsafe {
            self.reset_fifos();

            // Set transfer size (only data, no dummy bytes)
            spi_write(self.base, reg::BC, len as u32);
            spi_write(self.base, reg::TC, len as u32);
            spi_write(self.base, reg::BCC, len as u32);

            // Start exchange
            spi_write(self.base, reg::TCTRL,
                spi_read(self.base, reg::TCTRL) | tctrl::XCH);

            let mut tx_idx = 0usize;
            let mut rx_idx = 0usize;
            let mut tx_remain = len;
            let mut rx_remain = len;

            while tx_remain > 0 || rx_remain > 0 {
                // Fill TX FIFO
                while (self.tx_fifo_count() as u32) < FIFO_SIZE && tx_remain > 0 {
                    let byte = if tx_idx < tx.len() {
                        tx[tx_idx]
                    } else {
                        0xFFu8
                    };
                    spi_write_byte(self.base, byte);
                    tx_idx += 1;
                    tx_remain -= 1;
                }

                // Drain RX FIFO
                while self.rx_fifo_count() > 0 && rx_remain > 0 {
                    let byte = spi_read_byte(self.base);
                    if rx_idx < rx.len() {
                        rx[rx_idx] = byte;
                    }
                    rx_idx += 1;
                    rx_remain -= 1;
                }
            }

            // Wait for transfer complete
            let mut timeout: u32 = 0xFFFFF;
            while spi_read(self.base, reg::STA) & sta::TC == 0 && timeout > 0 {
                timeout -= 1;
                core::hint::spin_loop();
            }
            // Clear TC flag
            spi_write(self.base, reg::STA, sta::TC);

            if timeout == 0 {
                return Err(SpiError::Timeout);
            }
        }

        Ok(())
    }

    /// Send-only SPI transfer (ignores MISO)
    pub fn send(&self, tx: &[u8]) -> Result<(), SpiError> {
        // For pure TX, we need to read the same number of bytes from RX FIFO
        // Use a small stack buffer for batches
        let mut rx_buf = [0u8; 64];
        let mut off = 0;
        while off < tx.len() {
            let chunk = (tx.len() - off).min(64);
            self.transfer(&tx[off..off + chunk], &mut rx_buf[..chunk])?;
            off += chunk;
        }
        Ok(())
    }

    /// Receive-only SPI transfer (sends 0xFF as dummy TX)
    pub fn receive(&self, rx: &mut [u8]) -> Result<(), SpiError> {
        let tx = [0xFFu8; 1];
        let mut off = 0;
        while off < rx.len() {
            let chunk = (rx.len() - off).min(64);
            // Always send the same dummy byte, transfer handles TX expansion
            self.transfer(&tx, &mut rx[off..off + chunk])?;
            off += chunk;
        }
        Ok(())
    }

    // ── Convenience with CS control ────────────────────────────────────

    /// Assert CS, perform transfer, de-assert CS
    pub fn xfer(&self, tx: &[u8], rx: &mut [u8]) -> Result<(), SpiError> {
        unsafe { self.cs_assert(); }
        let result = self.transfer(tx, rx);
        unsafe { self.cs_deassert(); }
        result
    }

    /// Assert CS, send data, de-assert CS
    pub fn xfer_send(&self, tx: &[u8]) -> Result<(), SpiError> {
        unsafe { self.cs_assert(); }
        let result = self.send(tx);
        unsafe { self.cs_deassert(); }
        result
    }

    /// Assert CS, receive data, de-assert CS (no explicit CS in the method name,
    /// but it's the caller's responsibility to understand)
    pub fn xfer_receive(&self, rx: &mut [u8]) -> Result<(), SpiError> {
        unsafe { self.cs_assert(); }
        let result = self.receive(rx);
        unsafe { self.cs_deassert(); }
        result
    }
}

/// SPI error types
#[derive(Debug)]
pub enum SpiError {
    Timeout,
}

// ── Pre-defined Pin Configurations ────────────────────────────────────────

/// SPI0 pins: PC0=CS, PC1=CLK, PC2=MISO, PC3=MOSI, func1
pub const SPI0_PINS: SpiPins = SpiPins {
    clk_port: Port::C, clk_pin: 1,
    mosi_port: Port::C, mosi_pin: 3,
    miso_port: Port::C, miso_pin: 2,
    cs_port: Port::C, cs_pin: 0,
    pin_func: gpio::function::FUNC2,
};

/// SPI1 pins: PA0=CS, PA1=CLK, PA2=MISO, PA3=MOSI, func5
pub const SPI1_PINS: SpiPins = SpiPins {
    clk_port: Port::A, clk_pin: 1,
    mosi_port: Port::A, mosi_pin: 3,
    miso_port: Port::A, miso_pin: 2,
    cs_port: Port::A, cs_pin: 0,
    pin_func: gpio::function::FUNC5,
};

// ── Factory Functions ─────────────────────────────────────────────────────

/// Create and initialize SPI0 with default pins (PC0–PC3, func1)
pub fn spi0() -> Spi {
    let spi = unsafe { Spi::new(SPI0_BASE, clock::BusGate::Spi0, DEFAULT_AHB_HZ) };
    spi.init(&SPI0_PINS);
    spi
}

/// Create and initialize SPI1 with default pins (PA0–PA3, func5)
pub fn spi1() -> Spi {
    let spi = unsafe { Spi::new(SPI1_BASE, clock::BusGate::Spi1, DEFAULT_AHB_HZ) };
    spi.init(&SPI1_PINS);
    spi
}
