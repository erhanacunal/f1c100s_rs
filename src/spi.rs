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
//! - Max bus clock: 50 MHz (AHB/4; what the xboot SPL reads SPI NOR at)
//!
//! ## Clock Calculation
//! SPI_CLK = AHB_CLK / (2 × (DIV + 1))  for CDR2 mode (DIV ≤ 511)
//! SPI_CLK = AHB_CLK / (2 × 2^(DIV+1))  for CDR1 mode (DIV > 511)
//!
//! ## DMA
//! After [`Spi::set_dma`] (and [`dma::init`]), [`Spi::send`] and
//! [`Spi::receive`] move transfers of at least [`DMA_MIN_LEN`] bytes with
//! two normal DMA channels instead of the CPU. That includes everything
//! built on them: `xfer_send`, `xfer_receive`, `xfer_cmd_read`,
//! `xfer_cmd_send`, and so `spi_flash` reads and page programs.
//! Full-duplex [`Spi::transfer`] stays PIO.
//!
//! RX buffers need no special alignment. Only their 32-byte (cache line)
//! aligned interior goes through DMA; the unaligned head and tail bytes
//! are read with PIO inside the same CS-asserted transaction. Neighbouring
//! variables that share the first or last cache line are therefore never
//! clobbered.
//!
//! FIFO DRQ enables (TX bit 24, RX bit 8) and the discard-RX bit (TCR bit
//! 8) follow Linux `drivers/spi/spi-sun6i.c`.

use crate::clock;
use crate::dma;
use crate::gpio::{self, Port, PullMode, DriveLevel};
use crate::mmu;

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
    pub const DHB: u32 = 1 << 8;        // Discard RX ("discard hash burst"): TX-only transfers
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
    pub const TF_DRQ_EN: u32 = 1 << 24; // TX FIFO DMA request enable
    pub const RF_RST: u32 = 1 << 15;    // RX FIFO reset
    pub const RF_DRQ_EN: u32 = 1 << 8;  // RX FIFO DMA request enable
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
/// AHB/4 = CDR2 divider 1 (CCR 0x1001), the clock the xboot SPL
/// (`sys-spinor.c`) boots from SPI NOR with, no sample delay needed.
const BUS_MAX_CLK: u32 = 50_000_000;
/// Default AHB clock (200 MHz after standard init)
const DEFAULT_AHB_HZ: u32 = 200_000_000;

/// Shortest `send` / `receive` that goes through DMA once [`Spi::set_dma`]
/// is set. Shorter transfers fit in a few FIFO loads and are faster as PIO.
pub const DMA_MIN_LEN: usize = 256;
/// Bytes per DMA-driven SPI burst: one NDMA hardware transfer
/// (`dma::NDMA_MAX_BYTES`), so the SPI never waits on a chunk switch.
const DMA_BURST_MAX: usize = dma::NDMA_MAX_BYTES as usize;
/// TX FIFO DMA request level: refill when at most half full.
const DMA_TX_TRIG: u32 = FIFO_SIZE / 2;
/// RX FIFO DMA request level: one byte, so the tail of any length drains.
const DMA_RX_TRIG: u32 = 1;
/// Poll iterations before a DMA burst is declared stuck (≈ seconds).
const DMA_TIMEOUT: u32 = 0x0400_0000;
/// D-cache line size; RX DMA only targets whole lines.
const LINE: usize = mmu::CACHE_LINE_SIZE as usize;

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

/// The two normal DMA channels an SPI bus uses for TX and RX
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpiDma {
    pub tx: dma::Channel,
    pub rx: dma::Channel,
}

/// A configured hardware SPI bus
pub struct Spi {
    base: u32,
    gate: clock::BusGate,
    ahb_hz: u32,
    dma: Option<SpiDma>,
}

impl Spi {
    /// Create a new SPI instance.
    ///
    /// # Safety
    /// Caller must ensure the base address is valid for this CPU.
    pub unsafe fn new(base: u32, gate: clock::BusGate, ahb_hz: u32) -> Self {
        Self { base, gate, ahb_hz, dma: None }
    }

    /// Use DMA for `send` / `receive` of at least [`DMA_MIN_LEN`] bytes.
    ///
    /// `tx` and `rx` must be two different normal channels (N0–N3), because
    /// the SPI request lines only exist on NDMA. [`dma::init`] must have run.
    /// The channels belong to this bus from now on; do not start other
    /// transfers on them.
    pub fn set_dma(&mut self, tx: dma::Channel, rx: dma::Channel) -> Result<(), SpiError> {
        if tx.is_dedicated() || rx.is_dedicated() || tx == rx {
            return Err(SpiError::Dma(dma::Error::PortMismatch));
        }
        self.dma = Some(SpiDma { tx, rx });
        Ok(())
    }

    /// Go back to PIO for every transfer.
    pub fn clear_dma(&mut self) {
        self.dma = None;
    }

    /// DMA channels in use, if any
    pub fn dma(&self) -> Option<SpiDma> {
        self.dma
    }

    fn drq(&self) -> dma::NdmaDrq {
        if self.base == SPI1_BASE { dma::NdmaDrq::Spi1 } else { dma::NdmaDrq::Spi0 }
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
            // Mirror the proven F1C100s bring-up (xboot sys-spinor.c / the SPL
            // sys_copyself path that boots this board): clock → reset+enable
            // (one write, wait for self-clear) → transfer control → FIFO reset.

            // 1. Clock divider (source: AHB).
            self.set_clock(max_hz);

            // 2. Soft-reset + enable in a single GCR write, then wait for the
            //    reset bit to self-clear. TP_EN pauses TX when the RX FIFO is
            //    full so long read bursts never drop bytes.
            spi_write(self.base, reg::CTRL,
                ctrl::RST | ctrl::TP_EN | ctrl::MODE_MASTER | ctrl::EN);
            let mut timeout: u32 = 0xFFFFF;
            while spi_read(self.base, reg::CTRL) & ctrl::RST != 0 && timeout > 0 {
                timeout -= 1;
                core::hint::spin_loop();
            }

            // 3. Transfer control: SPI mode, bit order, software CS, SPOL
            //    (CS active low / idle high — REQUIRED for SPI NOR; without it
            //    the chip is never selected and every read returns garbage),
            //    CS line, and leave CS de-asserted (SS_LEVEL=1) at idle.
            let mut tctrl_val = tctrl::SS_OWNER_SW | tctrl::SPOL | tctrl::SS_LEVEL;
            match mode {
                SpiMode::Mode0 => {} // both 0
                SpiMode::Mode1 => { tctrl_val |= tctrl::CPHA; }
                SpiMode::Mode2 => { tctrl_val |= tctrl::CPOL; }
                SpiMode::Mode3 => { tctrl_val |= tctrl::CPOL | tctrl::CPHA; }
            }
            if order == BitOrder::LsbFirst {
                tctrl_val |= tctrl::FBS_LSB;
            }
            tctrl_val |= (cs as u32) << tctrl::SS_SEL_SHIFT;
            spi_write(self.base, reg::TCTRL, tctrl_val);

            // 4. Reset FIFOs.
            self.reset_fifos();
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

        // Burst the exchange in ≤64-byte (FIFO-sized) chunks, mirroring the
        // proven F1C100s sequence (xboot sys-spinor.c sys_spi_transfer):
        //   set MBC/MTC/BCC = n  →  preload the whole TX FIFO  →  start XCH  →
        //   wait XCH self-clear  →  wait RX FIFO count == n  →  drain.
        // Loading the TX FIFO *before* starting the exchange is essential;
        // starting XCH with an empty FIFO clocks garbage.
        let mut off = 0usize;
        while off < len {
            let n = (len - off).min(FIFO_SIZE as usize);
            unsafe {
                spi_write(self.base, reg::BC, n as u32);   // MBC: total burst
                spi_write(self.base, reg::TC, n as u32);   // MTC: bytes transmitted
                spi_write(self.base, reg::BCC, n as u32);  // BCC: single-mode TX count

                // Preload TX FIFO (real data, padded with 0xFF for RX-only).
                for i in 0..n {
                    let byte = if off + i < tx.len() { tx[off + i] } else { 0xFFu8 };
                    spi_write_byte(self.base, byte);
                }

                // Start exchange, wait for it to self-clear.
                spi_write(self.base, reg::TCTRL,
                    spi_read(self.base, reg::TCTRL) | tctrl::XCH);
                let mut timeout: u32 = 0xFFFFF;
                while spi_read(self.base, reg::TCTRL) & tctrl::XCH != 0 {
                    if timeout == 0 {
                        return Err(SpiError::Timeout);
                    }
                    timeout -= 1;
                    core::hint::spin_loop();
                }

                // Wait until all n bytes are in the RX FIFO, then drain them.
                let mut timeout: u32 = 0xFFFFF;
                while self.rx_fifo_count() < n as u8 {
                    if timeout == 0 {
                        return Err(SpiError::Timeout);
                    }
                    timeout -= 1;
                    core::hint::spin_loop();
                }
                for i in 0..n {
                    let byte = spi_read_byte(self.base);
                    if off + i < rx.len() {
                        rx[off + i] = byte;
                    }
                }
            }
            off += n;
        }

        Ok(())
    }

    /// Send-only SPI transfer (ignores MISO).
    ///
    /// Uses DMA for `tx.len() >= DMA_MIN_LEN` when [`Spi::set_dma`] is set.
    pub fn send(&self, tx: &[u8]) -> Result<(), SpiError> {
        if let Some(d) = self.dma {
            if tx.len() >= DMA_MIN_LEN {
                for chunk in tx.chunks(DMA_BURST_MAX) {
                    // SAFETY: `chunk` is borrowed for the whole blocking burst.
                    unsafe { self.dma_burst(d, Some(chunk.as_ptr() as u32), None, chunk.len() as u32)? };
                }
                return Ok(());
            }
        }
        self.send_pio(tx)
    }

    fn send_pio(&self, tx: &[u8]) -> Result<(), SpiError> {
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

    /// Receive-only SPI transfer.
    ///
    /// PIO clocks out 0xFF as dummy TX. The DMA path (`rx.len() >=
    /// DMA_MIN_LEN` with [`Spi::set_dma`]) lets the controller generate the
    /// dummy bytes itself, which is fine for SPI NOR data phases. Use PIO
    /// (no `set_dma`) for devices that require 0xFF on MOSI, such as SD
    /// cards in SPI mode.
    pub fn receive(&self, rx: &mut [u8]) -> Result<(), SpiError> {
        if let Some(d) = self.dma {
            if rx.len() >= DMA_MIN_LEN {
                // Split into an unaligned head, a cache-line aligned interior
                // (DMA) and an unaligned tail, all in this one CS window.
                let start = rx.as_ptr() as usize;
                let head = ((start + LINE - 1) & !(LINE - 1)) - start;
                let body = (rx.len() - head) & !(LINE - 1);
                let (head_buf, rest) = rx.split_at_mut(head);
                let (body_buf, tail_buf) = rest.split_at_mut(body);

                self.receive_pio(head_buf)?;
                for chunk in body_buf.chunks_mut(DMA_BURST_MAX) {
                    let addr = chunk.as_mut_ptr() as u32;
                    let len = chunk.len() as u32;
                    // SAFETY: `chunk` is exclusively borrowed for the whole
                    // blocking burst and covers whole cache lines only.
                    unsafe { self.dma_burst(d, None, Some(addr), len)? };
                    // Drop any line the CPU speculatively refetched meanwhile.
                    mmu::invalidate_dcache(addr, len);
                }
                return self.receive_pio(tail_buf);
            }
        }
        self.receive_pio(rx)
    }

    fn receive_pio(&self, rx: &mut [u8]) -> Result<(), SpiError> {
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

    /// One DMA-driven SPI burst of `len` bytes (≤ `DMA_BURST_MAX`): TX-only
    /// (`tx` = source address) or RX-only (`rx` = destination address).
    /// Sequence after Linux `spi-sun6i.c`: FIFO reset + DRQ enable →
    /// counters → start DMA → XCH → wait for transfer-complete and DMA.
    ///
    /// # Safety
    /// `tx` / `rx` must point to `len` valid bytes that stay untouched by
    /// the CPU until this returns.
    unsafe fn dma_burst(&self, d: SpiDma, tx: Option<u32>, rx: Option<u32>, len: u32) -> Result<(), SpiError> {
        let b = self.base;
        let drq = self.drq();

        spi_write(b, reg::STA, !0); // clear stale status (W1C)
        self.reset_fifos();
        let mut f = (DMA_TX_TRIG << fctl::TX_TRIG_SHIFT) | (DMA_RX_TRIG << fctl::RX_TRIG_SHIFT);
        if tx.is_some() { f |= fctl::TF_DRQ_EN; }
        if rx.is_some() { f |= fctl::RF_DRQ_EN; }
        spi_write(b, reg::FCTL, f);

        // TX-only: discard what comes back so the RX FIFO never fills
        // (TP_EN would otherwise pause the bus).
        let t = spi_read(b, reg::TCTRL);
        spi_write(b, reg::TCTRL, if rx.is_none() { t | tctrl::DHB } else { t & !tctrl::DHB });

        let tx_len = if tx.is_some() { len } else { 0 };
        spi_write(b, reg::BC, len);     // MBC: total burst
        spi_write(b, reg::TC, tx_len);  // MTC: bytes from TX FIFO
        spi_write(b, reg::BCC, tx_len); // BCC/STC: single-mode TX count

        let fifo = |off: u32| dma::Endpoint::ndma(b + off, drq)
            .width(dma::Width::Bits8).burst(dma::Burst::Single);
        let mem = |addr: u32| dma::Endpoint::memory(addr)
            .width(dma::Width::Bits8).burst(dma::Burst::Single);

        let started = (|| -> Result<(), dma::Error> {
            if let Some(dst) = rx {
                dma::start(d.rx, &dma::Transfer::new(fifo(reg::RXD), mem(dst), len))?;
            }
            if let Some(src) = tx {
                dma::start(d.tx, &dma::Transfer::new(mem(src), fifo(reg::TXD), len))?;
            }
            Ok(())
        })();
        let result = match started {
            Err(e) => Err(SpiError::Dma(e)),
            Ok(()) => {
                spi_write(b, reg::TCTRL, spi_read(b, reg::TCTRL) | tctrl::XCH);
                self.dma_wait(d)
            }
        };

        if result.is_err() {
            dma::stop(d.tx);
            dma::stop(d.rx);
        }
        // Back to the PIO configuration: no DRQs, RX kept.
        spi_write(b, reg::FCTL, 0);
        spi_write(b, reg::TCTRL, spi_read(b, reg::TCTRL) & !(tctrl::DHB | tctrl::XCH));
        result
    }

    /// Wait for the SPI exchange to finish and both DMA channels to go idle.
    fn dma_wait(&self, d: SpiDma) -> Result<(), SpiError> {
        let mut timeout = DMA_TIMEOUT;
        loop {
            let xch = unsafe { spi_read(self.base, reg::TCTRL) } & tctrl::XCH != 0;
            // poll() also advances the channels when IRQs are off.
            let tx_busy = dma::poll(d.tx);
            let rx_busy = dma::poll(d.rx);
            if !xch && !tx_busy && !rx_busy {
                unsafe { spi_write(self.base, reg::STA, sta::TC) };
                return Ok(());
            }
            if timeout == 0 {
                return Err(SpiError::Timeout);
            }
            timeout -= 1;
        }
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

    /// Single CS-asserted transaction: send `cmd` (MISO discarded), then receive
    /// `rx` (clocking 0xFF), then de-assert CS.
    ///
    /// SPI NOR read commands (e.g. 0x03) require CS to stay low across the
    /// command/address bytes AND the data bytes — splitting them into two
    /// `xfer`/`xfer_receive` calls de-asserts CS in between, aborting the read
    /// so the data phase returns garbage.
    pub fn xfer_cmd_read(&self, cmd: &[u8], rx: &mut [u8]) -> Result<(), SpiError> {
        unsafe { self.cs_assert(); }
        // Command phase: send cmd bytes, discard whatever the flash drives back.
        let result = self.transfer(cmd, &mut []).and_then(|_| self.receive(rx));
        unsafe { self.cs_deassert(); }
        result
    }

    /// Single CS-asserted transaction: send `cmd` then send `data` (both TX-only,
    /// MISO ignored), then de-assert CS.
    ///
    /// SPI NOR page-program (0x02) requires CS to stay low across the
    /// command/address bytes AND the data bytes.
    pub fn xfer_cmd_send(&self, cmd: &[u8], data: &[u8]) -> Result<(), SpiError> {
        unsafe { self.cs_assert(); }
        let result = self.send(cmd).and_then(|_| self.send(data));
        unsafe { self.cs_deassert(); }
        result
    }
}

/// SPI error types
#[derive(Debug)]
pub enum SpiError {
    Timeout,
    /// A DMA channel could not be started (e.g. `dma::init` not called,
    /// channel busy)
    Dma(dma::Error),
}

// ── Pre-defined Pin Configurations ────────────────────────────────────────

/// SPI0 pins: PC0=CS, PC1=CLK, PC2=MISO, PC3=MOSI.
///
/// SPI0 is the **first alternate function** on PC0–PC3 — pin-mux value `2`.
/// In this HAL the `function::FUNCn` constants are off-by-one (`FUNC1 = 0x02`,
/// `FUNC2 = 0x03`, …), so mux value 2 is `FUNC1`, NOT `FUNC2`. The proven SPL
/// `sys_copyself` and xboot `sys_spinor_init` both write the literal `2` here;
/// using `FUNC2` (value 3) muxes the pins to the wrong function so the SPI
/// signals never reach the chip and every transfer times out (JEDEC reads 00).
pub const SPI0_PINS: SpiPins = SpiPins {
    clk_port: Port::C, clk_pin: 1,
    mosi_port: Port::C, mosi_pin: 3,
    miso_port: Port::C, miso_pin: 2,
    cs_port: Port::C, cs_pin: 0,
    pin_func: gpio::function::FUNC1,
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
