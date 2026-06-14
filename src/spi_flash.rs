//! SPI NOR Flash driver for Allwinner F1C100s
//!
//! Provides a minimal SPI NOR flash interface for common chips
//! (Winbond W25Qxx, GigaDevice GD25Qxx, etc.).
//!
//! ## Supported Operations
//! - Read JEDEC ID (manufacturer + device ID)
//! - Read data (standard read, 03h)
//! - Write data (page program, 02h) — caller must erase first
//! - Sector erase (4KB, 20h) / Block erase (32KB/64KB)
//! - Read status register
//! - Write enable / disable
//!
//! ## Usage
//! ```ignore
//! use f1c100s::spi::{self, SpiMode, BitOrder, ChipSelect};
//! use f1c100s::spi_flash::SpiFlash;
//!
//! let spi = spi::spi0();
//! spi.configure(SpiMode::Mode0, BitOrder::MsbFirst, ChipSelect::Ss0, 10_000_000);
//!
//! let mut flash = SpiFlash::new(&spi);
//! let id = flash.read_jedec_id().unwrap();
//! ```

use crate::spi::Spi;

// ── SPI Flash Commands ────────────────────────────────────────────────────

#[allow(dead_code)]
mod cmd {
    pub const WRITE_ENABLE: u8 = 0x06;
    pub const WRITE_DISABLE: u8 = 0x04;
    pub const READ_STATUS: u8 = 0x05;
    pub const READ_STATUS2: u8 = 0x35;
    pub const WRITE_STATUS: u8 = 0x01;
    pub const READ_DATA: u8 = 0x03;
    pub const FAST_READ: u8 = 0x0B;
    pub const PAGE_PROGRAM: u8 = 0x02;
    pub const SECTOR_ERASE_4K: u8 = 0x20;
    pub const BLOCK_ERASE_32K: u8 = 0x52;
    pub const BLOCK_ERASE_64K: u8 = 0xD8;
    pub const CHIP_ERASE: u8 = 0xC7;
    pub const READ_JEDEC_ID: u8 = 0x9F;
    pub const READ_UNIQUE_ID: u8 = 0x4B;
    pub const POWER_DOWN: u8 = 0xB9;
    pub const RELEASE_POWER_DOWN: u8 = 0xAB;
}

// ── Status Register Bits ──────────────────────────────────────────────────

pub mod status {
    pub const BUSY: u8 = 1 << 0;
    pub const WEL: u8 = 1 << 1;   // Write Enable Latch
}

// ── SPI Flash Driver ──────────────────────────────────────────────────────

/// SPI NOR Flash driver
pub struct SpiFlash<'a> {
    spi: &'a Spi,
}

impl<'a> SpiFlash<'a> {
    /// Create a new SPI flash driver using the given SPI bus.
    /// The SPI bus must already be configured.
    pub fn new(spi: &'a Spi) -> Self {
        Self { spi }
    }

    // ── Low-level helpers ──────────────────────────────────────────────

    fn write_enable(&self) -> Result<(), FlashError> {
        self.spi.xfer_send(&[cmd::WRITE_ENABLE])?;
        Ok(())
    }

    fn wait_busy(&self) -> Result<(), FlashError> {
        loop {
            let sr = self.read_status()?;
            if sr & status::BUSY == 0 {
                break;
            }
            core::hint::spin_loop();
        }
        Ok(())
    }

    // ── Identification ─────────────────────────────────────────────────

    /// Read the 3-byte JEDEC ID (manufacturer, memory_type, capacity)
    pub fn read_jedec_id(&self) -> Result<[u8; 3], FlashError> {
        let mut id = [0u8; 3];
        self.spi.xfer(&[cmd::READ_JEDEC_ID, 0, 0, 0], &mut id)?;
        // Response is in bytes 1–3 (byte 0 is dummy during command phase)
        let result = [id[1], id[2], id[0]]; // reorder for typical manuf-type-capacity
        Ok(result)
    }

    /// Read the 8-byte unique ID
    pub fn read_unique_id(&self) -> Result<[u8; 8], FlashError> {
        let cmd = [cmd::READ_UNIQUE_ID, 0, 0, 0, 0];
        let mut id = [0u8; 12]; // 4 dummy + 8 data
        self.spi.xfer(&cmd, &mut id)?;
        let mut result = [0u8; 8];
        result.copy_from_slice(&id[4..12]);
        Ok(result)
    }

    // ── Status ─────────────────────────────────────────────────────────

    /// Read the status register
    pub fn read_status(&self) -> Result<u8, FlashError> {
        let mut rx = [0u8; 2];
        self.spi.xfer(&[cmd::READ_STATUS, 0], &mut rx)?;
        Ok(rx[1])
    }

    /// Check if the flash is busy
    pub fn is_busy(&self) -> Result<bool, FlashError> {
        Ok(self.read_status()? & status::BUSY != 0)
    }

    // ── Read ───────────────────────────────────────────────────────────

    /// Read data from flash at the given 24-bit address.
    pub fn read(&self, addr: u32, buf: &mut [u8]) -> Result<(), FlashError> {
        let cmd = [
            cmd::READ_DATA,
            ((addr >> 16) & 0xFF) as u8,
            ((addr >> 8) & 0xFF) as u8,
            (addr & 0xFF) as u8,
        ];
        // Command + address + data MUST share one CS-asserted transaction; a
        // de-assert between phases aborts the read and the data comes back as
        // garbage.
        self.spi.xfer_cmd_read(&cmd, buf)?;
        Ok(())
    }

    // ── Erase ──────────────────────────────────────────────────────────

    /// Erase a 4 KB sector
    pub fn sector_erase_4k(&self, addr: u32) -> Result<(), FlashError> {
        self.write_enable()?;
        let cmd = [
            cmd::SECTOR_ERASE_4K,
            ((addr >> 16) & 0xFF) as u8,
            ((addr >> 8) & 0xFF) as u8,
            (addr & 0xFF) as u8,
        ];
        self.spi.xfer_send(&cmd)?;
        self.wait_busy()?;
        Ok(())
    }

    /// Erase a 32 KB block
    pub fn block_erase_32k(&self, addr: u32) -> Result<(), FlashError> {
        self.write_enable()?;
        let cmd = [
            cmd::BLOCK_ERASE_32K,
            ((addr >> 16) & 0xFF) as u8,
            ((addr >> 8) & 0xFF) as u8,
            (addr & 0xFF) as u8,
        ];
        self.spi.xfer_send(&cmd)?;
        self.wait_busy()?;
        Ok(())
    }

    /// Erase a 64 KB block
    pub fn block_erase_64k(&self, addr: u32) -> Result<(), FlashError> {
        self.write_enable()?;
        let cmd = [
            cmd::BLOCK_ERASE_64K,
            ((addr >> 16) & 0xFF) as u8,
            ((addr >> 8) & 0xFF) as u8,
            (addr & 0xFF) as u8,
        ];
        self.spi.xfer_send(&cmd)?;
        self.wait_busy()?;
        Ok(())
    }

    /// Erase entire chip
    pub fn chip_erase(&self) -> Result<(), FlashError> {
        self.write_enable()?;
        self.spi.xfer_send(&[cmd::CHIP_ERASE])?;
        self.wait_busy()?;
        Ok(())
    }

    // ── Write ──────────────────────────────────────────────────────────

    /// Program up to 256 bytes at the given address (page program).
    /// The write must not cross a page boundary (256 bytes).
    /// The target area must be erased first.
    pub fn page_program(&self, addr: u32, data: &[u8]) -> Result<(), FlashError> {
        if data.len() > 256 {
            return Err(FlashError::TooLarge);
        }
        self.write_enable()?;
        let cmd = [
            cmd::PAGE_PROGRAM,
            ((addr >> 16) & 0xFF) as u8,
            ((addr >> 8) & 0xFF) as u8,
            (addr & 0xFF) as u8,
        ];
        // Command + address + data MUST share one CS-asserted transaction; a
        // de-assert between phases aborts the page program after the address so
        // no data is written.
        self.spi.xfer_cmd_send(&cmd, data)?;
        self.wait_busy()?;
        Ok(())
    }

    /// Write data spanning multiple pages. Handles page alignment.
    pub fn write(&self, addr: u32, data: &[u8]) -> Result<(), FlashError> {
        let page_size: u32 = 256;
        let mut offset: u32 = 0;

        while offset < data.len() as u32 {
            let page_addr = addr + offset;
            let page_remain = page_size - (page_addr & (page_size - 1));
            let chunk = page_remain.min(data.len() as u32 - offset) as usize;

            self.page_program(page_addr, &data[offset as usize..offset as usize + chunk])?;
            offset += chunk as u32;
        }
        Ok(())
    }

    // ── Power Management ───────────────────────────────────────────────

    /// Enter deep power-down mode
    pub fn power_down(&self) -> Result<(), FlashError> {
        self.spi.xfer_send(&[cmd::POWER_DOWN])?;
        Ok(())
    }

    /// Release from deep power-down mode
    pub fn release_power_down(&self) -> Result<(), FlashError> {
        self.spi.xfer_send(&[cmd::RELEASE_POWER_DOWN])?;
        // Wait typical tRES1 (3 µs)
        for _ in 0..100 {
            core::hint::spin_loop();
        }
        Ok(())
    }
}

// ── Errors ────────────────────────────────────────────────────────────────

#[derive(Debug)]
pub enum FlashError {
    Spi,
    Timeout,
    TooLarge,
}

impl From<crate::spi::SpiError> for FlashError {
    fn from(_: crate::spi::SpiError) -> Self {
        FlashError::Spi
    }
}
