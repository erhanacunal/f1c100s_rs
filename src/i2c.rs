//! TWI / I2C driver for Allwinner F1C100s
//!
//! The F1C100s provides three TWI (Two-Wire Interface) controllers
//! that implement I²C master operation:
//! - Standard mode (100 kHz) and Fast mode (400 kHz)
//! - 7-bit addressing
//! - Interrupt-driven operation with thread-blocking via [`crate::ipc::EventFlags`]
//!
//! ## Architecture
//!
//! This driver is **interrupt-driven**: the calling thread is blocked on an
//! [`EventFlags`] object while the TWI state machine runs inside the ISR.
//! When the transfer completes (or an error occurs), the ISR sets the event
//! and wakes the thread.  This is the same pattern used by rt-thread's
//! Allwinner TINA I2C driver.
//!
//! ## Base Addresses
//! - TWI0: `0x01C27000` (default: PD0=SCL, PD12=SDA, func2)
//! - TWI1: `0x01C27400` (default: PD5=SCL, PD6=SDA, func2)
//! - TWI2: `0x01C27800` (default: PD15=SCL, PD16=SDA, func3)
//!
//! ## Clock Derivation
//! F_scl = APB_CLK / (2^CLK_N × (CLK_M + 1) × 10)

use crate::clock::{self, BusGate};
use crate::gpio::{self, Port, PullMode, DriveLevel};
use crate::interrupt;
use crate::ipc::{EventFlags, EventMode};
use crate::timer;
use log::debug;

// ── Base Addresses ────────────────────────────────────────────────────────

pub const TWI0_BASE: u32 = 0x01C27000;
pub const TWI1_BASE: u32 = 0x01C27400;
pub const TWI2_BASE: u32 = 0x01C27800;

// ── Register Offsets ──────────────────────────────────────────────────────

#[allow(dead_code)]
mod reg {
    pub const ADDR: u32 = 0x00;
    pub const XADDR: u32 = 0x04;
    pub const DATA: u32 = 0x08;
    pub const CTL: u32 = 0x0C;
    pub const STAT: u32 = 0x10;
    pub const CLK: u32 = 0x14;
    pub const SRST: u32 = 0x18;
    pub const EFR: u32 = 0x1C;
    pub const LCR: u32 = 0x20;
    pub const DVFS: u32 = 0x24;
}

// ── Control Register Bits ─────────────────────────────────────────────────

#[allow(dead_code)]
mod ctl {
    pub const ACK: u32 = 1 << 2;
    pub const INTFLG: u32 = 1 << 3;
    pub const STP: u32 = 1 << 4;
    pub const STA: u32 = 1 << 5;
    pub const BUSEN: u32 = 1 << 6;
    pub const INTEN: u32 = 1 << 7;
}

// ── Status Register Codes ─────────────────────────────────────────────────

#[allow(dead_code)]
mod stat {
    pub const MASK: u32 = 0xFF;
    pub const BUS_ERR: u8 = 0x00;
    pub const TX_STA: u8 = 0x08;
    pub const TX_RESTA: u8 = 0x10;
    pub const TX_AW_ACK: u8 = 0x18;
    pub const TX_AW_NAK: u8 = 0x20;
    pub const TXD_ACK: u8 = 0x28;
    pub const TXD_NAK: u8 = 0x30;
    pub const ARB_LOST: u8 = 0x38;
    pub const TX_AR_ACK: u8 = 0x40;
    pub const TX_AR_NAK: u8 = 0x48;
    pub const RXD_ACK: u8 = 0x50;
    pub const RXD_NAK: u8 = 0x58;
    pub const TXD_ADDR2_ACK: u8 = 0xD0;
    pub const TXD_ADDR2_NAK: u8 = 0xD8;
    pub const IDLE: u8 = 0xF8;
}

// ── LCR Register Bits ─────────────────────────────────────────────────────

#[allow(dead_code)]
mod lcr {
    pub const SDA_EN: u32 = 1 << 0;
    pub const SDA_CTL: u32 = 1 << 1;
    pub const SCL_EN: u32 = 1 << 2;
    pub const SCL_CTL: u32 = 1 << 3;
    pub const SDA_STATE: u32 = 1 << 4;
    pub const SCL_STATE: u32 = 1 << 5;
    pub const IDLE_STATUS: u32 = 0x3A;
}

// ── Return Codes ──────────────────────────────────────────────────────────

#[derive(Debug, PartialEq, Eq)]
pub enum I2cError {
    Ok,
    Fail,
    Busy,
    StartFail,
    StopFail,
    Nack,
    ArbLost,
    Timeout,
}

// ── MMIO Helpers ──────────────────────────────────────────────────────────

#[inline]
unsafe fn twi_read(base: u32, offset: u32) -> u32 {
    ((base + offset) as *const u32).read_volatile()
}

#[inline]
unsafe fn twi_write(base: u32, offset: u32, val: u32) {
    ((base + offset) as *mut u32).write_volatile(val);
}

// ── Internal Helpers ──────────────────────────────────────────────────────

#[inline]
unsafe fn twi_get_status(base: u32) -> u8 {
    (twi_read(base, reg::STAT) & stat::MASK) as u8
}

#[inline]
unsafe fn twi_query_irq_flag(base: u32) -> bool {
    twi_read(base, reg::CTL) & ctl::INTFLG != 0
}

#[inline]
unsafe fn twi_clear_irq_flag(base: u32) {
    let val = twi_read(base, reg::CTL) & !ctl::INTFLG;
    twi_write(base, reg::CTL, val);
    let check = twi_read(base, reg::CTL);
    if check & ctl::INTFLG != 0 {
        twi_write(base, reg::CTL, check & !ctl::INTFLG);
    }
}

#[inline]
unsafe fn twi_enable_bus(base: u32) {
    let val = twi_read(base, reg::CTL) | ctl::BUSEN;
    twi_write(base, reg::CTL, val);
}

#[inline]
unsafe fn twi_disable_bus(base: u32) {
    let val = twi_read(base, reg::CTL) & !ctl::BUSEN;
    twi_write(base, reg::CTL, val);
}

#[inline]
unsafe fn twi_enable_ack(base: u32) {
    let val = twi_read(base, reg::CTL) | ctl::ACK;
    twi_write(base, reg::CTL, val);
}

#[inline]
unsafe fn twi_disable_ack(base: u32) {
    let val = twi_read(base, reg::CTL) & !ctl::ACK;
    twi_write(base, reg::CTL, val);
}

/// Enable TWI interrupt in the CTL register.
/// Per reference C: set INTEN|INTFLG (to prevent accidental clear),
/// clear STA|STP (to prevent double-trigger).
unsafe fn twi_enable_irq(base: u32) {
    let val = twi_read(base, reg::CTL);
    let val = (val | (ctl::INTEN | ctl::INTFLG)) & !(ctl::STA | ctl::STP);
    twi_write(base, reg::CTL, val);
}

/// Disable TWI interrupt.  Written twice as per reference C.
unsafe fn twi_disable_irq(base: u32) {
    let val = twi_read(base, reg::CTL) & !ctl::INTEN;
    twi_write(base, reg::CTL, val);
    twi_write(base, reg::CTL, val);
}

#[inline]
unsafe fn twi_set_start(base: u32) {
    let val = twi_read(base, reg::CTL) | ctl::STA;
    twi_write(base, reg::CTL, val);
}

unsafe fn twi_get_start(base: u32) -> bool {
    (twi_read(base, reg::CTL) >> 5) & 1 != 0
}

#[inline]
unsafe fn twi_set_stop(base: u32) {
    let val = twi_read(base, reg::CTL) | ctl::STP;
    twi_write(base, reg::CTL, val);
}

unsafe fn twi_get_stop(base: u32) -> bool {
    twi_read(base, reg::CTL) & ctl::STP != 0
}

#[inline]
unsafe fn twi_put_byte(base: u32, byte: u8) {
    twi_write(base, reg::DATA, byte as u32);
    twi_clear_irq_flag(base);
}

#[inline]
unsafe fn twi_get_byte(base: u32) -> u8 {
    let val = twi_read(base, reg::DATA);
    let byte = (val & 0xFF) as u8;
    twi_clear_irq_flag(base);
    byte
}

#[inline]
unsafe fn twi_get_last_byte(base: u32) -> u8 {
    let val = twi_read(base, reg::DATA);
    (val & 0xFF) as u8
}

#[inline]
unsafe fn twi_send_addr(base: u32, addr: u8, read: bool) {
    let tmp = ((addr & 0x7F) << 1) | (read as u8);
    twi_put_byte(base, tmp);
}

#[inline]
unsafe fn twi_soft_reset(base: u32) {
    twi_write(base, reg::SRST, 1 << 0);
}

#[inline]
unsafe fn twi_set_efr(base: u32, efr: u32) {
    let val = twi_read(base, reg::EFR);
    let val = (val & !0x3) | (efr & 0x3);
    twi_write(base, reg::EFR, val);
}

/// Wait for START bit to self-clear (polling helper).
unsafe fn twi_wait_start_clear(base: u32) -> Result<(), I2cError> {
    let mut timeout: u32 = 0xFF;
    while twi_get_start(base) && timeout > 0 {
        timeout -= 1;
        core::hint::spin_loop();
    }
    if timeout == 0 {
        Err(I2cError::StartFail)
    } else {
        Ok(())
    }
}

/// Send STOP and poll until bus is fully idle.
unsafe fn twi_send_stop_and_wait(base: u32) -> Result<(), I2cError> {
    twi_set_stop(base);
    twi_clear_irq_flag(base);

    // Dummy read to delay 1 cycle (as per reference C)
    twi_get_stop(base);

    let mut timeout: u32 = 0xFF;
    while twi_get_stop(base) && timeout > 0 {
        timeout -= 1;
        core::hint::spin_loop();
    }
    if timeout == 0 {
        return Err(I2cError::StopFail);
    }

    timeout = 0xFF;
    while twi_get_status(base) != stat::IDLE && timeout > 0 {
        timeout -= 1;
        core::hint::spin_loop();
    }
    if timeout == 0 {
        return Err(I2cError::StopFail);
    }

    timeout = 0xFF;
    while twi_read(base, reg::LCR) != lcr::IDLE_STATUS && timeout > 0 {
        timeout -= 1;
        core::hint::spin_loop();
    }
    if timeout == 0 {
        return Err(I2cError::StopFail);
    }

    Ok(())
}

/// Send STOP without checking (error recovery).
unsafe fn twi_stop_quiet(base: u32) {
    twi_set_stop(base);
    twi_clear_irq_flag(base);
    for _ in 0..0xFF {
        core::hint::spin_loop();
    }
    twi_soft_reset(base);
}

/// Compute and write clock divider for the requested SCL frequency.
///
/// F_scl = APB_CLK / (2^N × (M+1) × 10)
unsafe fn twi_set_clock(base: u32, apb_clk: u32, scl_hz: u32) {
    let src_clk = apb_clk / 10;
    let divider = src_clk / scl_hz;

    if divider == 0 {
        twi_write(base, reg::CLK, 1 << 3); // M=1, N=0
        return;
    }

    let mut clk_n: u32 = 0;
    let mut pow_n: u32 = 1;
    while clk_n < 8 {
        let mut clk_m = divider.wrapping_div(pow_n).saturating_sub(1);
        while clk_m < 16 {
            let sclk_real = src_clk / (clk_m + 1) / pow_n;
            if sclk_real <= scl_hz {
                twi_write(base, reg::CLK, (clk_m << 3) | clk_n);
                return;
            }
            clk_m += 1;
        }
        clk_n += 1;
        pow_n *= 2;
    }

    // Fallback
    twi_write(base, reg::CLK, 1 | 7);
}

// ── Per-Channel Transfer State ────────────────────────────────────────────

/// Event flag bit used to wake the blocked xfer thread.
const TWI_WAKEUP_EVENT: u32 = 1 << 0;

/// Transfer status.
#[derive(Clone, Copy, PartialEq, Eq)]
enum XferStatus {
    Idle = 0x1,
    Start = 0x2,
    Running = 0x4,
}

/// A single I2C message within a transfer.
struct TwiMsg {
    addr: u8,
    /// 0 = write, 1 = read
    read: bool,
    buf: *mut u8,
    len: usize,
}

/// Per-channel state stored in a static so both ISR and thread can access it.
struct TwiChannel {
    event: EventFlags,
    msgs: [Option<TwiMsg>; 2],
    msg_idx: usize,
    msg_ptr: usize,
    msg_num: usize,
    status: XferStatus,
    #[allow(dead_code)]
    debug_state: u8,
}

impl TwiChannel {
    const fn new() -> Self {
        Self {
            event: EventFlags::new(),
            msgs: [const { None }, const { None }],
            msg_idx: 0,
            msg_ptr: 0,
            msg_num: 0,
            status: XferStatus::Idle,
            debug_state: 0,
        }
    }

    unsafe fn reset(&mut self) {
        self.msgs = [None, None];
        self.msg_idx = 0;
        self.msg_ptr = 0;
        self.msg_num = 0;
        self.status = XferStatus::Idle;
    }
}

static mut TWI0_CH: TwiChannel = TwiChannel::new();
static mut TWI1_CH: TwiChannel = TwiChannel::new();
static mut TWI2_CH: TwiChannel = TwiChannel::new();

/// Get a raw-mut pointer to the channel for the given base address.
unsafe fn twi_channel_for_base(base: u32) -> *mut TwiChannel {
    match base {
        TWI0_BASE => &raw mut TWI0_CH,
        TWI1_BASE => &raw mut TWI1_CH,
        TWI2_BASE => &raw mut TWI2_CH,
        _ => panic!("unknown TWI base {:08X}", base),
    }
}

fn twi_int_vector(base: u32) -> u32 {
    match base {
        TWI0_BASE => interrupt::TWI0_INTERRUPT,
        TWI1_BASE => interrupt::TWI1_INTERRUPT,
        TWI2_BASE => interrupt::TWI2_INTERRUPT,
        _ => panic!("unknown TWI base {:08X}", base),
    }
}

// ── Transfer Completion ───────────────────────────────────────────────────

/// Called at end of transfer (success or error) inside the ISR.
///Cleans up state and wakes the blocked thread.
unsafe fn twi_xfer_complete(_base: u32, ch: *mut TwiChannel, code: i32) {
    let ch_ref = &mut *ch;
    ch_ref.msgs = [None, None];
    ch_ref.msg_num = 0;
    ch_ref.msg_ptr = 0;
    ch_ref.status = XferStatus::Idle;

    if code != 0 {
        ch_ref.msg_idx = code as usize;
    } else {
        ch_ref.msg_idx = 0; // success: matches cleared msg_num below
    }

    ch_ref.event.set(TWI_WAKEUP_EVENT);
}

// ── Interrupt Service Routines ────────────────────────────────────────────

/// The core I2C state machine — called from the ISR on each TWI interrupt.
/// Reads the status register and transitions through the protocol states.
unsafe fn twi_core_process(base: u32) {
    let ch = twi_channel_for_base(base);
    let ch_ref = &mut *ch;

    let state = twi_get_status(base);
    let msg_idx = ch_ref.msg_idx;
    let msg_num = ch_ref.msg_num;

    if msg_idx >= msg_num {
        twi_xfer_complete(base, ch, -1);
        return;
    }

    let cur_msg = match &ch_ref.msgs[msg_idx] {
        Some(m) => m,
        None => {
            twi_xfer_complete(base, ch, -1);
            return;
        }
    };

    match state {
        // Idle / bus error in ISR — unexpected
        0xF8 | 0x00 => {
            twi_xfer_complete(base, ch, -(state as i32));
            return;
        }

        // ── START transmitted → send slave address ────────────────────
        0x08 | 0x10 => {
            twi_send_addr(base, cur_msg.addr, cur_msg.read);
        }

        // ── ADDR+R NACK / second-addr NACK ───────────────────────────
        0xD8 | 0x20 => {
            twi_xfer_complete(base, ch, -0x20);
        }

        // ── ADDR+W ACK → send first data byte ────────────────────────
        0x18 => {
            if ch_ref.msg_ptr < cur_msg.len {
                let byte = *cur_msg.buf.add(ch_ref.msg_ptr);
                twi_put_byte(base, byte);
                ch_ref.msg_ptr += 1;
            } else {
                ch_ref.msg_idx += 1;
                ch_ref.msg_ptr = 0;
                if ch_ref.msg_idx >= ch_ref.msg_num {
                    twi_xfer_complete(base, ch, 0);
                } else {
                    twi_set_start(base);
                    twi_clear_irq_flag(base);
                    twi_wait_start_clear(base).ok();
                }
            }
        }

        // ── Data-byte / second-addr ACK → send next byte or restart
        0xD0 | 0x28 => {
            if ch_ref.msg_ptr < cur_msg.len {
                let byte = *cur_msg.buf.add(ch_ref.msg_ptr);
                twi_put_byte(base, byte);
                ch_ref.msg_ptr += 1;
            } else {
                ch_ref.msg_idx += 1;
                ch_ref.msg_ptr = 0;
                if ch_ref.msg_idx >= ch_ref.msg_num {
                    twi_xfer_complete(base, ch, 0);
                } else {
                    twi_set_start(base);
                    twi_clear_irq_flag(base);
                    twi_wait_start_clear(base).ok();
                }
            }
        }

        // ── Data byte NACK ───────────────────────────────────────────
        0x30 => {
            twi_xfer_complete(base, ch, -0x30);
        }

        // ── Arbitration lost ─────────────────────────────────────────
        0x38 => {
            twi_xfer_complete(base, ch, -0x38);
        }

        // ── ADDR+R ACK → enable ACK, clear INTFLG to receive ────────
        0x40 => {
            if cur_msg.len > 1 {
                twi_enable_ack(base);
            }
            twi_clear_irq_flag(base);
        }

        // ── ADDR+R NACK ─────────────────────────────────────────────
        0x48 => {
            twi_xfer_complete(base, ch, -0x48);
        }

        // ── Data byte received, ACK sent ─────────────────────────────
        0x50 => {
            if ch_ref.msg_ptr < cur_msg.len {
                if ch_ref.msg_ptr + 2 == cur_msg.len {
                    twi_disable_ack(base);
                }
                let byte = twi_get_byte(base);
                *cur_msg.buf.add(ch_ref.msg_ptr) = byte;
                ch_ref.msg_ptr += 1;
            } else {
                twi_xfer_complete(base, ch, -0x50);
            }
        }

        // ── Data byte received, NACK sent (last byte) ────────────────
        0x58 => {
            if ch_ref.msg_ptr == cur_msg.len.wrapping_sub(1) {
                let byte = twi_get_last_byte(base);
                *cur_msg.buf.add(ch_ref.msg_ptr) = byte;
                ch_ref.msg_idx += 1;
                ch_ref.msg_ptr = 0;
                if ch_ref.msg_idx >= ch_ref.msg_num {
                    twi_xfer_complete(base, ch, 0);
                } else {
                    twi_set_start(base);
                    twi_clear_irq_flag(base);
                    twi_wait_start_clear(base).ok();
                }
            } else {
                twi_xfer_complete(base, ch, -0x58);
            }
        }

        // ── Unknown state ────────────────────────────────────────────
        _ => {
            twi_xfer_complete(base, ch, -(state as i32));
        }
    }

    ch_ref.debug_state = state;
}

/// Generic TWI ISR.  Checks INTFLG, disables IE, runs the state machine,
/// then re-enables IE only if the transfer is still in progress.
unsafe fn twi_isr(base: u32, ch: *mut TwiChannel) {
    if !twi_query_irq_flag(base) {
        return;
    }

    twi_disable_irq(base);
    twi_core_process(base);

    if (*ch).status == XferStatus::Running {
        twi_enable_irq(base);
    }
}

// Per-channel ISR wrappers

fn twi0_isr(_vector: u32) {
    unsafe { twi_isr(TWI0_BASE, &raw mut TWI0_CH); }
}

fn twi1_isr(_vector: u32) {
    unsafe { twi_isr(TWI1_BASE, &raw mut TWI1_CH); }
}

fn twi2_isr(_vector: u32) {
    unsafe { twi_isr(TWI2_BASE, &raw mut TWI2_CH); }
}

// ── TWI Instance ──────────────────────────────────────────────────────────

/// Pin-mux configuration for one TWI bus
#[derive(Debug, Clone)]
pub struct TwiPins {
    pub scl_port: Port,
    pub scl_pin: u8,
    pub sda_port: Port,
    pub sda_pin: u8,
    pub pin_func: u8,
}

/// A configured TWI/I²C bus instance
pub struct Twi {
    base: u32,
    gate: BusGate,
}

impl Twi {
    /// Create a new TWI instance with the given base address and clock gate.
    ///
    /// # Safety
    /// The caller must ensure the base address corresponds to a real TWI controller.
    pub unsafe fn new(base: u32, gate: BusGate) -> Self {
        Self { base, gate }
    }

    /// Initialize the TWI bus: configure pins, enable clock, set clock rate,
    /// enable the bus, and install the interrupt handler.
    pub fn init(&self, pins: &TwiPins, apb_clk: u32, scl_hz: u32) {
        // Configure GPIO pins
        gpio::set_function(pins.scl_port, pins.scl_pin, pins.pin_func);
        gpio::set_pull_mode(pins.scl_port, pins.scl_pin, PullMode::Disable);
        gpio::set_drive_level(pins.scl_port, pins.scl_pin, DriveLevel::Level0);

        gpio::set_function(pins.sda_port, pins.sda_pin, pins.pin_func);
        gpio::set_pull_mode(pins.sda_port, pins.sda_pin, PullMode::Disable);
        gpio::set_drive_level(pins.sda_port, pins.sda_pin, DriveLevel::Level0);

        // Enable clock gate and release reset
        clock::bus_clk_init(self.gate);
        timer::delay_us(100);

        unsafe {
            twi_set_clock(self.base, apb_clk, scl_hz);
            twi_enable_bus(self.base);
        }

        // Install interrupt handler
        let isr: interrupt::IsrHandler = match self.base {
            TWI0_BASE => twi0_isr,
            TWI1_BASE => twi1_isr,
            TWI2_BASE => twi2_isr,
            _ => panic!("unknown TWI base {:08X}", self.base),
        };
        interrupt::install(twi_int_vector(self.base), isr);
        interrupt::unmask(twi_int_vector(self.base));
    }

    /// Deinitialize: mask interrupt, disable bus, disable clock gate.
    pub fn deinit(&self) {
        interrupt::mask(twi_int_vector(self.base));
        unsafe {
            twi_disable_irq(self.base);
            twi_disable_bus(self.base);
        }
        clock::bus_gate_disable(self.gate);
    }

    /// Return the TWI base address (for use by `probe`, etc.)
    pub fn base(&self) -> u32 {
        self.base
    }

    // ── Public Transfer API ──────────────────────────────────────────────

    /// Write data to a slave device.
    ///
    /// Performs: START → ADDR+W → [reg_addr?] → data... → STOP
    ///
    /// If `reg_addr` is `Some`, it is prepended to the data bytes.
    pub fn write(&self, addr: u8, reg_addr: Option<u8>, data: &[u8]) -> Result<usize, I2cError> {
        self.xfer(addr, reg_addr, Some(data), None)
    }

    /// Read data from a slave device.
    ///
    /// Performs:
    /// - Without `reg_addr`: START → ADDR+R → data... → STOP
    /// - With `reg_addr`: START → ADDR+W → reg_addr → RESTART → ADDR+R → data... → STOP
    pub fn read(&self, addr: u8, reg_addr: Option<u8>, buf: &mut [u8]) -> Result<usize, I2cError> {
        if buf.is_empty() {
            return Ok(0);
        }
        self.xfer(addr, reg_addr, None, Some(buf))
    }

    /// Write then read in a single combined transfer (with repeated START).
    pub fn write_read(
        &self,
        addr: u8,
        reg_addr: u8,
        buf: &mut [u8],
    ) -> Result<usize, I2cError> {
        if buf.is_empty() {
            return Ok(0);
        }
        self.xfer(addr, Some(reg_addr), None, Some(buf))
    }

    // ── Core Transfer ────────────────────────────────────────────────────

    /// Interrupt-driven transfer: builds internal messages, starts the
    /// transfer, blocks on EventFlags, and returns when the ISR finishes.
    fn xfer(
        &self,
        addr: u8,
        reg_addr: Option<u8>,
        write_data: Option<&[u8]>,
        mut read_buf: Option<&mut [u8]>,
    ) -> Result<usize, I2cError> {
        let base = self.base;
        let ch_ptr = unsafe { twi_channel_for_base(base) };

        let has_write = reg_addr.is_some()
            || write_data.as_ref().map_or(false, |d| !d.is_empty());
        let has_read = read_buf.as_ref().map_or(false, |b| !b.is_empty());

        // Stack buffer for the write phase.  Must live for the entire
        // xfer call — the ISR reads from it while the thread is blocked.
        let mut write_buf: [u8; 256] = [0u8; 256];
        let write_len: usize;

        unsafe {
            let ch = &mut *ch_ptr;

            // ── Pre-transfer checks ───────────────────────────────────
            if ch.status != XferStatus::Idle {
                return Err(I2cError::Busy);
            }

            twi_soft_reset(base);
            timer::delay_us(100);

            let mut timeout: u32 = 0xFF;
            while (twi_read(base, reg::SRST) & 1) != 0 && timeout > 0 {
                timeout -= 1;
                core::hint::spin_loop();
            }

            let st = twi_get_status(base);
            if st != stat::IDLE && st != stat::BUS_ERR {
                return Err(I2cError::Busy);
            }

            twi_enable_bus(base);
            twi_disable_irq(base);

            // ── Build messages ────────────────────────────────────────
            let mut msg_count: usize = 0;

            if has_write {
                let wdata = write_data.unwrap_or(&[]);
                let total_write_len = if reg_addr.is_some() { 1 + wdata.len() } else { wdata.len() };

                if total_write_len > 256 {
                    return Err(I2cError::Fail);
                }

                let mut idx = 0usize;
                if let Some(reg) = reg_addr {
                    write_buf[0] = reg;
                    idx = 1;
                }
                write_buf[idx..idx + wdata.len()].copy_from_slice(wdata);
                write_len = total_write_len;

                ch.msgs[0] = Some(TwiMsg {
                    addr,
                    read: false,
                    buf: write_buf.as_mut_ptr(),
                    len: total_write_len,
                });
                msg_count = 1;

                if has_read {
                    let rbuf = read_buf.as_mut().unwrap();
                    ch.msgs[1] = Some(TwiMsg {
                        addr,
                        read: true,
                        buf: rbuf.as_mut_ptr(),
                        len: rbuf.len(),
                    });
                    msg_count = 2;
                }
            } else if has_read {
                let rbuf = read_buf.as_mut().unwrap();
                ch.msgs[0] = Some(TwiMsg {
                    addr,
                    read: true,
                    buf: rbuf.as_mut_ptr(),
                    len: rbuf.len(),
                });
                write_len = 0;
                msg_count = 1;
            } else {
                return Ok(0);
            }

            ch.msg_idx = 0;
            ch.msg_ptr = 0;
            ch.msg_num = msg_count;
            // Mark Running BEFORE arming the IRQ and sending START. The TWI ISR
            // re-enables its own interrupt only while `status == Running`; if it
            // fires during the `twi_wait_start_clear` poll below (IRQs are
            // enabled here — this runs on the UI thread, not in a critical
            // section) and still saw the old `Start`, it would leave the IRQ
            // disabled and the transfer would stall until the event timeout.
            // That stall leaves the channel non-Idle, after which every later
            // call fast-returns Busy without blocking and the caller spins,
            // starving the scheduler. Setting Running first closes that window.
            ch.status = XferStatus::Running;
            ch.event.clear(TWI_WAKEUP_EVENT);

            // ── Arm interrupt and send START ──────────────────────────
            twi_enable_irq(base);
            twi_disable_ack(base);
            twi_set_efr(base, 0);

            twi_set_start(base);
            if twi_wait_start_clear(base).is_err() {
                twi_soft_reset(base);
                twi_disable_irq(base);
                ch.reset();
                return Err(I2cError::StartFail);
            }

            // ── Block until ISR completes the transfer ────────────────
            let result = ch.event.wait(
                TWI_WAKEUP_EVENT,
                EventMode::Or,
                true,  // clear on exit
                Some(10000),
            );

            if result.is_err() {
                twi_stop_quiet(base);
                twi_disable_irq(base);
                ch.reset();
                return Err(I2cError::Timeout);
            }

            let ret = ch.msg_idx;
            let num = ch.msg_num;

            if ret == num {
                // Success
                if let Err(e) = twi_send_stop_and_wait(base) {
                    ch.reset();
                    return Err(e);
                }
                let count = if has_read {
                    read_buf.as_ref().unwrap().len()
                } else {
                    write_len
                };
                ch.reset();
                Ok(count)
            } else {
                debug!("TWI: xfer incomplete, msg_idx={} (expected {})", ret, num);
                twi_stop_quiet(base);
                twi_disable_irq(base);
                ch.reset();
                Err(I2cError::Fail)
            }
        }
    }
}

// ── Pin Configurations ────────────────────────────────────────────────────

/// TWI0 default pins: PD0=SCL, PD12=SDA, func2
pub const TWI0_PINS: TwiPins = TwiPins {
    scl_port: Port::D,
    scl_pin: 0,
    sda_port: Port::D,
    sda_pin: 12,
    pin_func: gpio::function::FUNC2,
};

/// TWI0 alternative pins: PE11=SCL, PE12=SDA, func2
pub const TWI0_PINS_E: TwiPins = TwiPins {
    scl_port: Port::E,
    scl_pin: 11,
    sda_port: Port::E,
    sda_pin: 12,
    pin_func: gpio::function::FUNC2,
};

/// TWI1 default pins: PD5=SCL, PD6=SDA, func2
pub const TWI1_PINS: TwiPins = TwiPins {
    scl_port: Port::D,
    scl_pin: 5,
    sda_port: Port::D,
    sda_pin: 6,
    pin_func: gpio::function::FUNC2,
};

/// TWI2 default pins: PD15=SCL, PD16=SDA, func3
pub const TWI2_PINS: TwiPins = TwiPins {
    scl_port: Port::D,
    scl_pin: 15,
    sda_port: Port::D,
    sda_pin: 16,
    pin_func: gpio::function::FUNC3,
};

/// TWI2 alternative pins: PE0=SCL, PE1=SDA, func3
pub const TWI2_PINS_E: TwiPins = TwiPins {
    scl_port: Port::E,
    scl_pin: 0,
    sda_port: Port::E,
    sda_pin: 1,
    pin_func: gpio::function::FUNC3,
};

// ── Factory Functions ─────────────────────────────────────────────────────

/// Create and initialize TWI0 with default power-on parameters.
///
/// Uses PD0=SCL, PD12=SDA, 400 kHz on the APB bus.
pub fn twi0() -> Twi {
    let twi = unsafe { Twi::new(TWI0_BASE, BusGate::Twi0) };
    twi.init(&TWI0_PINS, clock::apb_hz(), 400_000);
    twi
}

/// Create and initialize TWI0 with alternative PE pins.
pub fn twi0_port_e() -> Twi {
    let twi = unsafe { Twi::new(TWI0_BASE, BusGate::Twi0) };
    twi.init(&TWI0_PINS_E, clock::apb_hz(), 400_000);
    twi
}

/// Create and initialize TWI1 with default power-on parameters.
///
/// Uses PD5=SCL, PD6=SDA, 400 kHz on the APB bus.
pub fn twi1() -> Twi {
    let twi = unsafe { Twi::new(TWI1_BASE, BusGate::Twi1) };
    twi.init(&TWI1_PINS, clock::apb_hz(), 400_000);
    twi
}

/// Create and initialize TWI2 with default power-on parameters.
///
/// Uses PD15=SCL, PD16=SDA, 400 kHz on the APB bus.
pub fn twi2() -> Twi {
    let twi = unsafe { Twi::new(TWI2_BASE, BusGate::Twi2) };
    twi.init(&TWI2_PINS, clock::apb_hz(), 400_000);
    twi
}

/// Create and initialize TWI2 with alternative PE pins.
pub fn twi2_port_e() -> Twi {
    let twi = unsafe { Twi::new(TWI2_BASE, BusGate::Twi2) };
    twi.init(&TWI2_PINS_E, clock::apb_hz(), 400_000);
    twi
}

// ── Probe ─────────────────────────────────────────────────────────────────

/// Probe whether a device exists at the given 7-bit address.
///
/// Sends START+ADDR+W, checks for ACK, then sends STOP.
/// Uses polling (not interrupt) since it's a simple quick check.
pub fn probe(twi: &Twi, addr: u8) -> bool {
    unsafe {
        let base = twi.base();

        twi_soft_reset(base);
        timer::delay_us(100);

        let mut timeout: u32 = 0xFF;
        while (twi_read(base, reg::SRST) & 1) != 0 && timeout > 0 {
            timeout -= 1;
            core::hint::spin_loop();
        }

        twi_enable_bus(base);
        twi_disable_ack(base);

        // START
        twi_set_start(base);
        if twi_wait_start_clear(base).is_err() {
            return false;
        }

        timeout = 0xFFFF;
        while !twi_query_irq_flag(base) && timeout > 0 {
            timeout -= 1;
            core::hint::spin_loop();
        }
        if timeout == 0 {
            return false;
        }

        let st = twi_get_status(base);
        if st != stat::TX_STA && st != stat::TX_RESTA {
            return false;
        }

        // Send address + write
        twi_send_addr(base, addr, false);

        timeout = 0xFFFF;
        while !twi_query_irq_flag(base) && timeout > 0 {
            timeout -= 1;
            core::hint::spin_loop();
        }
        if timeout == 0 {
            return false;
        }

        let st = twi_get_status(base);

        // Send STOP
        twi_set_stop(base);
        twi_clear_irq_flag(base);

        st == stat::TX_AW_ACK
    }
}
