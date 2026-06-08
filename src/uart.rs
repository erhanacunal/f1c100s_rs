//! UART driver for Allwinner F1C100s
//!
//! The F1C100s provides three 16550-compatible UARTs with:
//! - 64-byte TX and RX FIFOs
//! - Programmable baud rate (via divisor latch)
//! - 5/6/7/8 data bits, 1/1.5/2 stop bits
//! - Even/odd/none parity
//! - Interrupt support (RX available, TX empty, line status)
//!
//! Base addresses (pin function = register value written to CFG bits):
//! - UART0: `0x01C25000` (PE0=RX, PE1=TX,  FUNC4 = reg value 5)
//! - UART1: `0x01C25400` (PA2=RX, PA3=TX,  FUNC4 = reg value 5)
//! - UART2: `0x01C25800` (PE8=RX, PE7=TX,  FUNC2 = reg value 3 = IO_FUN_2)

use crate::gpio;
use crate::gpio::Port;

// ── Hardware Constants ──────────────────────────────────────────────────

/// UART base addresses
pub const UART0_BASE: u32 = 0x01C25000;
pub const UART1_BASE: u32 = 0x01C25400;
pub const UART2_BASE: u32 = 0x01C25800;

/// Register offsets
mod reg {
    pub const RBR_THR_DLL: u32 = 0x00;
    pub const IER_DLH: u32 = 0x04;
    pub const IIR_FCR: u32 = 0x08;
    pub const LCR: u32 = 0x0C;
    pub const MCR: u32 = 0x10;
    pub const LSR: u32 = 0x14;
    pub const MSR: u32 = 0x18;
    pub const SCH: u32 = 0x1C;
    pub const USR: u32 = 0x7C;
    pub const HALT: u32 = 0xA4;
    pub const DBG_DLL: u32 = 0xB0;
    pub const DBG_DLH: u32 = 0xB4;
}

/// LCR register bits
pub mod lcr {
    pub const DLAB: u32 = 1 << 7;
    pub const BREAK: u32 = 1 << 6;
    pub const PARITY_EVEN: u32 = 1 << 4;
    pub const PARITY_EN: u32 = 1 << 3;
    pub const STOP_BITS: u32 = 1 << 2;
    // Bits [1:0] = word length: 00=5, 01=6, 10=7, 11=8
    pub fn word_len(bits: u32) -> u32 {
        (bits - 5) & 0x3
    }
}

/// LSR register bits
pub mod lsr {
    pub const RX_DATA_READY: u32 = 1 << 0;
    pub const OE: u32 = 1 << 1;
    pub const PE: u32 = 1 << 2;
    pub const FE: u32 = 1 << 3;
    pub const BI: u32 = 1 << 4;
    pub const TX_HOLD_EMPTY: u32 = 1 << 5;
    pub const TX_EMPTY: u32 = 1 << 6;
    pub const RX_FIFO_ERR: u32 = 1 << 7;
}

/// USR register bits (F1C100s-specific)
pub mod usr {
    pub const BUSY: u32 = 1 << 0;
    pub const TX_FIFO_NOT_FULL: u32 = 1 << 1;
    pub const TX_FIFO_EMPTY: u32 = 1 << 2;
    pub const RX_FIFO_NOT_EMPTY: u32 = 1 << 3;
    pub const RX_FIFO_FULL: u32 = 1 << 4;
}

/// IER register bits
pub mod ier {
    pub const RX_AVAILABLE: u32 = 1 << 0;
    pub const TX_EMPTY: u32 = 1 << 1;
    pub const LINE_STATUS: u32 = 1 << 2;
    pub const MODEM_STATUS: u32 = 1 << 3;
}

/// IIR register bits
pub mod iir {
    pub const PENDING: u32 = 1 << 0;
    pub const INT_ID_MASK: u32 = 0x0E;
    pub const INT_RX: u32 = 0x04;
    pub const INT_RX_TIMEOUT: u32 = 0x0C;
    pub const INT_TX: u32 = 0x02;
    pub const INT_MODEM: u32 = 0x00;
    pub const INT_LINE: u32 = 0x06;
    pub const FIFO_ENABLED: u32 = 0xC0;
}

// ── Data Format Configuration ───────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataBits {
    Bits5 = 5,
    Bits6 = 6,
    Bits7 = 7,
    Bits8 = 8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopBits {
    Stop1 = 1,
    Stop2 = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parity {
    None,
    Even,
    Odd,
}

#[derive(Debug, Clone)]
pub struct UartConfig {
    pub baud_rate: u32,
    pub data_bits: DataBits,
    pub stop_bits: StopBits,
    pub parity: Parity,
}

impl Default for UartConfig {
    #[link_section = ".text.spl"]
    fn default() -> Self {
        Self {
            baud_rate: 115200,
            data_bits: DataBits::Bits8,
            stop_bits: StopBits::Stop1,
            parity: Parity::None,
        }
    }
}

// ── UART Instance ───────────────────────────────────────────────────────

pub struct Uart {
    base: u32,
}

impl Uart {
    /// # Safety
    /// The caller must ensure the base address is valid for this CPU.
    #[link_section = ".text.spl"]
    pub const unsafe fn new(base: u32) -> Self {
        Self { base }
    }

    #[inline(always)]
    fn rd(&self, offset: u32) -> u32 {
        unsafe { ((self.base + offset) as *const u32).read_volatile() }
    }

    #[inline(always)]
    fn wr(&self, offset: u32, val: u32) {
        unsafe { ((self.base + offset) as *mut u32).write_volatile(val) }
    }

    // ── Configuration ────────────────────────────────────────────────

    #[link_section = ".text.spl"]
    pub fn configure(&self, config: &UartConfig) {
        // Disable interrupts
        self.wr(reg::IER_DLH, 0x00);
        // Enable FIFO, clear TX/RX
        self.wr(reg::IIR_FCR, 0x07);
        // No hardware flow control
        self.wr(reg::MCR, 0x00);

        // Set DLAB=1 to access divisor latches
        self.wr(reg::LCR, lcr::DLAB);

        // Divisor = APB_CLK / (baud * 16)
        let divisor = crate::clock::apb_hz() / (config.baud_rate * 16);
        self.wr(reg::RBR_THR_DLL, divisor & 0xFF);
        self.wr(reg::IER_DLH, (divisor >> 8) & 0xFF);

        // Clear DLAB, then program word format
        let mut lcr_val: u32 = lcr::word_len(config.data_bits as u32);

        if config.stop_bits == StopBits::Stop2 {
            lcr_val |= lcr::STOP_BITS;
        }
        match config.parity {
            Parity::None => {}
            Parity::Even => lcr_val |= lcr::PARITY_EN | lcr::PARITY_EVEN,
            Parity::Odd => lcr_val |= lcr::PARITY_EN,
        }
        // Write LCR with DLAB=0 and final word format in one step
        self.wr(reg::LCR, lcr_val);
    }

    pub fn enable_rx_interrupt(&self) {
        let val = self.rd(reg::IER_DLH);
        self.wr(reg::IER_DLH, val | ier::RX_AVAILABLE);
    }

    pub fn disable_interrupts(&self) {
        self.wr(reg::IER_DLH, 0x00);
    }

    // ── Polling TX ───────────────────────────────────────────────────

    #[link_section = ".text.spl"]
    pub fn tx_ready(&self) -> bool {
        self.rd(reg::USR) & usr::TX_FIFO_NOT_FULL != 0
    }

    #[link_section = ".text.spl"]
    pub fn write_byte(&self, byte: u8) {
        while !self.tx_ready() {}
        self.wr(reg::RBR_THR_DLL, byte as u32);
    }

    #[link_section = ".text.spl"]
    pub fn write(&self, data: &[u8]) {
        for &byte in data {
            self.write_byte(byte);
        }
    }

    #[link_section = ".text.spl"]
    pub fn write_str(&self, s: &str) {
        self.write(s.as_bytes());
    }

    // ── Polling RX ───────────────────────────────────────────────────

    pub fn rx_ready(&self) -> bool {
        self.rd(reg::LSR) & lsr::RX_DATA_READY != 0
    }

    pub fn read_byte(&self) -> Option<u8> {
        if self.rx_ready() {
            Some(self.rd(reg::RBR_THR_DLL) as u8)
        } else {
            None
        }
    }

    pub fn read_byte_blocking(&self) -> u8 {
        while !self.rx_ready() {}
        self.rd(reg::RBR_THR_DLL) as u8
    }

    pub fn line_errors(&self) -> LineErrors {
        let lsr_val = self.rd(reg::LSR);
        LineErrors {
            overrun: lsr_val & lsr::OE != 0,
            parity: lsr_val & lsr::PE != 0,
            framing: lsr_val & lsr::FE != 0,
            break_int: lsr_val & lsr::BI != 0,
        }
    }

    pub fn interrupt_status(&self) -> u32 {
        self.rd(reg::IIR_FCR) & 0x0F
    }
}

/// Line error flags from LSR
#[derive(Debug, Default)]
pub struct LineErrors {
    pub overrun: bool,
    pub parity: bool,
    pub framing: bool,
    pub break_int: bool,
}

// ── Pre-defined UART instances ──────────────────────────────────────────

pub fn uart0() -> Uart {
    unsafe { Uart::new(UART0_BASE) }
}

pub fn uart1() -> Uart {
    unsafe { Uart::new(UART1_BASE) }
}

#[link_section = ".text.spl"]
pub fn uart2() -> Uart {
    unsafe { Uart::new(UART2_BASE) }
}

// ── Convenience init functions ──────────────────────────────────────────

/// Initialise UART0: PE0=RX, PE1=TX (FUNC4), 115200-8-N-1
pub fn init_uart0() -> Uart {
    crate::clock::bus_clk_init(crate::clock::BusGate::Uart0);
    gpio::set_function(Port::E, 0, gpio::function::FUNC4);
    gpio::set_function(Port::E, 1, gpio::function::FUNC4);
    let uart = uart0();
    uart.configure(&UartConfig::default());
    uart
}

/// Initialise UART1: PA2=RX, PA3=TX (FUNC4), 115200-8-N-1
pub fn init_uart1() -> Uart {
    crate::clock::bus_clk_init(crate::clock::BusGate::Uart1);
    gpio::set_function(Port::A, 2, gpio::function::FUNC4);
    gpio::set_function(Port::A, 3, gpio::function::FUNC4);
    let uart = uart1();
    uart.configure(&UartConfig::default());
    uart
}

/// Initialise UART2: PE8=RX, PE7=TX (FUNC2 = reg value 3), 115200-8-N-1
#[link_section = ".text.spl"]
pub fn init_uart2() -> Uart {
    gpio::set_function(Port::E, 8, gpio::function::FUNC2); // PE8 = UART2_RX
    gpio::set_function(Port::E, 7, gpio::function::FUNC2); // PE7 = UART2_TX
    crate::clock::bus_clk_init(crate::clock::BusGate::Uart2);
    let uart = uart2();
    uart.configure(&UartConfig::default());
    uart
}
