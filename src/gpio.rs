//! GPIO (General Purpose I/O) driver for Allwinner F1C100s
//!
//! The F1C100s has 6 GPIO ports (A–F), each with up to 28 pins.
//! Ports D, E, F support edge/level interrupt detection.
//!
//! Each port's register block is 0x24 bytes, starting at `0x01C20800`.
//! Interrupt configuration registers for D/E/F are at `0x01C20A00`.

// ── Port / Pin / Function Enums ────────────────────────────────────────

/// GPIO port identifier
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Port {
    A = 0,
    B = 1,
    C = 2,
    D = 3,
    E = 4,
    F = 5,
}

/// Number of GPIO ports
pub const PORT_COUNT: usize = 6;

/// GPIO pin number (0–27)
pub type Pin = u8;

/// Maximum pin count per port
pub const PINS_PER_PORT: u8 = 28;

/// Pin function multiplexing values
pub mod function {
    pub const INPUT: u8 = 0x00;
    pub const OUTPUT: u8 = 0x01;
    pub const FUNC1: u8 = 0x02;
    pub const FUNC2: u8 = 0x03;
    pub const FUNC3: u8 = 0x04;
    pub const FUNC4: u8 = 0x05;
    pub const FUNC5: u8 = 0x06;
    pub const DISABLE: u8 = 0x07;
}

/// Pull-up/down mode
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PullMode {
    Disable = 0,
    Up = 1,
    Down = 2,
}

/// Drive strength level
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveLevel {
    Level0 = 0,
    Level1 = 1,
    Level2 = 2,
    Level3 = 3,
}

/// Interrupt trigger type
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IrqType {
    PositiveEdge = 0,
    NegativeEdge = 1,
    HighLevel = 2,
    LowLevel = 3,
    DoubleEdge = 4,
}

/// Interrupt debounce clock source
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IrqClock {
    Losc32kHz = 0,
    Hosc24MHz = 1,
}

/// Debounce prescaler
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DebouncePrescaler {
    Div1 = 0,
    Div2 = 1,
    Div4 = 2,
    Div8 = 3,
    Div16 = 4,
    Div32 = 5,
    Div64 = 6,
    Div128 = 7,
}

// ── Hardware Base Addresses ─────────────────────────────────────────────

const GPIO_BASE: u32 = 0x01C20800;
const GPIO_INT_BASE: u32 = GPIO_BASE + 0x200;

// Port register offsets (relative to port base)
mod port_off {
    pub const CFG: u32 = 0x00; // CFG0–3: +0, +4, +8, +C
    pub const DATA: u32 = 0x10;
    pub const DRV: u32 = 0x14; // DRV0–1: +0, +4
    pub const PUL: u32 = 0x1C; // PUL0–1: +0, +4
}

// Interrupt register offsets (relative to port int base)
mod int_off {
    pub const CFG: u32 = 0x00; // CFG0–3
    pub const CTRL: u32 = 0x10;
    pub const STA: u32 = 0x14;
    pub const DEB: u32 = 0x18;
}

// ── Volatile Helpers ────────────────────────────────────────────────────

#[inline(always)]
unsafe fn rd(addr: u32) -> u32 {
    (addr as *const u32).read_volatile()
}

#[inline(always)]
unsafe fn wr(addr: u32, val: u32) {
    (addr as *mut u32).write_volatile(val)
}

#[inline(always)]
fn port_base(port: Port) -> u32 {
    GPIO_BASE + (port as u32) * 0x24
}

fn int_base(port: Port) -> u32 {
    let idx = match port {
        Port::D => 0,
        Port::E => 1,
        Port::F => 2,
        _ => panic!("port does not support interrupts"),
    };
    GPIO_INT_BASE + idx * 0x20
}

// ── Pin Function / Direction ────────────────────────────────────────────

/// Set the function multiplex for a pin
#[link_section = ".text.spl"]
pub fn set_function(port: Port, pin: Pin, func: u8) {
    assert!((port as u8) < PORT_COUNT as u8);
    assert!(pin < PINS_PER_PORT);

    let cfg_addr = port_base(port) + port_off::CFG + ((pin / 8) as u32) * 4;
    let offset = (pin % 8) * 4;

    unsafe {
        let val = rd(cfg_addr);
        wr(cfg_addr, (val & !(0x7 << offset)) | ((func as u32) << offset));
    }
}

/// Set pin direction to input
pub fn direction_input(port: Port, pin: Pin) {
    set_function(port, pin, function::INPUT);
}

/// Set pin direction to output with initial value
pub fn direction_output(port: Port, pin: Pin, value: bool) {
    set_value(port, pin, value);
    set_function(port, pin, function::OUTPUT);
}

// ── Pin Value ───────────────────────────────────────────────────────────

/// Read a pin's logic level
pub fn get_value(port: Port, pin: Pin) -> bool {
    assert!((port as u8) < PORT_COUNT as u8);
    assert!(pin < PINS_PER_PORT);

    let data_addr = port_base(port) + port_off::DATA;
    unsafe { (rd(data_addr) >> pin) & 0x01 != 0 }
}

/// Set a pin's output value
pub fn set_value(port: Port, pin: Pin, value: bool) {
    assert!((port as u8) < PORT_COUNT as u8);
    assert!(pin < PINS_PER_PORT);

    let data_addr = port_base(port) + port_off::DATA;
    unsafe {
        let val = rd(data_addr);
        wr(data_addr, (val & !(1 << pin)) | ((value as u32) << pin));
    }
}

/// Toggle a pin's output value; returns the new value
pub fn toggle(port: Port, pin: Pin) -> bool {
    let current = get_value(port, pin);
    set_value(port, pin, !current);
    !current
}

// ── Pull Mode ───────────────────────────────────────────────────────────

/// Set pull-up/pull-down mode for a pin
pub fn set_pull_mode(port: Port, pin: Pin, mode: PullMode) {
    assert!((port as u8) < PORT_COUNT as u8);
    assert!(pin < PINS_PER_PORT);

    let pul_idx = if pin > 15 { 1u32 } else { 0u32 };
    let pul_addr = port_base(port) + port_off::PUL + pul_idx * 4;
    let offset = ((pin & 0xF) * 2) as u32;

    unsafe {
        let val = rd(pul_addr);
        wr(pul_addr, (val & !(0x3 << offset)) | ((mode as u32) << offset));
    }
}

// ── Drive Level ─────────────────────────────────────────────────────────

/// Set drive strength for a pin
pub fn set_drive_level(port: Port, pin: Pin, level: DriveLevel) {
    assert!((port as u8) < PORT_COUNT as u8);
    assert!(pin < PINS_PER_PORT);

    let drv_idx = if pin > 15 { 1u32 } else { 0u32 };
    let drv_addr = port_base(port) + port_off::DRV + drv_idx * 4;
    let offset = ((pin & 0xF) * 2) as u32;

    unsafe {
        let val = rd(drv_addr);
        wr(drv_addr, (val & !(0x3 << offset)) | ((level as u32) << offset));
    }
}

// ── Interrupt Control (Ports D, E, F only) ─────────────────────────────

fn port_supports_irq(port: Port) -> bool {
    matches!(port, Port::D | Port::E | Port::F)
}

/// Set the interrupt trigger type for a pin
pub fn set_irq_type(port: Port, pin: Pin, irq_type: IrqType) {
    assert!(port_supports_irq(port), "Interrupts only supported on ports D, E, F");
    assert!(pin < PINS_PER_PORT);

    let cfg_addr = int_base(port) + int_off::CFG + ((pin / 8) as u32) * 4;
    let offset = (pin % 8) * 4;

    unsafe {
        let val = rd(cfg_addr);
        wr(cfg_addr, (val & !(0x7 << offset)) | ((irq_type as u32) << offset));
    }
}

/// Enable interrupt for a pin
pub fn irq_enable(port: Port, pin: Pin) {
    assert!(port_supports_irq(port));

    let ctrl_addr = int_base(port) + int_off::CTRL;
    unsafe {
        let val = rd(ctrl_addr);
        wr(ctrl_addr, val | (1 << pin));
    }
}

/// Disable and acknowledge interrupt for a pin
pub fn irq_disable(port: Port, pin: Pin) {
    assert!(port_supports_irq(port));

    let base = int_base(port);
    unsafe {
        // Acknowledge first (write 1 to clear)
        let sta = rd(base + int_off::STA);
        wr(base + int_off::STA, sta | (1 << pin));
        // Disable
        let ctrl = rd(base + int_off::CTRL);
        wr(base + int_off::CTRL, ctrl & !(1 << pin));
    }
}

/// Select debounce clock source
pub fn select_irq_clock(port: Port, clock: IrqClock) {
    assert!(port_supports_irq(port));

    let deb_addr = int_base(port) + int_off::DEB;
    unsafe {
        let val = rd(deb_addr);
        wr(deb_addr, (val & !0x01) | (clock as u32));
    }
}

/// Set debounce prescaler value
pub fn set_debounce(port: Port, prescaler: DebouncePrescaler) {
    assert!(port_supports_irq(port));

    let deb_addr = int_base(port) + int_off::DEB;
    unsafe {
        let val = rd(deb_addr);
        wr(deb_addr, (val & !(0x07 << 4)) | ((prescaler as u32) << 4));
    }
}

/// Return the interrupt status register (which pins have pending interrupts)
pub fn irq_pending(port: Port) -> u32 {
    assert!(port_supports_irq(port));
    unsafe { rd(int_base(port) + int_off::STA) }
}

/// Return which pins have interrupts enabled
pub fn irq_enabled_mask(port: Port) -> u32 {
    assert!(port_supports_irq(port));
    unsafe { rd(int_base(port) + int_off::CTRL) }
}

/// Acknowledge (clear) the interrupt status for a single pin
pub fn irq_ack(port: Port, pin: Pin) {
    assert!(port_supports_irq(port));

    let sta_addr = int_base(port) + int_off::STA;
    unsafe {
        let val = rd(sta_addr);
        wr(sta_addr, val | (1 << pin));
    }
}

// ── Initialisation ──────────────────────────────────────────────────────

/// Set all GPIO pins to disabled state (function 7) except port B
#[link_section = ".text.spl"]
pub fn init_default() {
    for port in [Port::A, Port::B, Port::C, Port::D, Port::E, Port::F] {
        if port == Port::B {
            continue;
        }
        let base = port_base(port);
        unsafe {
            wr(base + port_off::CFG + 0x00, 0x7777_7777);
            wr(base + port_off::CFG + 0x04, 0x7777_7777);
            wr(base + port_off::CFG + 0x08, 0x7777_7777);
            wr(base + port_off::CFG + 0x0C, 0x7777_7777);
        }
    }
}
