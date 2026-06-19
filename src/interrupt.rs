//! Interrupt Controller (INTC) for Allwinner F1C100s
//!
//! The F1C100s INTC supports up to 64 interrupt sources, organized
//! in two groups of 32. It provides:
//! - IRQ/FIQ routing per interrupt
//! - Priority-based interrupt selection
//! - Mask, enable, pending registers

// ── Interrupt Numbers ───────────────────────────────────────────────────

/// Total number of interrupt sources
pub const INTERRUPTS_MAX: usize = 64;

/// Number of interrupts per group
pub const GROUP_NUM: usize = 32;

// Interrupt source identifiers
pub const NMI_INTERRUPT: u32 = 0;
pub const UART0_INTERRUPT: u32 = 1;
pub const UART1_INTERRUPT: u32 = 2;
pub const UART2_INTERRUPT: u32 = 3;
pub const OWA_INTERRUPT: u32 = 5;
pub const CIR_INTERRUPT: u32 = 6;
pub const TWI0_INTERRUPT: u32 = 7;
pub const TWI1_INTERRUPT: u32 = 8;
pub const TWI2_INTERRUPT: u32 = 9;
pub const SPI0_INTERRUPT: u32 = 10;
pub const SPI1_INTERRUPT: u32 = 11;
pub const TIMER0_INTERRUPT: u32 = 13;
pub const TIMER1_INTERRUPT: u32 = 14;
pub const TIMER2_INTERRUPT: u32 = 15;
pub const WATCHDOG_INTERRUPT: u32 = 16;
pub const RSB_INTERRUPT: u32 = 17;
pub const DMA_INTERRUPT: u32 = 18;
pub const TOUCHPANEL_INTERRUPT: u32 = 20;
pub const AUDIOCODEC_INTERRUPT: u32 = 21;
pub const KEYADC_INTERRUPT: u32 = 22;
pub const SDC0_INTERRUPT: u32 = 23;
pub const SDC1_INTERRUPT: u32 = 24;
pub const USB_OTG_INTERRUPT: u32 = 26;
pub const TVD_INTERRUPT: u32 = 27;
pub const TVE_INTERRUPT: u32 = 28;
pub const TCON_INTERRUPT: u32 = 29;
pub const DE_FE_INTERRUPT: u32 = 30;
pub const DE_BE_INTERRUPT: u32 = 31;
pub const CSI_INTERRUPT: u32 = 32;
pub const DE_INTERLACER_INTERRUPT: u32 = 33;
pub const VE_INTERRUPT: u32 = 34;
pub const DAUDIO_INTERRUPT: u32 = 35;
pub const PIOD_INTERRUPT: u32 = 38;
pub const PIOE_INTERRUPT: u32 = 39;
pub const PIOF_INTERRUPT: u32 = 40;

// ── INTC Register Offsets ───────────────────────────────────────────────

const INTC_BASE: u32 = 0x01C20400;

mod reg {
    pub const VECTOR:    u32 = 0x00;
    pub const BASE_ADDR: u32 = 0x04;
    pub const PEND0:     u32 = 0x10;
    pub const PEND1:     u32 = 0x14;
    pub const EN0:       u32 = 0x20;
    pub const EN1:       u32 = 0x24;
    pub const MASK0:     u32 = 0x30;
    pub const MASK1:     u32 = 0x34;
    pub const RESP0:     u32 = 0x40;
    pub const RESP1:     u32 = 0x44;
    pub const FF0:       u32 = 0x50;
    pub const FF1:       u32 = 0x54;
}

#[inline(always)]
unsafe fn rd(offset: u32) -> u32 {
    ((INTC_BASE + offset) as *const u32).read_volatile()
}

#[inline(always)]
unsafe fn wr(offset: u32, val: u32) {
    ((INTC_BASE + offset) as *mut u32).write_volatile(val)
}

fn group_regs(group: usize) -> (u32, u32, u32) {
    if group == 0 { (reg::PEND0, reg::EN0, reg::MASK0) }
    else          { (reg::PEND1, reg::EN1, reg::MASK1) }
}

// ── ISR Handler Type ────────────────────────────────────────────────────

/// Interrupt service routine signature
pub type IsrHandler = fn(vector: u32);

/// An entry in the interrupt handler table
#[derive(Clone)]
struct IsrEntry {
    handler: IsrHandler,
}

impl Default for IsrEntry {
    fn default() -> Self {
        Self {
            handler: default_handler,
        }
    }
}

// ── Global State ────────────────────────────────────────────────────────

/// Interrupt nesting counter
static mut INTERRUPT_NEST: u32 = 0;

/// Whether we are currently in an interrupt context
static mut IN_ISR: bool = false;

// ── ISR Table ───────────────────────────────────────────────────────────

/// Static interrupt handler table
static mut ISR_TABLE: [IsrEntry; INTERRUPTS_MAX] = {
    // Initialize all entries with the default handler
    const ENTRY: IsrEntry = IsrEntry {
        handler: default_handler,
    };
    [ENTRY; INTERRUPTS_MAX]
};

/// Default unhandled interrupt handler
fn default_handler(vector: u32) {
    // Unhandled interrupt - in a real system you might log or panic here
    let _ = vector;
}

// ── Helper: vector-to-group/bit ─────────────────────────────────────────

#[inline] fn vector_group(vector: u32) -> usize { if vector >= 32 { 1 } else { 0 } }
#[inline] fn vector_bit(vector: u32) -> u32 { vector & 0x1F }

// ── Public API ──────────────────────────────────────────────────────────

/// Initialize the interrupt controller: disable and mask all sources.
pub fn init() {
    unsafe {
        wr(reg::BASE_ADDR, 0);
        wr(reg::EN0, 0);   wr(reg::EN1, 0);
        wr(reg::MASK0, 0xFFFF_FFFF); wr(reg::MASK1, 0xFFFF_FFFF);
        wr(reg::PEND0, 0); wr(reg::PEND1, 0);
        wr(reg::RESP0, 0); wr(reg::RESP1, 0);
        wr(reg::FF0,   0); wr(reg::FF1,   0);
        INTERRUPT_NEST = 0;
    }
}

/// Mask (disable) a specific interrupt source.
pub fn mask(vector: u32) {
    if vector >= INTERRUPTS_MAX as u32 { return; }
    let (_, _, mask_reg) = group_regs(vector_group(vector));
    let b = vector_bit(vector);
    unsafe { let v = rd(mask_reg); wr(mask_reg, v | (1 << b)); }
}

/// Unmask (enable) a specific interrupt source.
pub fn unmask(vector: u32) {
    if vector >= INTERRUPTS_MAX as u32 { return; }
    let (_, _, mask_reg) = group_regs(vector_group(vector));
    let b = vector_bit(vector);
    unsafe { let v = rd(mask_reg); wr(mask_reg, v & !(1 << b)); }
}

/// Install an interrupt handler and enable the interrupt source.
pub fn install(vector: u32, handler: IsrHandler) -> Option<IsrHandler> {
    if vector >= INTERRUPTS_MAX as u32 { return None; }
    let (pend_reg, en_reg, _) = group_regs(vector_group(vector));
    let b = vector_bit(vector);
    unsafe {
        let old = ISR_TABLE[vector as usize].handler;
        ISR_TABLE[vector as usize].handler = handler;
        let p = rd(pend_reg); wr(pend_reg, p & !(1 << b));
        let e = rd(en_reg);   wr(en_reg,   e | (1 << b));
        Some(old)
    }
}

/// Dispatch the current pending interrupt (called from IRQ exception handler).
pub fn dispatch() {
    let vector = unsafe {
        let v = rd(reg::VECTOR);
        let b = rd(reg::BASE_ADDR);
        (v - b) >> 2
    };
    unsafe { INTERRUPT_NEST += 1; IN_ISR = true; }
    if (vector as usize) < INTERRUPTS_MAX {
        let handler = unsafe { ISR_TABLE[vector as usize].handler };
        handler(vector);
        let (pend_reg, _, _) = group_regs(vector_group(vector));
        let b = vector_bit(vector);
        unsafe { let p = rd(pend_reg); wr(pend_reg, p & !(1 << b)); }
    }
    unsafe { IN_ISR = false; INTERRUPT_NEST -= 1; }
}

/// Returns true if we are currently servicing an interrupt
pub fn in_isr() -> bool {
    unsafe { IN_ISR }
}

/// Get the interrupt nesting level
pub fn nest_level() -> u32 {
    unsafe { INTERRUPT_NEST }
}

// ── Trap Handlers (called from assembly) ────────────────────────────────

/// Register state saved by exception handlers
#[repr(C)]
#[derive(Debug)]
pub struct TrapFrame {
    pub r0: u32,
    pub r1: u32,
    pub r2: u32,
    pub r3: u32,
    pub r4: u32,
    pub r5: u32,
    pub r6: u32,
    pub r7: u32,
    pub r8: u32,
    pub r9: u32,
    pub r10: u32,
    pub r11: u32,
    pub r12: u32,
    pub sp: u32,
    pub lr: u32,
    pub pc: u32,
    pub cpsr: u32,
}

/// IRQ trap handler - dispatches the interrupt
#[no_mangle]
pub unsafe extern "C" fn trap_irq() {
    dispatch();
}

/// FIQ trap handler - dispatches the interrupt
#[no_mangle]
pub unsafe extern "C" fn trap_fiq() {
    dispatch();
}

// ── DIAGNOSTIC: fault reporting over UART2 ───────────────────────────────────
// The CPU exception handlers below would otherwise spin silently, making a
// data abort / bad pointer indistinguishable from a deadlock. These helpers
// emit the fault type + faulting PC/LR + CP15 Fault Address Register on UART2
// (raw MMIO, since the UART abstraction is unavailable in abort context) so a
// memory fault is visible and locatable. Remove once the fault is fixed.
unsafe fn fault_putc(b: u8) {
    const UART2_THR: u32 = 0x01C2_5800;
    const UART2_USR: u32 = 0x01C2_5800 + 0x7C;
    while core::ptr::read_volatile(UART2_USR as *const u32) & (1 << 1) == 0 {}
    core::ptr::write_volatile(UART2_THR as *mut u32, b as u32);
}

unsafe fn fault_puthex(v: u32) {
    let hex = b"0123456789ABCDEF";
    let mut shift: i32 = 28;
    while shift >= 0 {
        fault_putc(hex[((v >> shift) & 0xF) as usize]);
        shift -= 4;
    }
}

unsafe fn fault_report(tag: u8, frame: &TrapFrame) -> ! {
    // CP15 Fault Address Register (c6) — the address a data abort tried to touch.
    let far: u32;
    core::arch::asm!("mrc p15, 0, {0}, c6, c0, 0", out(reg) far);
    loop {
        fault_putc(b'\r');
        fault_putc(b'\n');
        fault_putc(b'!');
        fault_putc(tag); // 'D' data abort, 'P' prefetch abort, 'U' undefined
        fault_putc(b' ');
        fault_putc(b'p');
        fault_putc(b'c');
        fault_putc(b'=');
        fault_puthex(frame.pc);
        fault_putc(b' ');
        fault_putc(b'l');
        fault_putc(b'r');
        fault_putc(b'=');
        fault_puthex(frame.lr);
        fault_putc(b' ');
        fault_putc(b'f');
        fault_putc(b'a');
        fault_putc(b'r');
        fault_putc(b'=');
        fault_puthex(far);
        // Crude delay so the line is readable and does not flood.
        for _ in 0..4_000_000 {
            core::hint::spin_loop();
        }
    }
}

/// Undefined instruction trap
#[no_mangle]
pub unsafe extern "C" fn trap_undef(frame: &TrapFrame) {
    fault_report(b'U', frame)
}

/// Software interrupt (SWI/SVC) trap
#[no_mangle]
pub unsafe extern "C" fn trap_swi(frame: &TrapFrame) {
    let _ = frame;
    loop {
        core::arch::asm!("mcr p15, 0, {0}, c7, c0, 4", in(reg) 0u32);
    }
}

/// Prefetch abort trap
#[no_mangle]
pub unsafe extern "C" fn trap_pabt(frame: &TrapFrame) {
    fault_report(b'P', frame)
}

/// Data abort trap
#[no_mangle]
pub unsafe extern "C" fn trap_dabt(frame: &TrapFrame) {
    fault_report(b'D', frame)
}

/// Reserved exception trap
#[no_mangle]
pub unsafe extern "C" fn trap_resv(frame: &TrapFrame) {
    let _ = frame;
    loop {
        core::arch::asm!("mcr p15, 0, {0}, c7, c0, 4", in(reg) 0u32);
    }
}
