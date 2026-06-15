//! CPU low-level operations for Allwinner F1C100s (ARM926EJ-S)
//!
//! Provides:
//! - ARM CPU mode constants
//! - CP15 coprocessor operations (cache, MMU, TLB)
//! - Interrupt enable/disable via CPSR
//! - Low-level hardware initialization (clocks, watchdog, GPIO)

use core::arch::asm;

// ── ARM CPU Mode Constants ──────────────────────────────────────────────

pub const MODE_USR: u32 = 0x10;
pub const MODE_FIQ: u32 = 0x11;
pub const MODE_IRQ: u32 = 0x12;
pub const MODE_SVC: u32 = 0x13;
pub const MODE_ABT: u32 = 0x17;
pub const MODE_UND: u32 = 0x1B;
pub const MODE_SYS: u32 = 0x1F;
pub const MODE_MASK: u32 = 0x1F;
pub const NOINT: u32 = 0xC0;
pub const I_BIT: u32 = 0x80;
pub const F_BIT: u32 = 0x40;

/// CP15 control register bits (c1, c0, 0)
pub mod cp15_ctrl {
    pub const MMU_ENABLE: u32 = 1 << 0;     // M: MMU enable
    pub const ALIGN_FAULT: u32 = 1 << 1;    // A: Alignment fault enable
    pub const DCACHE_ENABLE: u32 = 1 << 2;  // C: Data cache enable
    pub const WRITE_BUFFER: u32 = 1 << 3;   // W: Write buffer enable
    pub const BIG_ENDIAN: u32 = 1 << 7;     // B: Big-endian
    pub const SYS_PROT: u32 = 1 << 8;       // S: System protection
    pub const ROM_PROT: u32 = 1 << 9;       // R: ROM protection
    pub const BRANCH_PRED: u32 = 1 << 11;   // Z: Branch prediction
    pub const ICACHE_ENABLE: u32 = 1 << 12; // I: Instruction cache
    pub const HIGH_VECTORS: u32 = 1 << 13;  // V: High exception vectors
    pub const RR_REPLACE: u32 = 1 << 14;    // RR: Round-robin replacement
}

// ── CP15 Operations ─────────────────────────────────────────────────────

/// Read CP15 control register (c1, c0, 0)
#[inline]
pub fn cp15_read_ctrl() -> u32 {
    let val: u32;
    unsafe {
        asm!("mrc p15, 0, {0}, c1, c0, 0", out(reg) val);
    }
    val
}

/// Write CP15 control register (c1, c0, 0)
#[inline]
pub fn cp15_write_ctrl(val: u32) {
    unsafe {
        asm!("mcr p15, 0, {0}, c1, c0, 0", in(reg) val);
    }
}

/// Read CP15 Translation Table Base Register 0 (c2, c0, 0)
#[inline]
pub fn cp15_read_ttbr0() -> u32 {
    let val: u32;
    unsafe {
        asm!("mrc p15, 0, {0}, c2, c0, 0", out(reg) val);
    }
    val
}

/// Write CP15 Translation Table Base Register 0 (c2, c0, 0)
#[inline]
pub fn cp15_write_ttbr0(val: u32) {
    unsafe {
        asm!("mcr p15, 0, {0}, c2, c0, 0", in(reg) val);
    }
}

/// Read CP15 Domain Access Control Register (c3, c0, 0)
#[inline]
pub fn cp15_read_dacr() -> u32 {
    let val: u32;
    unsafe {
        asm!("mrc p15, 0, {0}, c3, c0, 0", out(reg) val);
    }
    val
}

/// Write CP15 Domain Access Control Register (c3, c0, 0)
#[inline]
pub fn cp15_write_dacr(val: u32) {
    unsafe {
        asm!("mcr p15, 0, {0}, c3, c0, 0", in(reg) val);
    }
}

// ── Cache & TLB Operations ──────────────────────────────────────────────

/// Invalidate entire I-cache
#[inline]
pub fn invalidate_icache() {
    unsafe {
        asm!("mcr p15, 0, {0}, c7, c5, 0", in(reg) 0u32);
    }
}

/// Invalidate entire D-cache
#[inline]
pub fn invalidate_dcache_all() {
    unsafe {
        asm!("mcr p15, 0, {0}, c7, c6, 0", in(reg) 0u32);
    }
}

/// Invalidate both I-cache and D-cache
#[inline]
pub fn invalidate_caches() {
    unsafe {
        asm!("mcr p15, 0, {0}, c7, c7, 0", in(reg) 0u32);
    }
}

/// Invalidate entire TLB (unified)
#[inline]
pub fn invalidate_tlb() {
    unsafe {
        asm!("mcr p15, 0, {0}, c8, c7, 0", in(reg) 0u32);
    }
}

/// Invalidate D-cache line by MVA (c7, c6, 1)
#[inline]
pub fn invalidate_dcache_mva(addr: u32) {
    unsafe {
        asm!("mcr p15, 0, {0}, c7, c6, 1", in(reg) addr);
    }
}

/// Clean D-cache line by MVA (c7, c10, 1)
#[inline]
pub fn clean_dcache_mva(addr: u32) {
    unsafe {
        asm!("mcr p15, 0, {0}, c7, c10, 1", in(reg) addr);
    }
}

/// Clean and invalidate D-cache line by MVA (c7, c14, 1)
#[inline]
pub fn clean_invalidate_dcache_mva(addr: u32) {
    unsafe {
        asm!("mcr p15, 0, {0}, c7, c14, 1", in(reg) addr);
    }
}

/// Clean and invalidate D-cache line by set/index (c7, c14, 2)
#[inline]
pub fn clean_invalidate_dcache_index(index: u32) {
    unsafe {
        asm!("mcr p15, 0, {0}, c7, c14, 2", in(reg) index);
    }
}

/// Data Synchronization Barrier
#[inline]
pub fn dsb() {
    unsafe {
        asm!("mcr p15, 0, {0}, c7, c10, 4", in(reg) 0u32);
    }
}

/// Data Memory Barrier
#[inline]
pub fn dmb() {
    unsafe {
        asm!("mcr p15, 0, {0}, c7, c10, 5", in(reg) 0u32);
    }
}

/// Instruction Synchronization Barrier
#[inline]
pub fn isb() {
    unsafe {
        asm!("mcr p15, 0, {0}, c7, c5, 4", in(reg) 0u32);
    }
}

// ── CPSR / Interrupt Control ────────────────────────────────────────────

/// Read CPSR register
#[inline]
pub fn read_cpsr() -> u32 {
    let val: u32;
    unsafe {
        asm!("mrs {0}, cpsr", out(reg) val);
    }
    val
}

/// Disable IRQ and FIQ interrupts; returns previous CPSR value
#[inline]
pub fn interrupt_disable() -> u32 {
    let cpsr: u32;
    unsafe {
        asm!(
            "mrs {0}, cpsr",
            "orr {1}, {0}, #{no_int}",
            "msr cpsr_c, {1}",
            out(reg) cpsr,
            out(reg) _,
            no_int = const NOINT,
        );
    }
    cpsr
}

/// Restore interrupt state from saved CPSR value
#[inline]
pub fn interrupt_enable(cpsr: u32) {
    unsafe {
        asm!("msr cpsr_c, {0}", in(reg) cpsr);
    }
}

/// Check if interrupts are currently enabled (IRQ not masked)
#[inline]
pub fn interrupts_enabled() -> bool {
    read_cpsr() & I_BIT == 0
}

/// Wait for interrupt (WFI)
#[inline]
pub fn wfi() {
    unsafe {
        asm!("mcr p15, 0, {0}, c7, c0, 4", in(reg) 0u32);
    }
}

// ── Low-level Hardware Initialization ───────────────────────────────────

/// Disable the hardware watchdog timer
pub fn disable_watchdog() {
    unsafe {
        let ptr = 0x01C20CB8 as *mut u32;
        ptr.write_volatile(0);
    }
}

#[inline]
unsafe fn read32(addr: u32) -> u32 { (addr as *const u32).read_volatile() }
#[inline]
unsafe fn write32(addr: u32, val: u32) { (addr as *mut u32).write_volatile(val); }

pub fn cpu_reset() -> ! {
    unsafe {
        let mut val = read32(0x01c20ca0 + 0x18); // Watchdog mode register
        val &= !(0xf << 4); // Clear existing reset mode
        val |= (1 << 4) | (0x1 << 0); // Set reset mode to "reset CPU" and enable watchdog
        write32(0x01c20ca0 + 0x18, val);
        write32(0x01c20ca0 + 0x10, (0xa57 << 1) | (1 << 0)); // Start watchdog with short timeout
        loop {
            core::hint::spin_loop();
        }
    }
}

// ── SPL Debug UART (UART2, PE7=TX / PE8=RX, 115200-8-N-1) ───────────────
//
// Raw register pokes only — no string literals, no statics: at SPL time
// only the `.text.spl` bytes loaded by the BROM exist in SRAM, so anything
// linked into .rodata/.data (DRAM addresses) must not be touched.
//
// Also NO runtime division/modulo anywhere in SPL code: ARM926 has no
// divide instruction, so `/` and `%` on runtime values compile to
// `__aeabi_uidiv` libcalls — compiler builtins live in plain `.text`,
// outside the BROM-loaded region, and the relative `bl` jumps into
// garbage when running from SRAM.

#[link_section = ".text.spl"]
unsafe fn spl_uart2_init() {
    #[inline]
    unsafe fn rmw(addr: u32, clear: u32, set: u32) {
        let p = addr as *mut u32;
        p.write_volatile((p.read_volatile() & !clear) | set);
    }
    rmw(0x01C2_0890, 0xF << 28, 0x3 << 28); // PE_CFG0: PE7 = UART2 TX (FUNC2)
    rmw(0x01C2_0894, 0xF, 0x3);             // PE_CFG1: PE8 = UART2 RX (FUNC2)
    rmw(0x01C2_0068, 0, 1 << 22);           // CCU BUS_GATE2: UART2 clock
    rmw(0x01C2_02D0, 0, 1 << 22);           // CCU BUS_SOFT_RST2: UART2 release

    let uart = 0x01C2_5800 as *mut u32;
    uart.add(1).write_volatile(0);          // IER = 0
    uart.add(2).write_volatile(0x07);       // FCR: FIFO enable, clear TX/RX
    uart.add(4).write_volatile(0);          // MCR = 0
    uart.add(3).write_volatile(0x80);       // LCR: DLAB
    // 100 MHz APB (clock::init_default) / (115200 * 16) = 54.
    // Hardcoded: apb_hz() divides at runtime → __aeabi_uidiv libcall,
    // which is outside the SPL (see module comment above).
    let div: u32 = 54;
    uart.write_volatile(div & 0xFF);        // DLL
    uart.add(1).write_volatile(div >> 8);   // DLH
    uart.add(3).write_volatile(0x03);       // LCR: 8-N-1, DLAB=0
}

/// Blocking TX of one byte on the SPL debug UART.
///
/// Flushes: waits until the TX FIFO is fully drained before returning, so
/// a checkpoint character on the wire proves the preceding step completed
/// (a crash can corrupt at most the character currently shifting out, and
/// can never silently swallow queued ones).
#[link_section = ".text.spl"]
pub unsafe fn spl_putc(c: u8) {
    let uart = 0x01C2_5800 as *mut u32;
    while uart.add(0x7C / 4).read_volatile() & 0x2 == 0 {} // USR.TX_FIFO_NOT_FULL
    uart.write_volatile(c as u32);
    while uart.add(0x7C / 4).read_volatile() & 0x4 == 0 {} // USR.TX_FIFO_EMPTY
}

/// Full low-level initialization called from assembly startup.
/// Initializes clocks, the SPL debug UART, DRAM, and GPIO defaults.
///
/// Silent on a healthy boot. On DRAM init failure it spams 'E' on UART2
/// forever — continuing would relocate into nonfunctional DRAM and die
/// without a trace. SPL exception handlers also report on this UART.
///
/// # Safety
/// This function should only be called once at system startup.
#[link_section = ".text.spl"]
#[no_mangle]
pub unsafe extern "C" fn low_level_init() {
    crate::clock::init_default();
    spl_uart2_init();

    if !crate::dram::init() {
        loop {
            spl_putc(b'E');
            for _ in 0..2_000_000 {
                core::hint::spin_loop();
            }
        }
    }

    crate::gpio::init_default();
}

// ── Copy-from-SPI ──────────────────────────────────────────────────────

// Symbols defined by linker script
extern "C" {
    static __image_start: u8;
    static __image_end: u8;
}

/// Copy the full firmware image from SPI NOR flash to DRAM.
///
/// Uses SPI0 in polling mode to read flash command `0x03` (standard read)
/// starting at offset 0, writing the entire image to `0x80000000` (DRAM base).
///
/// This is called from the SPL after clock + DRAM initialization, before
/// relocation. It must use only stack variables — no global state is valid yet.
///
/// # Safety
/// Must only be called after `clock::init_default()` + `dram::init()`.
#[link_section = ".text.spl"]
#[no_mangle]
pub unsafe extern "C" fn sys_copyself() {
    // ── SPI0 register addresses ──────────────────────────────────────
    const SPI0_BASE: u32 = 0x01C05000;
    const SPI_GCR: u32 = 0x04;   // Global Control
    const SPI_TCR: u32 = 0x08;   // Transfer Control
    const SPI_FCR: u32 = 0x18;   // FIFO Control
    const SPI_FSR: u32 = 0x1C;   // FIFO Status
    const SPI_CCR: u32 = 0x24;   // Clock Rate Control
    const SPI_MBC: u32 = 0x30;   // Master Burst Control
    const SPI_MTC: u32 = 0x34;   // Master Transmit Counter
    const SPI_BCC: u32 = 0x38;   // Burst Control
    const SPI_TXD: u32 = 0x200;  // TX Data (byte access)
    const SPI_RXD: u32 = 0x300;  // RX Data (byte access)

    let spi = SPI0_BASE;

    // ── Helper closures (capturing spi by value) ─────────────────────    
    #[inline]
    unsafe fn write8(addr: u32, val: u8) { (addr as *mut u8).write_volatile(val); }
    #[inline]
    unsafe fn read8(addr: u32) -> u8 { (addr as *const u8).read_volatile() }

    // ── 1. Configure GPIO PC0-PC3 for SPI0 function ─────────────────
    // GPIO config register: 0x01C20848 (CFG0 for port C)
    let gpio_cfg0 = 0x01C20848u32;
    for pin in 0u32..4u32 {
        let v = read32(gpio_cfg0);
        write32(gpio_cfg0, (v & !(0xF << (pin * 4))) | (2 << (pin * 4)));
    }

    // ── 2. Deassert SPI0 reset + enable clock gate ──────────────────
    // bus_soft_rst0 (0x01C202C0) bit 20 = SPI0
    let rst_addr: u32 = 0x01C202C0;
    write32(rst_addr, read32(rst_addr) | (1 << 20));
    // bus_clk_gating0 (0x01C20060) bit 20 = SPI0
    let gate_addr: u32 = 0x01C20060;
    write32(gate_addr, read32(gate_addr) | (1 << 20));

    // ── 3. Set SPI clock: DRS=1, CDR2=3 → AHB/(2*(3+1)) = 200/8 = 25 MHz
    write32(spi + SPI_CCR, (1u32 << 12) | 3u32);

    // ── 4. Enable SPI0 + soft reset + master mode ───────────────────
    let mut gcr = read32(spi + SPI_GCR);
    gcr |= (1 << 31) | (1 << 7) | (1 << 1) | (1 << 0); // RST|TP_EN|M_MASTER|EN
    write32(spi + SPI_GCR, gcr);
    while read32(spi + SPI_GCR) & (1 << 31) != 0 {}   // wait for RST clear

    // ── 5. Configure transfer: mode 0, software CS, CS idle high ────
    let mut tcr = read32(spi + SPI_TCR);
    tcr &= !(0x3 << 0);          // clear CPHA+CPOL → mode 0
    tcr |= (1 << 6) | (1 << 2);  // SS_OWNER_SW | SPOL (CS idle high)
    write32(spi + SPI_TCR, tcr);

    // ── 6. Reset FIFOs ──────────────────────────────────────────────
    write32(spi + SPI_FCR, (1 << 31) | (1 << 15)); // TF_RST | RF_RST

    // ── 7. Get image size from linker symbols ───────────────────────
    let dst: u32 = &__image_start as *const u8 as u32;
    let image_size: u32 = &__image_end as *const u8 as u32 - dst;

    // ── 8. Read entire image from flash to DRAM ────────────────────
    //
    // xboot transfer model (sys-spinor.c, known good on this silicon):
    // every burst sets MBC = MTC = BCC = n and pushes explicit 0xFF dummy
    // TX bytes for the receive phase. The "hardware dummy" mode
    // (MTC=0, BCC=n) is NOT used by xboot on this controller and never
    // completed on real hardware. CS is asserted once for a single
    // sequential read of the whole image.

    // Assert CS (SS_LEVEL=0, SS_SEL=0)
    let mut tcr_val = read32(spi + SPI_TCR);
    tcr_val &= !((0x3 << 4) | (1 << 7));
    write32(spi + SPI_TCR, tcr_val);

    // Command phase: read (0x03) from flash offset 0, full-duplex 4 bytes
    let cmd: [u8; 4] = [0x03, 0, 0, 0];
    write32(spi + SPI_MBC, 4);
    write32(spi + SPI_MTC, 4);
    write32(spi + SPI_BCC, 4);
    for i in 0..4 {
        write8(spi + SPI_TXD, cmd[i as usize]);
    }
    write32(spi + SPI_TCR, read32(spi + SPI_TCR) | (1 << 31));
    while read32(spi + SPI_TCR) & (1 << 31) != 0 {}
    while (read32(spi + SPI_FSR) & 0xFF) < 4 {}
    for _ in 0..4 {
        let _ = read8(spi + SPI_RXD); // discard RX clocked during command
    }

    // Data phase: 64-byte FIFO chunks, dummy TX bytes clock the read out
    const CHUNK: u32 = 64; // FIFO depth
    let mut off: u32 = 0;
    while off < image_size {
        let remaining = image_size - off;
        let n = if remaining < CHUNK { remaining } else { CHUNK };

        write32(spi + SPI_MBC, n);
        write32(spi + SPI_MTC, n);
        write32(spi + SPI_BCC, n);
        for _ in 0..n {
            write8(spi + SPI_TXD, 0xFF);
        }

        write32(spi + SPI_TCR, read32(spi + SPI_TCR) | (1 << 31));
        while read32(spi + SPI_TCR) & (1 << 31) != 0 {}

        while (read32(spi + SPI_FSR) & 0xFF) < n {}
        for i in 0..n {
            let byte = read8(spi + SPI_RXD);
            write8(dst + off + i, byte);
        }

        off += n;
    }

    // De-assert CS
    let mut tcr_val = read32(spi + SPI_TCR);
    tcr_val &= !((0x3 << 4) | (1 << 7));
    tcr_val |= 1 << 7; // SS_LEVEL=1 (idle)
    write32(spi + SPI_TCR, tcr_val);

    // ── 9. Disable SPI0 ────────────────────────────────────────────
    let gcr_val = read32(spi + SPI_GCR);
    write32(spi + SPI_GCR, gcr_val & !((1 << 1) | (1 << 0)));
}
