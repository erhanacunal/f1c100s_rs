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

/// Full low-level initialization called from assembly startup.
/// Initializes clocks and configures GPIO defaults.
///
/// # Safety
/// This function should only be called once at system startup.
#[link_section = ".text.spl"]
#[no_mangle]
pub unsafe extern "C" fn low_level_init() {
    crate::clock::init_default();
    crate::dram::init();
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
    unsafe fn read32(addr: u32) -> u32 { (addr as *const u32).read_volatile() }
    #[inline]
    unsafe fn write32(addr: u32, val: u32) { (addr as *mut u32).write_volatile(val); }
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
    const CHUNK: u32 = 64; // FIFO depth
    let mut flash_addr: u32 = 0;
    let mut remaining: u32 = image_size;

    while remaining > 0 {
        let n = if remaining < CHUNK { remaining } else { CHUNK };

        // Assert CS (SS_LEVEL=0, SS_SEL=0)
        let mut tcr_val = read32(spi + SPI_TCR);
        tcr_val &= !((0x3 << 4) | (1 << 7));
        write32(spi + SPI_TCR, tcr_val);

        // Send read command (0x03) + 3 address bytes
        let cmd: [u8; 4] = [
            0x03,
            ((flash_addr >> 16) & 0xFF) as u8,
            ((flash_addr >> 8) & 0xFF) as u8,
            (flash_addr & 0xFF) as u8,
        ];

        // Command phase: 4 TX bytes, RX discarded (MBC=MTC=4 → RX=MBC-MTC=0)
        write32(spi + SPI_MBC, 4);
        write32(spi + SPI_MTC, 4);
        write32(spi + SPI_BCC, 4);
        for i in 0..4 {
            write8(spi + SPI_TXD, cmd[i as usize]);
        }

        // Trigger command transfer
        write32(spi + SPI_TCR, read32(spi + SPI_TCR) | (1 << 31));
        while read32(spi + SPI_TCR) & (1 << 31) != 0 {}

        // Data phase: MTC=0 → hardware sends dummy bytes and captures n RX bytes
        write32(spi + SPI_MBC, n);
        write32(spi + SPI_MTC, 0);
        write32(spi + SPI_BCC, n);

        // Trigger data transfer
        write32(spi + SPI_TCR, read32(spi + SPI_TCR) | (1 << 31));
        while read32(spi + SPI_TCR) & (1 << 31) != 0 {}

        // Read received data bytes and write to correct DRAM address (no +4 offset)
        while (read32(spi + SPI_FSR) & 0xFF) < n {}
        for i in 0..n {
            let byte = read8(spi + SPI_RXD);
            write8(dst + flash_addr + i, byte);
        }

        // De-assert CS
        let mut tcr_val = read32(spi + SPI_TCR);
        tcr_val &= !((0x3 << 4) | (1 << 7));
        tcr_val |= 1 << 7; // SS_LEVEL=1 (idle)
        write32(spi + SPI_TCR, tcr_val);

        flash_addr += n;
        remaining -= n;
    }

    // ── 9. Disable SPI0 ────────────────────────────────────────────
    let gcr_val = read32(spi + SPI_GCR);
    write32(spi + SPI_GCR, gcr_val & !((1 << 1) | (1 << 0)));
}
