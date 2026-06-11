//! DRAM (DDR SDRAM) controller driver for Allwinner F1C100s
//!
//! Initializes the DDR memory controller with auto-detection of:
//! - Memory type (DDR vs SDR)
//! - Row/column address width
//! - Total memory size (16/32/64 MB)
//!
//! Based on the xboot-style dram.c from f1c_nonos.
//!
//! ## Memory Layout
//! DRAM base: `0x80000000`
//! Typical size: 32 MB (Lichee Pi Nano with embedded DDR)
//!
//! ## Timing
//! PLL_DDR: 312 MHz (DDR clock = 156 MHz)
//! All timing parameters are in clock cycles.

use crate::clock;

// ── Base Addresses ────────────────────────────────────────────────────────

const DRAM_BASE: u32 = 0x01C01000;
const GPIO_BASE: u32 = 0x01C20800;

// ── Register Offsets ──────────────────────────────────────────────────────

#[allow(dead_code)]
mod reg {
    pub const SCONR: u32 = 0x00;    // SDRAM configuration
    pub const STMG0R: u32 = 0x04;   // SDRAM timing 0
    pub const STMG1R: u32 = 0x08;   // SDRAM timing 1
    pub const SCTLR: u32 = 0x0C;    // SDRAM control
    pub const SREFR: u32 = 0x10;    // SDRAM refresh
    pub const SEXTMR: u32 = 0x14;   // SDRAM extended mode
    pub const DDLYR: u32 = 0x24;    // DDR delay line
    pub const DADRR: u32 = 0x28;    // DDR address
    pub const DVALR: u32 = 0x2C;    // DDR valid
    pub const DRPTR0: u32 = 0x30;   // DDR read pipe 0
    pub const DRPTR1: u32 = 0x34;   // DDR read pipe 1
    pub const DRPTR2: u32 = 0x38;   // DDR read pipe 2
    pub const DRPTR3: u32 = 0x3C;   // DDR read pipe 3
    pub const SEFR: u32 = 0x40;     // SDRAM extended function
    pub const MAE: u32 = 0x44;      // SDRAM master access enable
    pub const ASPR: u32 = 0x48;     // SDRAM auto-precharge
    pub const SDLY0: u32 = 0x4C;    // SDRAM delay 0
    pub const SDLY1: u32 = 0x50;    // SDRAM delay 1
    pub const SDLY2: u32 = 0x54;    // SDRAM delay 2
    // Mode registers (DDR PHY configuration)
    pub const MCR0: u32 = 0x100;
    pub const MCR1: u32 = 0x104;
    pub const MCR2: u32 = 0x108;
    pub const MCR3: u32 = 0x10C;
    pub const MCR4: u32 = 0x110;
    pub const MCR5: u32 = 0x114;
    pub const MCR6: u32 = 0x118;
    pub const MCR7: u32 = 0x11C;
    pub const MCR8: u32 = 0x120;
    pub const MCR9: u32 = 0x124;
    pub const MCR10: u32 = 0x128;
    pub const MCR11: u32 = 0x12C;
    pub const BWCR: u32 = 0x140;   // Bandwidth control
}

// ── Memory Types ──────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum DramType {
    Sdr = 0,
    Ddr = 1,
    Mddr = 2,
}

// ── DRAM Parameters ───────────────────────────────────────────────────────

/// Default F1C100s DRAM configuration
const PLL_DDR_CLK: u32 = 156_000_000;
const DDR_CLK_MHZ: u32 = PLL_DDR_CLK / 1_000_000;

/// DRAM base address
const DRAM_MEM_BASE: u32 = 0x80000000;

struct DramParams {
    clk_mhz: u32,       // DRAM clock in MHz
    access_mode: u32,   // 0 = interleave, 1 = sequential
    cs_num: u32,        // 1 or 2 chips
    ddr8_remap: u32,    // 8-bit DDR remap flag
    dram_type: DramType,
    bus_width: u32,     // data bus width (8 or 16)
    col_width: u32,     // column address width (9 or 10)
    row_width: u32,     // row address width (12 or 13)
    bank_size: u32,     // number of banks (4)
    size_mb: u32,       // detected size in MB
}

impl Default for DramParams {
    fn default() -> Self {
        Self {
            clk_mhz: DDR_CLK_MHZ,
            access_mode: 1,
            cs_num: 1,
            ddr8_remap: 0,
            dram_type: DramType::Ddr,
            bus_width: 16,
            col_width: 10,
            row_width: 13,
            bank_size: 4,
            size_mb: 32,
        }
    }
}

// ── Timing Parameters ─────────────────────────────────────────────────────

/// Timing values in DRAM clock cycles
const T_CAS: u32 = 2;
const T_RAS: u32 = 8;
const T_RCD: u32 = 3;
const T_RP: u32 = 3;
const T_WR: u32 = 3;
const T_RFC: u32 = 13;
const T_XSR: u32 = 249;
const T_RC: u32 = 11;
const T_INIT: u32 = 8;
const T_INIT_REF: u32 = 7;
const T_WTR: u32 = 2;
const T_RRD: u32 = 2;
const T_XP: u32 = 0;

// ── MMIO Helpers ──────────────────────────────────────────────────────────

#[link_section = ".text.spl"]
#[inline]
unsafe fn dram_read(offset: u32) -> u32 {
    ((DRAM_BASE + offset) as *const u32).read_volatile()
}

#[link_section = ".text.spl"]
#[inline]
unsafe fn dram_write(offset: u32, val: u32) {
    ((DRAM_BASE + offset) as *mut u32).write_volatile(val);
}

/// Busy-wait delay approximating `ms` milliseconds
#[link_section = ".text.spl"]
fn dram_delay_ms(ms: u32) {
    // Approximate: ~2M loop iterations per ms on 408 MHz ARM9
    for _ in 0..(ms * 2000) {
        core::hint::spin_loop();
    }
}

// ── DRAM Initialization Steps ─────────────────────────────────────────────

/// Step 1: Perform initial DRAM controller initialization.
/// Sets SCTLR bit 0 and waits for completion.
#[link_section = ".text.spl"]
unsafe fn dram_initial() -> bool {
    let mut timeout: u32 = 0xFFFFFF;

    dram_write(reg::SCTLR, dram_read(reg::SCTLR) | 0x1);
    while (dram_read(reg::SCTLR) & 0x1) != 0 && timeout > 0 {
        timeout -= 1;
    }
    timeout > 0
}

/// Step 2: Trigger delay line scan and wait.
#[link_section = ".text.spl"]
unsafe fn dram_delay_scan() -> bool {
    let mut timeout: u32 = 0xFFFFFF;

    dram_write(reg::DDLYR, dram_read(reg::DDLYR) | 0x1);
    while (dram_read(reg::DDLYR) & 0x1) != 0 && timeout > 0 {
        timeout -= 1;
    }
    timeout > 0
}

/// Calculate and set auto-refresh cycle register.
///
/// `clk_hz` — DRAM clock in **Hz** (xboot passes `para->clk * 1000000`).
/// The internal constants (`10_000_000 >> 6` etc.) are Hz-scale; passing
/// kHz here computes SREFR = 1 instead of ~1216 and the resulting refresh
/// storm starves all DRAM accesses (bus hang on first memory read).
#[link_section = ".text.spl"]
unsafe fn dram_set_autofresh_cycle(clk_hz: u32) {
    let row = (dram_read(reg::SCONR) >> 5) & 0xF;
    let mut val: u32 = 0;

    if row == 0xC {
        // 13-row refresh: 7.8 µs period
        if clk_hz >= 1_000_000 {
            let mut temp = clk_hz + (clk_hz >> 3) + (clk_hz >> 4) + (clk_hz >> 5);
            while temp >= (10_000_000 >> 6) {
                temp -= 10_000_000 >> 6;
                val += 1;
            }
        } else {
            val = (clk_hz * 499) >> 6;
        }
    } else if row == 0xB {
        // 12-row refresh
        if clk_hz >= 1_000_000 {
            let mut temp = clk_hz + (clk_hz >> 3) + (clk_hz >> 4) + (clk_hz >> 5);
            while temp >= (10_000_000 >> 7) {
                temp -= 10_000_000 >> 7;
                val += 1;
            }
        } else {
            val = (clk_hz * 499) >> 5;
        }
    }
    dram_write(reg::SREFR, val);
}

/// Write SCONR register from parameters and trigger initialization.
#[link_section = ".text.spl"]
unsafe fn dram_para_setup(p: &DramParams) -> bool {
    let val = (p.ddr8_remap)
        | (1 << 1)
        | ((p.bank_size >> 2) << 3)
        | ((p.cs_num >> 1) << 4)
        | ((p.row_width - 1) << 5)
        | ((p.col_width - 1) << 9)
        | (if p.dram_type == DramType::Sdr {
            p.bus_width >> 5
        } else {
            p.bus_width >> 4
        } << 13)
        | (p.access_mode << 15)
        | ((p.dram_type as u32) << 16);

    dram_write(reg::SCONR, val);
    dram_write(
        reg::SCTLR,
        dram_read(reg::SCTLR) | (1 << 19),
    );
    dram_initial()
}

/// Count valid delay bits across all DRPTR registers.
#[link_section = ".text.spl"]
unsafe fn dram_check_delay(bwidth: u32) -> u32 {
    let dsize = if bwidth == 16 { 4 } else { 2 };
    let mut num: u32 = 0;

    for i in 0..dsize {
        let mut dflag = match i {
            0 => dram_read(reg::DRPTR0),
            1 => dram_read(reg::DRPTR1),
            2 => dram_read(reg::DRPTR2),
            _ => dram_read(reg::DRPTR3),
        };

        for _ in 0..32 {
            if dflag & 0x1 != 0 {
                num += 1;
            }
            dflag >>= 1;
        }
    }
    num
}

/// Verify DRAM read pipe by writing/reading test pattern.
#[link_section = ".text.spl"]
unsafe fn sdr_readpipe_scan() -> bool {
    for k in 0..32u32 {
        ((DRAM_MEM_BASE + 4 * k) as *mut u32).write_volatile(k);
    }
    for k in 0..32u32 {
        if ((DRAM_MEM_BASE + 4 * k) as *const u32).read_volatile() != k {
            return false;
        }
    }
    true
}

/// Scan for best SDR read pipe value.
#[link_section = ".text.spl"]
unsafe fn sdr_readpipe_select() -> u32 {
    for i in 0..8u32 {
        let val = (dram_read(reg::SCTLR) & !(0x7 << 6)) | (i << 6);
        dram_write(reg::SCTLR, val);
        if sdr_readpipe_scan() {
            return i;
        }
    }
    0
}

/// Detect DRAM type (DDR vs SDR) by testing delay line behavior.
#[link_section = ".text.spl"]
unsafe fn dram_check_type(p: &mut DramParams) -> u32 {
    let mut times: u32 = 0;

    for i in 0..8u32 {
        let val = (dram_read(reg::SCTLR) & !(0x7 << 6)) | (i << 6);
        dram_write(reg::SCTLR, val);
        dram_delay_scan();
        if dram_read(reg::DDLYR) & 0x30 != 0 {
            times += 1;
        }
    }

    if times == 8 {
        p.dram_type = DramType::Sdr;
        0
    } else {
        p.dram_type = DramType::Ddr;
        1
    }
}

/// Scan read pipe for DDR memory to find best delay.
#[link_section = ".text.spl"]
unsafe fn dram_scan_readpipe(p: &mut DramParams) {
    if p.dram_type == DramType::Ddr {
        let mut rp_best: u32 = 0;
        let mut rp_val: u32 = 0;
        let mut readpipe = [0u32; 8];

        for i in 0..8u32 {
            let val = (dram_read(reg::SCTLR) & !(0x7 << 6)) | (i << 6);
            dram_write(reg::SCTLR, val);
            dram_delay_scan();
            readpipe[i as usize] = 0;

            let ddlyr = dram_read(reg::DDLYR);
            if ((ddlyr >> 4) & 0x3) == 0x0 && ((ddlyr >> 4) & 0x1) == 0x0 {
                readpipe[i as usize] = dram_check_delay(p.bus_width);
            }
            if rp_val < readpipe[i as usize] {
                rp_val = readpipe[i as usize];
                rp_best = i;
            }
        }

        let val = (dram_read(reg::SCTLR) & !(0x7 << 6)) | (rp_best << 6);
        dram_write(reg::SCTLR, val);
        dram_delay_scan();
    } else {
        // SDR mode
        let val = dram_read(reg::SCONR) & !(1 << 16) & !(0x3 << 13);
        dram_write(reg::SCONR, val);
        let rp_best = sdr_readpipe_select();
        let val = (dram_read(reg::SCTLR) & !(0x7 << 6)) | (rp_best << 6);
        dram_write(reg::SCTLR, val);
    }
}

/// Auto-detect column width, row width, and total DRAM size.
#[link_section = ".text.spl"]
unsafe fn dram_get_dram_size(p: &mut DramParams) {
    p.col_width = 10;
    p.row_width = 13;
    dram_para_setup(p);
    dram_scan_readpipe(p);

    // Detect column width: write to addresses 0x80000200 and 0x80000600
    // If they alias (col=9), we'll see 0x22222222 at 0x80000200.
    // xboot steps by 1 BYTE here (unaligned u32 writes) and relies on the
    // legacy ARM926 unaligned behavior (A bit clear). Our cpu_init_crit
    // enables alignment faults, so an unaligned access data-aborts into an
    // unloaded vector → hang. Step by 4 instead — the patterns are
    // byte-uniform and the alias distance is 0x400, so detection semantics
    // are identical.
    for i in 0..32u32 {
        ((DRAM_MEM_BASE + 0x200 + 4 * i) as *mut u32).write_volatile(0x11111111);
        ((DRAM_MEM_BASE + 0x600 + 4 * i) as *mut u32).write_volatile(0x22222222);
    }

    let mut count: u32 = 0;
    for i in 0..32u32 {
        if ((DRAM_MEM_BASE + 0x200 + 4 * i) as *const u32).read_volatile() == 0x22222222 {
            count += 1;
        }
    }

    if count == 32 {
        p.col_width = 9;
    } else {
        p.col_width = 10;
    }

    // Re-setup with detected col_width
    dram_para_setup(p);

    // Detect row width: write above/below the suspected boundary
    let (addr1, addr2) = if p.col_width == 10 {
        (DRAM_MEM_BASE + 0x400000, DRAM_MEM_BASE + 0xC00000)
    } else {
        (DRAM_MEM_BASE + 0x200000, DRAM_MEM_BASE + 0x600000)
    };

    // Aligned stepping for the same reason as the column detection above.
    count = 0;
    for i in 0..32u32 {
        ((addr1 + 4 * i) as *mut u32).write_volatile(0x33333333);
        ((addr2 + 4 * i) as *mut u32).write_volatile(0x44444444);
    }
    for i in 0..32u32 {
        if ((addr1 + 4 * i) as *const u32).read_volatile() == 0x44444444 {
            count += 1;
        }
    }

    if count == 32 {
        p.row_width = 12;
    } else {
        p.row_width = 13;
    }

    // Determine size
    p.size_mb = if p.row_width != 13 {
        16
    } else if p.col_width == 10 {
        64
    } else {
        32
    };

    dram_set_autofresh_cycle(p.clk_mhz * 1_000_000);
    p.access_mode = 0;
    dram_para_setup(p);
}

// ── Main DRAM Init ────────────────────────────────────────────────────────

/// Full DRAM initialization with auto-detection.
///
/// Configures PLL_DDR, DRAM timing, detects type/size, and verifies
/// basic read/write. Returns `true` on success.
///
/// # Safety
/// This must be called once during early boot, after clocks are up
/// but before any DRAM use. Stack must be in SRAM or other non-DRAM memory.
#[link_section = ".text.spl"]
pub unsafe fn init() -> bool {
    // xboot convention: ('X' << 24) | size_mb is written to 0x0000005c after a
    // successful DRAM init.  "xfel ddr" uses the same DRAM init code path, so
    // if the top byte is already 'X' the controller is alive — skip reinit.
    // This prevents reinitializing the DRAM controller while executing from it.
    let dsz = 0x0000005Cu32 as *const u32;
    if dsz.read_volatile() >> 24 == b'X' as u32 {
        return true;
    }

    let mut p = DramParams::default();

    // 1. Configure GPIO drive strength for DRAM pins
    //    GPIO port C config: DRV1 register at offset 0x24 of GPIO base
    //    Set bits 12-14 (drive for pins 6-7) to maximum
    let gpio_drv = (GPIO_BASE + 0x24) as *mut u32;
    gpio_drv.write_volatile(gpio_drv.read_volatile() | (0x7 << 12));
    dram_delay_ms(5);

    // 2. Configure PLL_DDR (matches xboot dram_init logic exactly)
    //    For clk > 96 MHz: m=0, divisor=24 → PLL = 24*(n+1) = clk*2
    //    For clk <= 96 MHz: m=1, divisor=12 → PLL = 24*(n+1)/2 = clk*2
    //    clk_mhz=156: m=0, n=156*2/24-1=12 → PLL=24*13=312 MHz (DDR=156 MHz)
    let (m, n) = if p.clk_mhz <= 96 {
        (1u32, p.clk_mhz * 2 / 12 - 1)
    } else {
        (0u32, p.clk_mhz * 2 / 24 - 1)
    };
    let pll_val = m | (0u32 << 4) | (n << 8) | (1u32 << 31);

    // Set SDRAM pad drive strength (PIO SDR_PAD_DRV at GPIO_BASE + 0x2C0).
    // xboot: write32(0x01c20800 + 0x2c0, ...) — this lives in the PIO block,
    // not the CCU (CCU_BASE + 0x2C0 is BUS_SOFT_RST0!).
    let sdr_pad_drv = (GPIO_BASE + 0x2C0) as *mut u32;
    if p.clk_mhz >= 180 {
        sdr_pad_drv.write_volatile(0xFFF);
    } else if p.clk_mhz >= 144 {
        sdr_pad_drv.write_volatile(0xAAA);
    }

    // Program PLL_DDR and wait for stable (bounded — a missing lock bit
    // must fail loudly instead of hanging silently)
    let pll_addr = (clock::CCU_BASE + 0x020) as *mut u32;
    pll_addr.write_volatile(pll_val);
    pll_addr.write_volatile(pll_addr.read_volatile() | (1 << 20)); // latch update
    let mut timeout: u32 = 0xFFFFFF;
    while pll_addr.read_volatile() & (1 << 28) == 0 {
        timeout -= 1;
        if timeout == 0 {
            return false;
        }
    }
    dram_delay_ms(5);

    // 3. Enable DRAM clock gate and pulse the controller reset.
    //    BUS_SOFT_RST0 bit 14: 0 = reset asserted, 1 = running.
    let gate0_addr = (clock::CCU_BASE + 0x060) as *mut u32;
    gate0_addr.write_volatile(gate0_addr.read_volatile() | (1 << 14)); // SDRAM gate enable

    let soft_rst0_addr = (clock::CCU_BASE + 0x2C0) as *mut u32;
    soft_rst0_addr.write_volatile(soft_rst0_addr.read_volatile() & !(1 << 14)); // assert reset
    dram_delay_ms(1); // xboot uses a ~10-cycle pulse; give the controller real settle time
    soft_rst0_addr.write_volatile(soft_rst0_addr.read_volatile() | (1 << 14)); // release reset
    dram_delay_ms(1);

    // 4. Select pad mode for the DRAM type
    //    (PIO SDR_PAD_PUL at GPIO_BASE + 0x2C4, bit 16: 1 = DDR, 0 = SDR)
    let sdr_pad_pul = (GPIO_BASE + 0x2C4) as *mut u32;
    let mut pad_cfg = sdr_pad_pul.read_volatile();
    if p.dram_type == DramType::Ddr {
        pad_cfg |= 1 << 16;
    } else {
        pad_cfg &= !(1 << 16);
    }
    sdr_pad_pul.write_volatile(pad_cfg);

    // 5. Set SDRAM timing registers
    let stmg0 = (T_CAS << 0)
        | (T_RAS << 3)
        | (T_RCD << 7)
        | (T_RP << 10)
        | (T_WR << 13)
        | (T_RFC << 15)
        | (T_XSR << 19)
        | (T_RC << 28);
    dram_write(reg::STMG0R, stmg0);

    let stmg1 = (T_INIT << 0)
        | (T_INIT_REF << 16)
        | (T_WTR << 20)
        | (T_RRD << 22)
        | (T_XP << 25);
    dram_write(reg::STMG1R, stmg1);

    // 6. Setup DRAM parameters and initialize
    if !dram_para_setup(&p) {
        return false; // controller init timed out
    }

    // 7. Detect DRAM type (DDR/SDR)
    dram_check_type(&mut p);

    // 8. Update pad mode with detected type
    pad_cfg = sdr_pad_pul.read_volatile();
    if p.dram_type == DramType::Ddr {
        pad_cfg |= 1 << 16;
    } else {
        pad_cfg &= !(1 << 16);
    }
    sdr_pad_pul.write_volatile(pad_cfg);

    // 9. Set auto-refresh and scan read pipe
    dram_set_autofresh_cycle(p.clk_mhz * 1_000_000);
    dram_scan_readpipe(&mut p);

    // 10. Auto-detect size
    dram_get_dram_size(&mut p);

    // 11. Verify: write/read test pattern
    for i in 0..128u32 {
        // Actually the C code writes to para->base which is DRAM_MEM_BASE
        ((DRAM_MEM_BASE + 4 * i) as *mut u32).write_volatile(DRAM_MEM_BASE + 4 * i);
    }

    for i in 0..128u32 {
        let expected = DRAM_MEM_BASE + 4 * i;
        if ((DRAM_MEM_BASE + 4 * i) as *const u32).read_volatile() != expected {
            return false;
        }
    }
    // Store magic and size at a known location (used by bootloader chain)
    let dsz = 0x0000005Cu32 as *mut u32;
    dsz.write_volatile(((b'X' as u32) << 24) | p.size_mb);

    true
}

/// Returns the detected DRAM size in megabytes.
/// Only valid after `init()` has succeeded.
#[link_section = ".text.spl"]
pub fn size_mb() -> u32 {
    unsafe {
        let dsz = 0x0000005Cu32 as *const u32;
        dsz.read_volatile() & 0xFFFFFF
    }
}
