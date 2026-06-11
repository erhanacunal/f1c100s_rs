//! Clock Control Unit (CCU) driver for Allwinner F1C100s
//!
//! The CCU manages all PLLs, clock source selection, dividers,
//! bus clock gating, and software reset lines.
//!
//! ## Clock Tree
//! ```
//! OSC24M (24 MHz) ─┬─ PLL_CPU ──── CPU_CLK ──────── HCLKC (CPU/1,2,3,4)
//!                   │   (408 MHz)
//!                   ├─ PLL_PERIPH ── AHB_CLK ──────── APB_CLK (AHB/2,4,8)
//!                   │   (600 MHz)    (200 MHz)        (100 MHz)
//!                   ├─ PLL_AUDIO ── Audio codecs
//!                   ├─ PLL_VIDEO ── Display engine
//!                   ├─ PLL_VE ───── Video engine
//!                   └─ PLL_DDR ──── DRAM controller
//! ```

// ── Base Address ────────────────────────────────────────────────────────

/// CCU register base address (public for use by other peripherals)
pub const CCU_BASE: u32 = 0x01C20000;

pub mod reg {
    pub const CCU_PLL_CPU_CTRL: u32 = 0x000;
    pub const CCU_PLL_AUDIO_CTRL: u32 = 0x008;
    pub const CCU_PLL_VIDEO_CTRL: u32 = 0x010;
    pub const CCU_PLL_VE_CTRL: u32 = 0x018;
    pub const CCU_PLL_DDR_CTRL: u32 = 0x020;
    pub const CCU_PLL_PERIPH_CTRL: u32 = 0x028;
    pub const CCU_CPU_CFG: u32 = 0x050;
    pub const CCU_AHB_APB_CFG: u32 = 0x054;
}

/// Read a CCU register
#[link_section = ".text.spl"]
unsafe fn read_reg(offset_bytes: u32) -> u32 {
    let ptr = (CCU_BASE + offset_bytes) as *const u32;
    ptr.read_volatile()
}

/// Write a CCU register
#[link_section = ".text.spl"]
unsafe fn write_reg(offset_bytes: u32, val: u32) {
    let ptr = (CCU_BASE + offset_bytes) as *mut u32;
    ptr.write_volatile(val);
}

// ── Known Constants ─────────────────────────────────────────────────────

/// 24 MHz oscillator frequency
pub const HZ_24M: u32 = 24_000_000;

/// 32 kHz internal oscillator
pub const HZ_32K: u32 = 32_000;

// ── Clock Source Selection ──────────────────────────────────────────────

/// Clock source mux values (for cpu_clk_src and AHB source select)
pub mod src {
    pub const LOSC: u32 = 0x00; // 32 kHz internal oscillator
    pub const OSC24M: u32 = 0x01; // 24 MHz crystal
    pub const PLL: u32 = 0x02; // PLL output
    pub const PRE_DIV: u32 = 0x03; // PLL_PERIPH ÷ pre-divider (AHB only)
}

// ── PLL Register Bit-Field Helpers ──────────────────────────────────────

pub mod pll_cpu_mask {
    pub const ENABLE: u32 = 1 << 31;
    pub const STABLE: u32 = 1 << 28;
    pub fn p(reg: u32) -> u32 {
        (reg >> 16) & 0x3
    }
    pub fn n(reg: u32) -> u32 {
        (reg >> 8) & 0x1F
    }
    pub fn k(reg: u32) -> u32 {
        (reg >> 4) & 0x3
    }
    pub fn m(reg: u32) -> u32 {
        reg & 0x3
    }
}

pub mod pll_audio_mask {
    pub const ENABLE: u32 = 1 << 31;
    pub const STABLE: u32 = 1 << 28;
    pub fn n(reg: u32) -> u32 {
        (reg >> 8) & 0x7F
    }
    pub fn m(reg: u32) -> u32 {
        reg & 0x1F
    }
}

pub mod pll_video_mask {
    pub const ENABLE: u32 = 1 << 31;
    pub const MODE: u32 = 1 << 30;
    pub const STABLE: u32 = 1 << 28;
    pub const FRAC_OUT: u32 = 1 << 25;
    pub const MODE_SEL: u32 = 1 << 24;
    pub fn n(reg: u32) -> u32 {
        (reg >> 8) & 0x7F
    }
    pub fn m(reg: u32) -> u32 {
        reg & 0xF
    }
}

pub mod pll_ve_mask {
    pub const ENABLE: u32 = 1 << 31;
    pub const STABLE: u32 = 1 << 28;
    pub const FRAC_OUT: u32 = 1 << 25;
    pub const MODE_SEL: u32 = 1 << 24;
    pub fn n(reg: u32) -> u32 {
        (reg >> 8) & 0x7F
    }
    pub fn m(reg: u32) -> u32 {
        reg & 0xF
    }
}

pub mod pll_ddr_mask {
    pub const ENABLE: u32 = 1 << 31;
    pub const STABLE: u32 = 1 << 28;
    pub fn n(reg: u32) -> u32 {
        (reg >> 8) & 0x1F
    }
    pub fn k(reg: u32) -> u32 {
        (reg >> 4) & 0x3
    }
    pub fn m(reg: u32) -> u32 {
        reg & 0x3
    }
}

pub mod pll_periph_mask {
    pub const ENABLE: u32 = 1 << 31;
    pub const STABLE: u32 = 1 << 28;
    pub const OUT_24M_EN: u32 = 1 << 18;
    pub fn n(reg: u32) -> u32 {
        (reg >> 8) & 0x1F
    }
    pub fn k(reg: u32) -> u32 {
        (reg >> 4) & 0x3
    }
}

pub mod pll_channel {
    use crate::clock::reg;

    pub const CPU: u32 = reg::CCU_PLL_CPU_CTRL;
    pub const AUDIO: u32 = reg::CCU_PLL_AUDIO_CTRL;
    pub const VIDEO: u32 = reg::CCU_PLL_VIDEO_CTRL;
    pub const VE: u32 = reg::CCU_PLL_VE_CTRL;
    pub const DDR: u32 = reg::CCU_PLL_DDR_CTRL;
    pub const PERIPH: u32 = reg::CCU_PLL_PERIPH_CTRL;
}

// ── AHB/APB bit-fields ──────────────────────────────────────────────────

pub mod ahb_apb_mask {
    /// CPU clock divider (bits 17:16): 00=/1, 01=/2, 10=/3, 11=/4
    pub fn hclkc_div(reg: u32) -> u32 {
        (reg >> 16) & 0x3
    }
    /// AHB source (bits 13:12): 00=LOSC, 01=OSC24M, 10=PLL, 11=PLL_PERIPH/pre-div
    pub fn ahb_src(reg: u32) -> u32 {
        (reg >> 12) & 0x3
    }
    /// APB divisor (bits 9:8): 00=/2, 01=/2, 10=/4, 11=/8
    pub fn apb_div(reg: u32) -> u32 {
        (reg >> 8) & 0x3
    }
    /// AHB pre-divider (bits 7:6): 00=/1, 01=/2, 10=/3, 11=/4
    pub fn ahb_pre_div(reg: u32) -> u32 {
        (reg >> 6) & 0x3
    }
    /// AHB clock divider (bits 5:4): 2^n
    pub fn ahb_div(reg: u32) -> u32 {
        (reg >> 4) & 0x3
    }
}

pub mod abp_div {
    pub const DIV2: u32 = 0x1;
    pub const DIV4: u32 = 0x2;
    pub const DIV8: u32 = 0x3;
}

// ── PLL Wait & Frequency Queries ────────────────────────────────────────

/// Short busy-wait loop (matches xboot's sdelay)
#[link_section = ".text.spl"]
#[allow(unused_assignments)]
fn sdelay(loops: u32) {
    let mut n = loops;
    unsafe {
        core::arch::asm!(
            "1:",
            "subs {0}, {0}, #1",
            "bne 1b",
            inout(reg) n,
            options(nostack),
        );
    }
}

/// Spin-wait for a PLL to become stable (bit 28 set)
#[link_section = ".text.spl"]
fn wait_pll_stable(offset: u32) -> bool {
    let mut timeout: u32 = 0xFFFF;
    while timeout > 0 {
        let val = unsafe { read_reg(offset) };
        if val & (1 << 28) != 0 {
            return true;
        }
        timeout -= 1;
    }
    false
}

/// Get PLL_CPU output frequency in Hz
pub fn cpu_pll_hz() -> u32 {
    let ctl = unsafe { read_reg(reg::CCU_PLL_CPU_CTRL) };
    if ctl & (1 << 31) == 0 {
        return 0;
    }
    let n = pll_cpu_mask::n(ctl) + 1;
    let k = pll_cpu_mask::k(ctl) + 1;
    let m = pll_cpu_mask::m(ctl) + 1;
    let mut p = pll_cpu_mask::p(ctl);
    // p encoding (bits 17:16): 00=/1, 01=/2, 10=/4, 11=/4
    p = 1 << p;
    // Fout = 24MHz * n * k / (m * p)
    HZ_24M * n * k / (m * p)
}

/// Get PLL_PERIPH output frequency in Hz
pub fn periph_pll_hz() -> u32 {
    let ctl = unsafe { read_reg(0x028) };
    if ctl & (1 << 31) == 0 {
        return 0;
    }
    let n = pll_periph_mask::n(ctl) + 1;
    let k = pll_periph_mask::k(ctl) + 1;
    // Fout = 24MHz * n * k
    HZ_24M * n * k
}

/// Get PLL_AUDIO output frequency in Hz
pub fn audio_pll_hz() -> u32 {
    let ctl = unsafe { read_reg(0x008) };
    if ctl & (1 << 31) == 0 {
        return 0;
    }
    let n = pll_audio_mask::n(ctl) + 1;
    let m = pll_audio_mask::m(ctl) + 1;
    HZ_24M * 2 * n / m
}

/// Get PLL_VIDEO output frequency in Hz
pub fn video_pll_hz() -> u32 {
    let ctl = unsafe { read_reg(0x010) };
    if ctl & (1 << 31) == 0 {
        return 0;
    }
    if ctl & pll_video_mask::MODE_SEL != 0 {
        let n = pll_video_mask::n(ctl) + 1;
        let m = pll_video_mask::m(ctl) + 1;
        return (HZ_24M * n) / m;
    }
    if ctl & pll_video_mask::FRAC_OUT != 0 {
        270_000_000
    } else {
        297_000_000
    }
}

/// Get PLL_VE output frequency in Hz
pub fn ve_pll_hz() -> u32 {
    let ctl = unsafe { read_reg(0x018) };
    if ctl & (1 << 31) == 0 {
        return 0;
    }
    if ctl & pll_ve_mask::MODE_SEL != 0 {
        let n = pll_ve_mask::n(ctl) + 1;
        let m = pll_ve_mask::m(ctl) + 1;
        return HZ_24M * n / m;
    }
    if ctl & pll_ve_mask::FRAC_OUT != 0 {
        297_000_000
    } else {
        270_000_000
    }
}

/// Get PLL_DDR output frequency in Hz
pub fn ddr_pll_hz() -> u32 {
    let ctl = unsafe { read_reg(0x020) };
    if ctl & (1 << 31) == 0 {
        return 0;
    }
    let n = pll_ddr_mask::n(ctl) + 1;
    let k = pll_ddr_mask::k(ctl) + 1;
    let m = pll_ddr_mask::m(ctl) + 1;
    HZ_24M * n * k / m
}

// ── System Clock Frequency Queries ──────────────────────────────────────

/// Get current CPU clock source (0=LOSC, 1=OSC24M, 2=PLL)
#[link_section = ".text.spl"]
pub fn cpu_clk_src() -> u32 {
    unsafe { (read_reg(reg::CCU_CPU_CFG) >> 16) & 0x3 }
}

/// Get current CPU clock frequency
#[link_section = ".text.spl"]
pub fn cpu_hz() -> u32 {
    let src = cpu_clk_src();    
    match src {
        src::PLL => cpu_pll_hz(),
        src::OSC24M => HZ_24M,
        src::LOSC => HZ_32K,
        _ => 0,
    }
}

/// Get current AHB bus frequency
#[link_section = ".text.spl"]
pub fn ahb_hz() -> u32 {
    let cfg = unsafe { read_reg(0x054) };
    let src = ahb_apb_mask::ahb_src(cfg);
    let mut div = ahb_apb_mask::ahb_div(cfg);
    let prediv = ahb_apb_mask::ahb_pre_div(cfg);
    div = 1 << div; // Convert from power-of-2 encoding
    match src {
        src::LOSC => HZ_32K / div,
        src::OSC24M => HZ_24M / div,
        src::PLL => cpu_pll_hz() / div,
        src::PRE_DIV => periph_pll_hz() / (prediv + 1) / div,
        _ => 0,
    }
}

/// Get current APB bus frequency
#[link_section = ".text.spl"]
pub fn apb_hz() -> u32 {
    let cfg = unsafe { read_reg(0x054) };
    let div = ahb_apb_mask::apb_div(cfg);
    let ahb = ahb_hz();
    if div == abp_div::DIV4 {
        ahb / 4
    } else if div == abp_div::DIV8 {
        ahb / 8
    } else {
        ahb / 2
    }
}
// ── Bus Clock Gating ────────────────────────────────────────────────────

const BUS_GATE_SHIFT: u32 = 12;

/// Bus gate identifiers for peripheral clock gating
#[derive(Debug, Clone, Copy)]
pub enum BusGate {
    // Gate 0 (reg 0x060)
    UsbOtg = (0x18 | (0 << BUS_GATE_SHIFT)),
    Spi1 = (0x15 | (0 << BUS_GATE_SHIFT)),
    Spi0 = (0x14 | (0 << BUS_GATE_SHIFT)),
    Sdram = (0x0E | (0 << BUS_GATE_SHIFT)),
    Sd1 = (0x09 | (0 << BUS_GATE_SHIFT)),
    Sd0 = (0x08 | (0 << BUS_GATE_SHIFT)),
    Dma = (0x06 | (0 << BUS_GATE_SHIFT)),
    // Gate 1 (reg 0x064)
    Defe = (0x0E | (1 << BUS_GATE_SHIFT)),
    Debe = (0x0C | (1 << BUS_GATE_SHIFT)),
    Tve = (0x0A | (1 << BUS_GATE_SHIFT)),
    Tvd = (0x09 | (1 << BUS_GATE_SHIFT)),
    Csi = (0x08 | (1 << BUS_GATE_SHIFT)),
    Deinterlace = (0x05 | (1 << BUS_GATE_SHIFT)),
    Lcd = (0x04 | (1 << BUS_GATE_SHIFT)),
    Ve = (0x00 | (1 << BUS_GATE_SHIFT)),
    // Gate 2 (reg 0x068)
    Uart2 = (0x16 | (2 << BUS_GATE_SHIFT)),
    Uart1 = (0x15 | (2 << BUS_GATE_SHIFT)),
    Uart0 = (0x14 | (2 << BUS_GATE_SHIFT)),
    Twi2 = (0x12 | (2 << BUS_GATE_SHIFT)),
    Twi1 = (0x11 | (2 << BUS_GATE_SHIFT)),
    Twi0 = (0x10 | (2 << BUS_GATE_SHIFT)),
    Daudio = (0x0C | (2 << BUS_GATE_SHIFT)),
    Rsb = (0x03 | (2 << BUS_GATE_SHIFT)),
    Cir = (0x02 | (2 << BUS_GATE_SHIFT)),
    Owa = (0x01 | (2 << BUS_GATE_SHIFT)),
    AudioCodec = (0x00 | (2 << BUS_GATE_SHIFT)),
}

/// DRAM gate identifiers
#[derive(Debug, Clone, Copy)]
pub enum DramGate {
    Be = 26,
    Fe = 24,
    Tvd = 3,
    Deinterlace = 2,
    Csi = 1,
    Ve = 0,
}

/// Enable the clock gate for a peripheral
#[link_section = ".text.spl"]
pub fn bus_gate_enable(gate: BusGate) {
    let val = gate as u32;
    let offset = val & 0xFFF;
    let reg = ((val >> BUS_GATE_SHIFT) & 0x3) as u32;
    let reg_offset = match reg {
        0 => 0x060,
        1 => 0x064,
        2 => 0x068,
        _ => return,
    };
    unsafe {
        let cur = read_reg(reg_offset);
        write_reg(reg_offset, cur | (1 << offset));
    }
}

/// Disable the clock gate for a peripheral
pub fn bus_gate_disable(gate: BusGate) {
    let val = gate as u32;
    let offset = val & 0xFFF;
    let reg = ((val >> BUS_GATE_SHIFT) & 0x3) as u32;
    let reg_offset = match reg {
        0 => 0x060,
        1 => 0x064,
        2 => 0x068,
        _ => return,
    };
    unsafe {
        let cur = read_reg(reg_offset);
        write_reg(reg_offset, cur & !(1 << offset));
    }
}

// ── Software Reset ──────────────────────────────────────────────────────

/// Assert software reset for a peripheral (Bus_Soft_Rst bit=0 → in reset)
#[link_section = ".text.spl"]
pub fn sw_reset_assert(gate: BusGate) {
    let val = gate as u32;
    let offset = val & 0xFFF;
    let reg = ((val >> BUS_GATE_SHIFT) & 0x3) as u32;
    let reg_offset = match reg {
        0 => 0x2C0,
        1 => 0x2C4,
        2 => 0x2D0,
        _ => return,
    };
    unsafe {
        let cur = read_reg(reg_offset);
        write_reg(reg_offset, cur & !(1 << offset));
    }
}

/// De-assert software reset for a peripheral (Bus_Soft_Rst bit=1 → normal operation)
#[link_section = ".text.spl"]
pub fn sw_reset_deassert(gate: BusGate) {
    let val = gate as u32;
    let offset = val & 0xFFF;
    let reg = ((val >> BUS_GATE_SHIFT) & 0x3) as u32;
    let reg_offset = match reg {
        0 => 0x2C0,
        1 => 0x2C4,
        2 => 0x2D0,
        _ => return,
    };
    unsafe {
        let cur = read_reg(reg_offset);
        write_reg(reg_offset, cur | (1 << offset));
    }
}

/// Reset a peripheral: assert, then de-assert the reset line
#[link_section = ".text.spl"]
pub fn sw_reset_peripheral(gate: BusGate) {
    sw_reset_assert(gate);
    // Brief delay for reset to take effect
    for _ in 0..100 {
        unsafe {
            core::arch::asm!("nop");
        }
    }
    sw_reset_deassert(gate);
}

/// Enable clock gate and release reset for a peripheral (standard init sequence)
#[link_section = ".text.spl"]
pub fn bus_clk_init(gate: BusGate) {
    bus_gate_enable(gate);
    sw_reset_peripheral(gate);
}

// ── DRAM Gating ─────────────────────────────────────────────────────────

/// Enable DRAM clock gate
pub fn dram_gate_enable(gate: DramGate) {
    unsafe {
        let cur = read_reg(0x100);
        write_reg(0x100, cur | (1 << (gate as u32)));
    }
}

/// Disable DRAM clock gate
pub fn dram_gate_disable(gate: DramGate) {
    unsafe {
        let cur = read_reg(0x100);
        write_reg(0x100, cur & !(1 << (gate as u32)));
    }
}
#[link_section = ".text.spl"]
fn clk_hclk_config(div: u32) {
    if div == 0 || div > 4 {
        return;
    }
    unsafe {
        let cfg = read_reg(0x054) & !(0x3 << 16);
        write_reg(0x054, cfg | ((div - 1) << 16));
    }
}
#[link_section = ".text.spl"]
fn clk_ahb_config(src: u32, pre_div: u32, mut div: u32) {
    if pre_div == 0 || pre_div > 4 || div == 0 || ((div > 4) && (div != 8) || (div == 3)) {
        return;
    }
    if div == 3 {
        div = 4; // div=3 is encoded the same as div=4
    }
    if div == 8 {
        div = 4; // div=8 is encoded the same as div=4 with a different APB divisor
    }
    unsafe {
        let cfg = read_reg(reg::CCU_AHB_APB_CFG) & !((0x3 << 12) | (0xF << 4));
        write_reg(
            reg::CCU_AHB_APB_CFG,
            cfg | (src << 12) | ((pre_div - 1) << 6) | ((div - 1) << 4),
        );
    }
}
#[link_section = ".text.spl"]
fn clk_apb_config(div: u32) {
    if div == 0 || div > 8 || div == 3 || div == 5 || div == 6 || div == 7 {
        return;
    }
    let apb_div = match div {
        1 | 2 => 0x1, // APB divisor of 1 or 2 is encoded the same way
        4 => 0x2,
        8 => 0x3,
        _ => return,
    };
    unsafe {
        let cfg = read_reg(reg::CCU_AHB_APB_CFG) & !(0x3 << 8);
        write_reg(reg::CCU_AHB_APB_CFG, cfg | (apb_div << 8));
    }
}
#[link_section = ".text.spl"]
fn clk_cpu_config(src: u32) {
    if src > src::PLL {
        return;
    }
    unsafe {
        let cfg = read_reg(0x050) & !(0x3 << 16);
        write_reg(0x050, cfg | (src << 16));
    }
}
#[link_section = ".text.spl"]
fn pll_cpu_init(mul: u32, div: u32) {
    if mul == 0 || div == 0 || mul > 128 || div > 16 {
        return;
    }
    unsafe {
        let mut n: u32 = 0;
        let mut k: u32 = 0;
        let mut m: u32 = 0;
        let mut p: u32 = 0;
        for i in 1..5 {
            k = i;
            n = mul / i;
            if n < 32 && (n * i == mul) {
                break;
            }
        }
        if n * k != mul {
            return;
        }
        for i in 1..5 {
            m = i;
            p = div / i;
            if (p == 1 || p == 2 || p == 4) && (i * p == div) {
                break;
            }
        }
        if m * p != div {
            return;
        }
        p -= 1;
        if p == 3 {
            p = 2; // p=3 is encoded the same as p=2
        }
        let mut val = read_reg(reg::CCU_PLL_CPU_CTRL);
        val &= (1 << 31) | (1 << 28);
        val |= ((n - 1) << 8) | ((k - 1) << 4) | (m - 1) | (p << 16);
        write_reg(reg::CCU_PLL_CPU_CTRL, val);
    }
}
#[link_section = ".text.spl"]
fn pll_video_init(pll: u32, mul: u32, div: u32) {
    if mul == 0 || div == 0 || mul > 128 || div > 16 {
        return;
    }
    unsafe {
        let mut val = read_reg(pll);
        val &= (1 << 31) | (1 << 28);
        val |= ((mul - 1) << 8) | (div - 1) | (1 << 24); // MODE_SEL=1 for video PLLs
        write_reg(pll, val);
    }
}
#[link_section = ".text.spl"]
fn pll_periph_init(mul: u32, div: u32) {
    if mul == 0 || div == 0 || mul > 32 || div > 4 {
        return;
    }
    unsafe {
        let mut val = read_reg(pll_channel::PERIPH);
        val &= (1 << 31) | (1 << 28);
        val |= ((mul - 1) << 8) | ((div - 1) << 4) | (1 << 18); // do we need 24m output?;
        write_reg(pll_channel::PERIPH, val);
    }
}
#[link_section = ".text.spl"]
fn clk_pll_init(pll: u32, mul: u32, div: u32) {
    match pll {
        pll_channel::CPU => pll_cpu_init(mul, div),
        pll_channel::VIDEO | pll_channel::VE => pll_video_init(pll, mul, div),
        pll_channel::PERIPH => pll_periph_init(mul, div),
        _ => {
            // Other PLLs not implemented yet
        }
    }
}
#[link_section = ".text.spl"]
fn clk_pll_enable(pll: u32) {
    unsafe {
        write_reg(pll, read_reg(pll) | (1 << 31));
    }
}
#[link_section = ".text.spl"]
#[allow(dead_code)]
fn clk_pll_disable(pll: u32) {
    unsafe {
        write_reg(pll, read_reg(pll) & !(1 << 31));
    }
}

// ── Convenience: full default clock init ────────────────────────────────

/// Initialize clocks — mirrors xboot sys_clock_init exactly.
/// CPU: 408 MHz, PLL_PERIPH: 600 MHz, AHB: 200 MHz, APB: 100 MHz
#[link_section = ".text.spl"]
pub fn init_default() {
    unsafe {
        clk_cpu_config(src::OSC24M);
        sdelay(10);

        clk_pll_init(pll_channel::PERIPH, 25, 1);
        clk_pll_enable(pll_channel::PERIPH);

        wait_pll_stable(pll_channel::PERIPH);

        // Configure bus clocks: CPU=PLL/1, AHB=PLL_PERIPH/3, APB=AHB/2
        clk_hclk_config(1); // HCLKC = CPU clock / 1
        clk_ahb_config(src::PRE_DIV, 3, 1); // AHB = PLL_PERIPH / 3 / 1 = 200 MHz
        clk_apb_config(2); // APB = AHB / 2 = 100 MHz
        sdelay(100);

        clk_pll_init(pll_channel::VIDEO, 99, 8); // 297 MHz for display engine
        clk_pll_enable(pll_channel::VIDEO);
        wait_pll_stable(pll_channel::VIDEO);

        clk_pll_init(pll_channel::CPU, 17, 1); // 408 MHz for CPU: Fout = 24MHz * 17 = 408 MHz
        clk_pll_enable(pll_channel::CPU);
        wait_pll_stable(pll_channel::CPU);

        clk_cpu_config(src::PLL);

        // Enable DRAM-related clock gates in CCU_DRAM_CLK_GATE
        let val =
            read_reg(0x100) | (1 << 26) | (1 << 24) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0);
        write_reg(0x100, val);
        sdelay(100);
    }
}
