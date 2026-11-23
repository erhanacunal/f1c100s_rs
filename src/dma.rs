//! DMA controller driver for Allwinner F1C100s
//!
//! The F1C100s DMA block (at `0x01C0_2000`, INTC vector 18) has **8 channels**:
//!
//! | Channels | Kind           | Max bytes / hw transfer | Typical use                         |
//! | -------- | -------------- | ----------------------- | ----------------------------------- |
//! | N0 – N3  | Normal (NDMA)  | 128 KiB (`0x2_0000`)    | Peripheral FIFOs: SPI, UART, audio  |
//! | D0 – D3  | Dedicated (DDMA)| 16 MiB (`0x100_0000`)  | Memory copies, LCD, USB             |
//!
//! Each side (source / destination) of a transfer is described by an
//! [`Endpoint`]: a bus address, a DRQ port (memory or a peripheral request
//! line), an address mode (increment / fixed), a data width and a burst
//! length.
//!
//! Transfers longer than a channel's hardware limit are split into chunks
//! automatically; the next chunk is programmed from the completion
//! interrupt (or from [`wait`] when interrupts are disabled).
//!
//! ## Cache coherency
//!
//! For every [`Endpoint::memory`] side, [`start`] cleans the source range
//! and cleans+invalidates the destination range in the D-cache before the
//! hardware runs. The ARM926EJ-S does not prefetch speculatively, so no
//! post-transfer invalidate is needed **as long as the CPU does not touch
//! the destination buffer while the transfer is in flight**. Destination
//! buffers should be 32-byte (cache line) aligned and sized, otherwise a
//! neighbouring variable sharing the first/last line can be lost.
//!
//! In [`Mode::Cyclic`] the hardware loops over the same buffer forever; the
//! cache maintenance is done once at start and refilling the buffer
//! coherently (e.g. `mmu::clean_dcache` on the half just written) is the
//! caller's job.
//!
//! ## Example
//!
//! ```ignore
//! use f1c100s::dma::{self, Channel};
//!
//! dma::init();
//!
//! // Blocking memory-to-memory copy on a dedicated channel
//! dma::memcpy(Channel::D0, &mut dst, &src).unwrap();
//!
//! // Stream a buffer into the SPI0 TX FIFO
//! let tx = dma::Transfer::new(
//!     dma::Endpoint::memory(buf.as_ptr() as u32).width(dma::Width::Bits8).burst(dma::Burst::Single),
//!     dma::Endpoint::ndma(SPI0_TXD, dma::NdmaDrq::Spi0),
//!     buf.len() as u32,
//! );
//! unsafe { dma::start(Channel::N0, &tx).unwrap() };
//! dma::wait(Channel::N0);
//! ```
//!
//! Register layout and DRQ numbers follow xboot's `dma-f1c100s.c` and Linux
//! `drivers/dma/sun4i-dma.c` (`allwinner,suniv-f1c100s-dma`), which agree on CFG
//! bit fields (width at bits 24/8, burst 1/4, continuous at bit 29), SDRAM
//! DRQ numbers (NDMA 0x11, DDMA 0x1) and the 128 KiB NDMA limit. The channel
//! stride and DDMA IRQ bits were settled on hardware — see `CH_STRIDE`.

use core::ptr::{addr_of, addr_of_mut};

use crate::clock;
use crate::cpu;
use crate::interrupt;
use crate::mmu;

// ── Base Address ──────────────────────────────────────────────────────────

const DMA_BASE: u32 = 0x01C0_2000;

/// INTC vector of the DMA controller
pub const DMA_IRQ: u32 = interrupt::DMA_INTERRUPT;

// ── Register Offsets ──────────────────────────────────────────────────────

mod reg {
    /// Interrupt enable: per channel 2 bits (half, full).
    /// NDMA n → bits [2n+1:2n], DDMA n → bits [DDMA_IRQ_BASE + 2n + 1 : DDMA_IRQ_BASE + 2n]
    pub const INT_CTL: u32 = 0x00;
    /// Interrupt status (write 1 to clear), same layout as INT_CTL
    pub const INT_STA: u32 = 0x04;
    /// Priority / auto-gating configuration
    pub const PTY_CFG: u32 = 0x08;
}

/// Per-channel register offsets (relative to the channel base)
mod ch {
    pub const CFG: u32 = 0x00;
    pub const SRC: u32 = 0x04;
    pub const DST: u32 = 0x08;
    pub const CNT: u32 = 0x0C;
    /// DDMA only: page-mode parameters
    pub const PAR: u32 = 0x18;
    /// DDMA only: general data
    pub const GEN: u32 = 0x1C;
}

/// Channel configuration register bits
mod cfg {
    pub const fn src_drq(v: u32) -> u32 { v & 0x1F }
    pub const fn src_mode(v: u32) -> u32 { (v & 0x3) << 5 }
    pub const fn src_burst(v: u32) -> u32 { (v & 0x1) << 7 }
    pub const fn src_width(v: u32) -> u32 { (v & 0x3) << 8 }
    pub const fn dst_drq(v: u32) -> u32 { (v & 0x1F) << 16 }
    pub const fn dst_mode(v: u32) -> u32 { (v & 0x3) << 21 }
    pub const fn dst_burst(v: u32) -> u32 { (v & 0x1) << 23 }
    pub const fn dst_width(v: u32) -> u32 { (v & 0x3) << 24 }
    /// Continuous mode: restart automatically after each full transfer
    pub const CONTINUOUS: u32 = 1 << 29;
    /// Busy (read-only)
    pub const BUSY: u32 = 1 << 30;
    /// Load / start
    pub const LOAD: u32 = 1 << 31;
}

// ── Register layout (measured on hardware, Lichee Pi Nano) ──
//
// Sources disagree for channels other than N0 / D0:
// - xboot `dma-f1c100s.c`: channel stride 0x40, DDMA IRQ bits start at 16
// - Linux `sun4i-dma.c` (suniv): channel stride 0x20, DDMA IRQ bits start at 8
//
// Hardware probe (`docs/examples/dma_demo.rs`): the SRC registers at
// 0x124/0x164/0x324/0x364 hold written values while 0x184/0x1C4/0x384/0x3C4
// read back 0, so the stride is 0x20. D0 completion was observed on
// INT_STA bit 17, so DDMA IRQ bits start at 16 (see the probe's D1 check).

/// Distance between consecutive channel register blocks
const CH_STRIDE: u32 = 0x20;
/// First INT_CTL / INT_STA bit of DDMA channel D0
const DDMA_IRQ_BASE: u32 = 16;

/// Reset value of `PTY_CFG` used by xboot (vendor default)
const PTY_CFG_DEFAULT: u32 = 0x0000_0190;

/// Maximum byte count of a single NDMA hardware transfer
pub const NDMA_MAX_BYTES: u32 = 0x2_0000;
/// Maximum byte count of a single DDMA hardware transfer
pub const DDMA_MAX_BYTES: u32 = 0x100_0000;

const NDMA_PORT_SRAM: u32 = 16;
const NDMA_PORT_SDRAM: u32 = 17;
const DDMA_PORT_SRAM: u32 = 0;
const DDMA_PORT_SDRAM: u32 = 1;

/// Start of external DRAM; anything below is treated as on-chip SRAM
const DRAM_BASE: u32 = 0x8000_0000;

// ── MMIO Helpers ──────────────────────────────────────────────────────────

#[inline]
unsafe fn rd(offset: u32) -> u32 {
    ((DMA_BASE + offset) as *const u32).read_volatile()
}

#[inline]
unsafe fn wr(offset: u32, val: u32) {
    ((DMA_BASE + offset) as *mut u32).write_volatile(val);
}

// ── Public Types ──────────────────────────────────────────────────────────

/// DMA channel
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    N0 = 0,
    N1 = 1,
    N2 = 2,
    N3 = 3,
    D0 = 4,
    D1 = 5,
    D2 = 6,
    D3 = 7,
}

impl Channel {
    /// All channels, in index order
    pub const ALL: [Channel; 8] = [
        Channel::N0, Channel::N1, Channel::N2, Channel::N3,
        Channel::D0, Channel::D1, Channel::D2, Channel::D3,
    ];

    #[inline]
    pub const fn index(self) -> usize {
        self as usize
    }

    /// `true` for a dedicated (DDMA) channel
    #[inline]
    pub const fn is_dedicated(self) -> bool {
        (self as u32) >= 4
    }

    /// Largest byte count one hardware transfer on this channel can move
    #[inline]
    pub const fn max_bytes(self) -> u32 {
        if self.is_dedicated() { DDMA_MAX_BYTES } else { NDMA_MAX_BYTES }
    }

    const fn base(self) -> u32 {
        let i = self as u32;
        if i < 4 { 0x100 + i * CH_STRIDE } else { 0x300 + (i - 4) * CH_STRIDE }
    }

    /// Bit offset of this channel's (half, full) pair in INT_CTL / INT_STA
    const fn irq_shift(self) -> u32 {
        let i = self as u32;
        if i < 4 { i << 1 } else { DDMA_IRQ_BASE + ((i - 4) << 1) }
    }

    const fn irq_half(self) -> u32 { 0x1 << self.irq_shift() }
    const fn irq_full(self) -> u32 { 0x2 << self.irq_shift() }
    const fn irq_mask(self) -> u32 { 0x3 << self.irq_shift() }
}

/// Normal-DMA peripheral request lines
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NdmaDrq {
    Ir = 0,
    Owa = 1,
    Spi0 = 4,
    Spi1 = 5,
    Uart0 = 8,
    Uart1 = 9,
    Uart2 = 10,
    AudioCodec = 12,
    TouchPanel = 13,
    Daudio = 14,
    Usb = 20,
    UsbEp1 = 21,
    UsbEp2 = 22,
    UsbEp3 = 23,
}

/// Dedicated-DMA peripheral request lines
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DdmaDrq {
    Lcd = 2,
    Usb = 4,
    Ahb = 9,
}

/// DRQ port of one side of a transfer
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Port {
    /// SRAM or SDRAM — resolved from the address and the channel kind
    Memory,
    /// Peripheral request line, usable on N0–N3
    Ndma(NdmaDrq),
    /// Peripheral request line, usable on D0–D3
    Ddma(DdmaDrq),
}

/// Address behaviour of one side of a transfer
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddrMode {
    /// Address advances by `width` after each beat (memory buffers)
    Increment = 0,
    /// Address stays constant (peripheral FIFO / IO register)
    Fixed = 1,
}

/// Bus width of one side of a transfer
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Width {
    Bits8 = 0,
    Bits16 = 1,
    Bits32 = 2,
}

impl Width {
    #[inline]
    pub const fn bytes(self) -> u32 {
        1 << (self as u32)
    }
}

/// Burst length of one side of a transfer
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Burst {
    /// One beat per request
    Single = 0,
    /// Four beats per request
    Incr4 = 1,
}

/// One side (source or destination) of a DMA transfer
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Endpoint {
    pub addr: u32,
    pub port: Port,
    pub mode: AddrMode,
    pub width: Width,
    pub burst: Burst,
}

impl Endpoint {
    /// A memory buffer (SRAM or DRAM): incrementing, 32-bit, burst 4.
    pub const fn memory(addr: u32) -> Self {
        Self { addr, port: Port::Memory, mode: AddrMode::Increment, width: Width::Bits32, burst: Burst::Incr4 }
    }

    /// A peripheral register served by a normal-DMA request line:
    /// fixed address, 8-bit, single beat.
    pub const fn ndma(addr: u32, drq: NdmaDrq) -> Self {
        Self { addr, port: Port::Ndma(drq), mode: AddrMode::Fixed, width: Width::Bits8, burst: Burst::Single }
    }

    /// A peripheral register served by a dedicated-DMA request line:
    /// fixed address, 32-bit, single beat.
    pub const fn ddma(addr: u32, drq: DdmaDrq) -> Self {
        Self { addr, port: Port::Ddma(drq), mode: AddrMode::Fixed, width: Width::Bits32, burst: Burst::Single }
    }

    pub const fn width(mut self, width: Width) -> Self {
        self.width = width;
        self
    }

    pub const fn burst(mut self, burst: Burst) -> Self {
        self.burst = burst;
        self
    }

    pub const fn mode(mut self, mode: AddrMode) -> Self {
        self.mode = mode;
        self
    }

    fn drq(&self, dedicated: bool) -> Result<u32, Error> {
        match (self.port, dedicated) {
            (Port::Memory, false) => Ok(if self.addr >= DRAM_BASE { NDMA_PORT_SDRAM } else { NDMA_PORT_SRAM }),
            (Port::Memory, true) => Ok(if self.addr >= DRAM_BASE { DDMA_PORT_SDRAM } else { DDMA_PORT_SRAM }),
            (Port::Ndma(d), false) => Ok(d as u32),
            (Port::Ddma(d), true) => Ok(d as u32),
            _ => Err(Error::PortMismatch),
        }
    }

    #[inline]
    fn increments(&self) -> bool {
        self.mode == AddrMode::Increment
    }
}

/// Transfer mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Move `len` bytes once (split into chunks if needed), then stop
    OneShot,
    /// Loop over the same `len` bytes until [`stop`]. `len` must fit in a
    /// single hardware transfer. The half callback fires at the middle and
    /// the done callback at the end of every pass (ping-pong buffering).
    Cyclic,
}

/// Completion / half-way callback, called from interrupt context (or from
/// [`wait`] when it services the channel itself)
pub type Callback = fn(Channel);

/// A complete DMA transfer description
#[derive(Debug, Clone, Copy)]
pub struct Transfer {
    pub src: Endpoint,
    pub dst: Endpoint,
    /// Length in bytes
    pub len: u32,
    pub mode: Mode,
    pub on_half: Option<Callback>,
    pub on_done: Option<Callback>,
}

impl Transfer {
    /// One-shot transfer without callbacks
    pub const fn new(src: Endpoint, dst: Endpoint, len: u32) -> Self {
        Self { src, dst, len, mode: Mode::OneShot, on_half: None, on_done: None }
    }

    pub const fn cyclic(mut self) -> Self {
        self.mode = Mode::Cyclic;
        self
    }

    pub const fn on_half(mut self, cb: Callback) -> Self {
        self.on_half = Some(cb);
        self
    }

    pub const fn on_done(mut self, cb: Callback) -> Self {
        self.on_done = Some(cb);
        self
    }
}

/// DMA errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// [`init`] has not been called
    NotInitialized,
    /// The channel is already running a transfer
    Busy,
    /// Length is zero, or not a multiple of the endpoint widths
    InvalidLength,
    /// Cyclic transfer longer than the channel's hardware limit
    TooLong,
    /// A peripheral port does not belong to this channel kind
    /// (e.g. [`Port::Ndma`] on a D channel)
    PortMismatch,
    /// [`memcpy`] slices have different lengths
    LengthMismatch,
}

// ── Driver State ──────────────────────────────────────────────────────────

#[derive(Clone, Copy)]
struct ChState {
    active: bool,
    cyclic: bool,
    cfg: u32,
    src: u32,
    dst: u32,
    src_inc: bool,
    dst_inc: bool,
    total: u32,
    done: u32,
    chunk: u32,
    on_half: Option<Callback>,
    on_done: Option<Callback>,
}

impl ChState {
    const IDLE: ChState = ChState {
        active: false,
        cyclic: false,
        cfg: 0,
        src: 0,
        dst: 0,
        src_inc: false,
        dst_inc: false,
        total: 0,
        done: 0,
        chunk: 0,
        on_half: None,
        on_done: None,
    };
}

static mut STATE: [ChState; 8] = [ChState::IDLE; 8];
static mut INITIALIZED: bool = false;

#[inline]
fn critical<R>(f: impl FnOnce() -> R) -> R {
    let cpsr = cpu::interrupt_disable();
    let r = f();
    cpu::interrupt_enable(cpsr);
    r
}

// ── Public API ────────────────────────────────────────────────────────────

/// Initialise the DMA controller: enable its bus clock, pulse its reset,
/// clear every channel and install the interrupt handler.
pub fn init() {
    clock::bus_clk_init(clock::BusGate::Dma);

    unsafe {
        wr(reg::INT_CTL, 0);
        wr(reg::INT_STA, 0xFFFF_FFFF);
        wr(reg::PTY_CFG, PTY_CFG_DEFAULT);
        for c in Channel::ALL {
            let b = c.base();
            wr(b + ch::CFG, 0);
            wr(b + ch::SRC, 0);
            wr(b + ch::DST, 0);
            wr(b + ch::CNT, 0);
            if c.is_dedicated() {
                wr(b + ch::PAR, 0);
                wr(b + ch::GEN, 0);
            }
        }
        STATE = [ChState::IDLE; 8];
        addr_of_mut!(INITIALIZED).write_volatile(true);
    }

    interrupt::install(DMA_IRQ, dma_isr);
    interrupt::unmask(DMA_IRQ);
}

/// Start a transfer on `channel`.
///
/// Returns immediately; use [`wait`], [`is_busy`] or the transfer's
/// `on_done` callback to learn when it finishes.
///
/// # Safety
///
/// The hardware reads `len` bytes at `src.addr` and writes `len` bytes at
/// `dst.addr` behind the compiler's back. The caller must guarantee both
/// ranges are valid physical addresses, that the destination is not read
/// or written by the CPU and the source is not modified until the transfer
/// completes (or [`stop`] is called), and — for [`Mode::Cyclic`] — for as
/// long as the channel keeps running.
pub unsafe fn start(channel: Channel, t: &Transfer) -> Result<(), Error> {
    if !addr_of!(INITIALIZED).read_volatile() {
        return Err(Error::NotInitialized);
    }
    if t.len == 0
        || t.len % t.src.width.bytes() != 0
        || t.len % t.dst.width.bytes() != 0
    {
        return Err(Error::InvalidLength);
    }
    if t.mode == Mode::Cyclic && t.len > channel.max_bytes() {
        return Err(Error::TooLong);
    }

    let dedicated = channel.is_dedicated();
    let src_drq = t.src.drq(dedicated)?;
    let dst_drq = t.dst.drq(dedicated)?;

    let mut c = cfg::src_drq(src_drq)
        | cfg::src_mode(t.src.mode as u32)
        | cfg::src_burst(t.src.burst as u32)
        | cfg::src_width(t.src.width as u32)
        | cfg::dst_drq(dst_drq)
        | cfg::dst_mode(t.dst.mode as u32)
        | cfg::dst_burst(t.dst.burst as u32)
        | cfg::dst_width(t.dst.width as u32);
    if t.mode == Mode::Cyclic {
        c |= cfg::CONTINUOUS;
    }

    // Cache maintenance before the hardware touches memory
    if t.src.port == Port::Memory {
        let span = if t.src.increments() { t.len } else { t.src.width.bytes() };
        mmu::clean_dcache(t.src.addr, span);
    }
    if t.dst.port == Port::Memory {
        let span = if t.dst.increments() { t.len } else { t.dst.width.bytes() };
        mmu::clean_invalidate_dcache(t.dst.addr, span);
    }

    critical(|| {
        let st = &mut *addr_of_mut!(STATE[channel.index()]);
        if st.active || rd(channel.base() + ch::CFG) & (cfg::BUSY | cfg::LOAD) != 0 {
            return Err(Error::Busy);
        }
        *st = ChState {
            active: true,
            cyclic: t.mode == Mode::Cyclic,
            cfg: c,
            src: t.src.addr,
            dst: t.dst.addr,
            src_inc: t.src.increments(),
            dst_inc: t.dst.increments(),
            total: t.len,
            done: 0,
            chunk: 0,
            on_half: t.on_half,
            on_done: t.on_done,
        };

        wr(reg::INT_STA, channel.irq_mask());
        let mut en = channel.irq_full();
        if st.cyclic && st.on_half.is_some() {
            en |= channel.irq_half();
        }
        wr(reg::INT_CTL, (rd(reg::INT_CTL) & !channel.irq_mask()) | en);

        program_chunk(channel, st);
        Ok(())
    })
}

/// Abort a running transfer (or end a cyclic one). No callback is called.
pub fn stop(channel: Channel) {
    critical(|| unsafe {
        let b = channel.base();
        wr(b + ch::CFG, rd(b + ch::CFG) & !(cfg::LOAD | cfg::CONTINUOUS));
        wr(reg::INT_CTL, rd(reg::INT_CTL) & !channel.irq_mask());
        wr(reg::INT_STA, channel.irq_mask());
        (*addr_of_mut!(STATE[channel.index()])).active = false;
    });
}

/// `true` while a transfer started on `channel` has not completed
#[inline]
pub fn is_busy(channel: Channel) -> bool {
    unsafe { addr_of!(STATE[channel.index()].active).read_volatile() }
}

/// Block until the transfer on `channel` completes.
///
/// Works with interrupts enabled or disabled: the channel's status bits are
/// serviced here too, so multi-chunk transfers progress either way. For a
/// cyclic transfer this only returns after [`stop`].
pub fn wait(channel: Channel) {
    while is_busy(channel) {
        critical(|| unsafe {
            let pending = rd(reg::INT_STA) & channel.irq_mask();
            if pending != 0 {
                wr(reg::INT_STA, pending);
                service(channel, pending);
            }
        });
    }
}

/// Bytes already moved by the current / last one-shot transfer
/// (whole chunks only).
pub fn transferred(channel: Channel) -> u32 {
    unsafe { addr_of!(STATE[channel.index()].done).read_volatile() }
}

/// Blocking memory-to-memory copy `src → dst` through `channel`.
///
/// Uses 32-bit bursts when both buffers and the length are word-aligned,
/// byte transfers otherwise. A dedicated channel (D0–D3) is preferred for
/// large copies since it moves up to 16 MiB per hardware transfer.
pub fn memcpy(channel: Channel, dst: &mut [u8], src: &[u8]) -> Result<(), Error> {
    if dst.len() != src.len() {
        return Err(Error::LengthMismatch);
    }
    let s = src.as_ptr() as u32;
    let d = dst.as_mut_ptr() as u32;
    let len = src.len() as u32;

    let (w, b) = if (s | d | len) & 3 == 0 {
        (Width::Bits32, Burst::Incr4)
    } else {
        (Width::Bits8, Burst::Single)
    };
    let t = Transfer::new(
        Endpoint::memory(s).width(w).burst(b),
        Endpoint::memory(d).width(w).burst(b),
        len,
    );
    // SAFETY: both slices are borrowed for the whole call and we block
    // until the hardware is done.
    unsafe { start(channel, &t)? };
    wait(channel);
    Ok(())
}

// ── Internals ─────────────────────────────────────────────────────────────

/// Program and launch the next chunk of `st` on `channel`.
/// Must be called with interrupts disabled and the channel idle.
unsafe fn program_chunk(channel: Channel, st: &mut ChState) {
    let b = channel.base();
    let chunk = core::cmp::min(channel.max_bytes(), st.total - st.done);
    st.chunk = chunk;
    wr(b + ch::SRC, st.src + if st.src_inc { st.done } else { 0 });
    wr(b + ch::DST, st.dst + if st.dst_inc { st.done } else { 0 });
    wr(b + ch::CNT, chunk);
    wr(b + ch::CFG, st.cfg);
    cpu::dsb();
    wr(b + ch::CFG, st.cfg | cfg::LOAD);
}

/// Handle the `pending` (already acknowledged) status bits of `channel`.
/// Must be called with interrupts disabled.
unsafe fn service(channel: Channel, pending: u32) {
    let st = &mut *addr_of_mut!(STATE[channel.index()]);
    if !st.active {
        return;
    }

    if pending & channel.irq_half() != 0 {
        if let Some(cb) = st.on_half {
            cb(channel);
        }
    }

    if pending & channel.irq_full() == 0 {
        return;
    }

    if st.cyclic {
        // Hardware restarts by itself; just report the finished pass.
        if let Some(cb) = st.on_done {
            cb(channel);
        }
        return;
    }

    st.done += st.chunk;
    if st.done < st.total {
        program_chunk(channel, st);
    } else {
        wr(reg::INT_CTL, rd(reg::INT_CTL) & !channel.irq_mask());
        st.active = false;
        if let Some(cb) = st.on_done {
            cb(channel);
        }
    }
}

/// DMA controller interrupt handler (INTC vector 18)
fn dma_isr(_vector: u32) {
    critical(|| unsafe {
        let pending = rd(reg::INT_STA);
        if pending == 0 {
            return;
        }
        wr(reg::INT_STA, pending);
        for c in Channel::ALL {
            let p = pending & c.irq_mask();
            if p != 0 {
                service(c, p);
            }
        }
    });
}
