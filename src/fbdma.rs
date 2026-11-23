//! DMA-accelerated framebuffer operations
//!
//! On parallel/serial RGB panels the display engine (DEBE) already scans the
//! framebuffer out of DRAM with its own bus master, so there is nothing to
//! "DMA to the LCD". What costs CPU time is *drawing*: clearing the screen,
//! filling rectangles, copying the previous frame into the back buffer,
//! blitting sprites or scrolling. This module offloads those to the DMA
//! controller (a dedicated channel D0–D3 is recommended: 16 MiB per
//! hardware transfer).
//!
//! - [`fill_rect`] / [`fill_rect_async`]: fill a rectangle with one colour
//!   (the source is a single colour word read at a fixed address)
//! - [`blit`] / [`blit_async`]: copy a rectangle between surfaces, or inside
//!   one surface (vertical overlap such as scrolling is handled by copying
//!   rows bottom-up when needed)
//!
//! A rectangle that spans whole, gap-less rows is moved as one transfer.
//! Otherwise rows are chained from the DMA completion callback, so the
//! `_async` variants return immediately and the CPU stays free; [`wait`]
//! (also used by the blocking variants) advances the chain even with
//! interrupts disabled.
//!
//! Cache: destination rows are cleaned+invalidated and source rows cleaned
//! before each row is moved (done by `dma::start`). While an operation is
//! running, the CPU must not draw into the destination rectangle or into
//! pixels that share a 32-byte cache line with its edges.
//!
//! ```ignore
//! let fb = unsafe { fbdma::Surface::new(FB, 480, 480, lcd::ColorMode::Argb8888) };
//! fbdma::fill_rect(Channel::D0, &fb, fbdma::Rect::new(0, 0, 480, 480), 0xFF00_0000)?; // clear
//! fbdma::blit(Channel::D0, &fb, 10, 10, &sprite, sprite.rect())?;
//! ```

use core::ptr::{addr_of, addr_of_mut};

use crate::dma::{self, AddrMode, Burst, Channel, Endpoint, Transfer, Width};
use crate::lcd::ColorMode;

// ── Public types ──────────────────────────────────────────────────────────

/// A framebuffer (or any 2D pixel buffer) in memory
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Surface {
    /// Address of pixel (0, 0)
    pub addr: u32,
    pub width: u32,
    pub height: u32,
    /// Bytes from one row to the next
    pub stride: u32,
    /// Bytes per pixel: 2 (RGB565) or 4 (XRGB/ARGB8888)
    pub bytes_pp: u32,
}

impl Surface {
    /// Describe a tightly packed `width × height` buffer at `fb`.
    ///
    /// # Safety
    /// `fb` must point to at least `width * height * bpp` bytes that stay
    /// valid for every operation using this surface.
    pub unsafe fn new(fb: *const u8, width: u32, height: u32, mode: ColorMode) -> Self {
        let bytes_pp = mode.bpp() / 8;
        Self { addr: fb as u32, width, height, stride: width * bytes_pp, bytes_pp }
    }

    /// Same surface with a custom row stride (in bytes)
    pub const fn with_stride(mut self, stride: u32) -> Self {
        self.stride = stride;
        self
    }

    /// The whole surface as a rectangle
    pub const fn rect(&self) -> Rect {
        Rect { x: 0, y: 0, w: self.width, h: self.height }
    }

    fn pixel_addr(&self, x: u32, y: u32) -> u32 {
        self.addr + y * self.stride + x * self.bytes_pp
    }

    fn contains(&self, r: &Rect) -> bool {
        r.x.checked_add(r.w).map_or(false, |e| e <= self.width)
            && r.y.checked_add(r.h).map_or(false, |e| e <= self.height)
    }

    /// Rows `r` occupies are back to back in memory (full width, no gap)
    fn rows_contiguous(&self, r: &Rect) -> bool {
        r.x == 0 && r.w * self.bytes_pp == self.stride
    }

    fn byte_range(&self, r: &Rect) -> (u32, u32) {
        let start = self.pixel_addr(r.x, r.y);
        let end = self.pixel_addr(r.x + r.w, r.y + r.h - 1);
        (start, end)
    }
}

/// Rectangle in pixels
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rect {
    pub x: u32,
    pub y: u32,
    pub w: u32,
    pub h: u32,
}

impl Rect {
    pub const fn new(x: u32, y: u32, w: u32, h: u32) -> Self {
        Self { x, y, w, h }
    }
}

/// Framebuffer DMA errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Starting a DMA transfer failed (e.g. `dma::init` not called)
    Dma(dma::Error),
    /// The rectangle does not fit in its surface
    OutOfBounds,
    /// Source and destination have different bytes per pixel
    FormatMismatch,
    /// Same-row overlapping copy to the right; cannot be done with forward DMA
    Overlap,
    /// An operation is already running on this channel
    Busy,
    /// Surface stride or address breaks 16-bit alignment
    Unaligned,
}

// ── Per-channel job state ─────────────────────────────────────────────────

#[derive(Clone, Copy)]
struct Job {
    active: bool,
    src: u32,
    src_fixed: bool,
    src_step: i32,
    dst: u32,
    dst_step: i32,
    row_bytes: u32,
    rows_left: u32,
    width: Width,
    burst: Burst,
    error: Option<dma::Error>,
    on_done: Option<dma::Callback>,
}

impl Job {
    const IDLE: Job = Job {
        active: false,
        src: 0,
        src_fixed: false,
        src_step: 0,
        dst: 0,
        dst_step: 0,
        row_bytes: 0,
        rows_left: 0,
        width: Width::Bits32,
        burst: Burst::Incr4,
        error: None,
        on_done: None,
    };
}

static mut JOBS: [Job; 8] = [Job::IDLE; 8];

/// Fill colour words, one cache line per channel so a clean of one never
/// races with another channel's word.
#[repr(C, align(32))]
struct ColorLine([u32; 8]);
static mut FILL_COLOR: [ColorLine; 8] = [const { ColorLine([0; 8]) }; 8];

fn job(ch: Channel) -> *mut Job {
    unsafe { addr_of_mut!(JOBS[ch.index()]) }
}

/// `true` while an operation started on `ch` is still running
pub fn is_busy(ch: Channel) -> bool {
    unsafe { addr_of!((*job(ch)).active).read_volatile() }
}

/// Block until the operation on `ch` finishes; returns its outcome.
pub fn wait(ch: Channel) -> Result<(), Error> {
    while is_busy(ch) {
        dma::poll(ch);
    }
    match unsafe { addr_of!((*job(ch)).error).read_volatile() } {
        Some(e) => Err(Error::Dma(e)),
        None => Ok(()),
    }
}

// ── Engine ────────────────────────────────────────────────────────────────

/// Pick the widest beat the addresses, steps and row length allow.
fn beat(addrs: &[u32], steps: &[i32], row_bytes: u32) -> Result<(Width, Burst), Error> {
    let mut or = row_bytes;
    for a in addrs { or |= *a; }
    for s in steps { or |= *s as u32; }
    if or & 3 == 0 {
        Ok((Width::Bits32, Burst::Incr4))
    } else if or & 1 == 0 {
        Ok((Width::Bits16, Burst::Single))
    } else {
        Err(Error::Unaligned)
    }
}

/// Start the row described by `j` on `ch`.
unsafe fn start_row(ch: Channel, j: &Job) -> Result<(), dma::Error> {
    let src = Endpoint::memory(j.src)
        .width(j.width)
        .burst(j.burst)
        .mode(if j.src_fixed { AddrMode::Fixed } else { AddrMode::Increment });
    let dst = Endpoint::memory(j.dst).width(j.width).burst(j.burst);
    dma::start(ch, &Transfer::new(src, dst, j.row_bytes).on_done(row_done))
}

/// DMA completion callback: chain the next row or finish the job.
fn row_done(ch: Channel) {
    unsafe {
        let j = &mut *job(ch);
        if !j.active {
            return;
        }
        j.rows_left -= 1;
        if j.rows_left > 0 {
            if !j.src_fixed {
                j.src = j.src.wrapping_add(j.src_step as u32);
            }
            j.dst = j.dst.wrapping_add(j.dst_step as u32);
            match start_row(ch, j) {
                Ok(()) => return,
                Err(e) => j.error = Some(e),
            }
        }
        j.active = false;
        if let Some(cb) = j.on_done {
            cb(ch);
        }
    }
}

/// Install `j` on `ch` and start its first row.
unsafe fn launch(ch: Channel, mut j: Job) -> Result<(), Error> {
    let slot = &mut *job(ch);
    if slot.active || dma::is_busy(ch) {
        return Err(Error::Busy);
    }
    j.active = true;
    j.error = None;
    *slot = j;
    if let Err(e) = start_row(ch, slot) {
        slot.active = false;
        return Err(Error::Dma(e));
    }
    Ok(())
}

// ── Fill ──────────────────────────────────────────────────────────────────

/// Start filling `rect` of `dst` with `color` and return immediately.
///
/// `color` is the raw pixel value: `0xAARRGGBB` for 32 bpp, the low 16 bits
/// (RGB565) for 16 bpp. `on_done` runs (in interrupt context, or inside
/// [`wait`]) when the whole rectangle is written.
///
/// # Safety
/// `dst` must stay valid, and the CPU must not touch the rectangle (or
/// pixels sharing its edge cache lines) until the operation completes.
pub unsafe fn fill_rect_async(
    ch: Channel,
    dst: &Surface,
    rect: Rect,
    color: u32,
    on_done: Option<dma::Callback>,
) -> Result<(), Error> {
    if !dst.contains(&rect) {
        return Err(Error::OutOfBounds);
    }
    if rect.w == 0 || rect.h == 0 {
        if let Some(cb) = on_done { cb(ch); }
        return Ok(());
    }
    if is_busy(ch) || dma::is_busy(ch) {
        return Err(Error::Busy);
    }

    let word = match dst.bytes_pp {
        2 => (color & 0xFFFF) | (color << 16),
        _ => color,
    };
    let color_addr = {
        let line = &mut *addr_of_mut!(FILL_COLOR[ch.index()]);
        // Fill the whole line: with an Incr4 burst the controller reads four
        // consecutive words even when the source is set to Fixed.
        line.0 = [word; 8];
        line.0.as_ptr() as u32
    };

    let (row_bytes, rows) = if dst.rows_contiguous(&rect) {
        (rect.w * dst.bytes_pp * rect.h, 1)
    } else {
        (rect.w * dst.bytes_pp, rect.h)
    };
    let first = dst.pixel_addr(rect.x, rect.y);
    let (width, burst) = beat(&[first], &[dst.stride as i32], row_bytes)?;

    launch(ch, Job {
        src: color_addr,
        src_fixed: true,
        src_step: 0,
        dst: first,
        dst_step: dst.stride as i32,
        row_bytes,
        rows_left: rows,
        width,
        burst,
        on_done,
        ..Job::IDLE
    })
}

/// Fill `rect` of `dst` with `color` and wait for it to finish.
pub fn fill_rect(ch: Channel, dst: &Surface, rect: Rect, color: u32) -> Result<(), Error> {
    unsafe { fill_rect_async(ch, dst, rect, color, None)? };
    wait(ch)
}

// ── Blit ──────────────────────────────────────────────────────────────────

/// Start copying `src_rect` of `src` to (`dx`, `dy`) of `dst` and return
/// immediately. `src` and `dst` may be the same surface: vertically
/// overlapping copies (scrolling) are handled; a same-row copy to the right
/// over itself returns [`Error::Overlap`].
///
/// # Safety
/// Both surfaces must stay valid; the CPU must not modify the source or
/// touch the destination rectangle until the operation completes.
pub unsafe fn blit_async(
    ch: Channel,
    dst: &Surface,
    dx: u32,
    dy: u32,
    src: &Surface,
    src_rect: Rect,
    on_done: Option<dma::Callback>,
) -> Result<(), Error> {
    let dst_rect = Rect::new(dx, dy, src_rect.w, src_rect.h);
    if !src.contains(&src_rect) || !dst.contains(&dst_rect) {
        return Err(Error::OutOfBounds);
    }
    if src.bytes_pp != dst.bytes_pp {
        return Err(Error::FormatMismatch);
    }
    if src_rect.w == 0 || src_rect.h == 0 {
        if let Some(cb) = on_done { cb(ch); }
        return Ok(());
    }
    if is_busy(ch) || dma::is_busy(ch) {
        return Err(Error::Busy);
    }

    let (s0, s1) = src.byte_range(&src_rect);
    let (d0, d1) = dst.byte_range(&dst_rect);
    let overlap = s0 < d1 && d0 < s1;

    // Overlapping copy with the destination below the source: go bottom-up.
    let bottom_up = overlap && d0 > s0;
    if overlap && bottom_up && dy == src_rect.y && dst.stride == src.stride {
        // Same rows, moving right: every row overlaps itself.
        return Err(Error::Overlap);
    }

    let contiguous = !overlap && src.rows_contiguous(&src_rect) && dst.rows_contiguous(&dst_rect);
    let (row_bytes, rows) = if contiguous {
        (src_rect.w * src.bytes_pp * src_rect.h, 1)
    } else {
        (src_rect.w * src.bytes_pp, src_rect.h)
    };

    let (src_first, dst_first, src_step, dst_step) = if bottom_up {
        let last = src_rect.h - 1;
        (
            src.pixel_addr(src_rect.x, src_rect.y + last),
            dst.pixel_addr(dx, dy + last),
            -(src.stride as i32),
            -(dst.stride as i32),
        )
    } else {
        (
            src.pixel_addr(src_rect.x, src_rect.y),
            dst.pixel_addr(dx, dy),
            src.stride as i32,
            dst.stride as i32,
        )
    };
    let (width, burst) = beat(&[src_first, dst_first], &[src_step, dst_step], row_bytes)?;

    launch(ch, Job {
        src: src_first,
        src_fixed: false,
        src_step,
        dst: dst_first,
        dst_step,
        row_bytes,
        rows_left: rows,
        width,
        burst,
        on_done,
        ..Job::IDLE
    })
}

/// Copy `src_rect` of `src` to (`dx`, `dy`) of `dst` and wait for it.
pub fn blit(
    ch: Channel,
    dst: &Surface,
    dx: u32,
    dy: u32,
    src: &Surface,
    src_rect: Rect,
) -> Result<(), Error> {
    unsafe { blit_async(ch, dst, dx, dy, src, src_rect, None)? };
    wait(ch)
}
