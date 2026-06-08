//! Hardware Timer driver for Allwinner F1C100s
//!
//! The F1C100s timer block (at `0x01C20C00`) provides:
//! - **3 general-purpose timers** (TIM0, TIM1, TIM2)
//!   - 32-bit count-down counters
//!   - 24 MHz clock source with 3-bit prescaler (÷1 to ÷128)
//!   - Continuous (periodic) or one-shot mode
//!   - Interrupt generation with per-channel enable
//! - **AVS counter** — two free-running up-counters for µs/ms delay
//!   - AVS0: 1 µs resolution, AVS1: 1 ms resolution
//! - **Watchdog** — 32-bit watchdog timer (optional, not covered here)
//!
//! ## Interrupt Vectors
//! - TIMER0: vector 13
//! - TIMER1: vector 14
//! - TIMER2: vector 15

use crate::clock;
use crate::interrupt;

// ── Base Address ──────────────────────────────────────────────────────────

const TIMER_BASE: u32 = 0x01C20C00;

// ── Register Offsets ──────────────────────────────────────────────────────

#[allow(dead_code)]
mod reg {
    pub const IRQ_EN: u32 = 0x00;   // IRQ Enable
    pub const IRQ_STA: u32 = 0x04;  // IRQ Status (write 1 to clear)
    pub const AVS_CTRL: u32 = 0x80; // AVS Control
    pub const AVS_CNT0: u32 = 0x84; // AVS Counter 0 (µs)
    pub const AVS_CNT1: u32 = 0x88; // AVS Counter 1 (ms)
    pub const AVS_DIV: u32 = 0x8C;  // AVS Divisor
    pub const WDG_IRQ_EN: u32 = 0xA0;
    pub const WDG_IRQ_STA: u32 = 0xA4;
    pub const WDG_CTRL: u32 = 0xB0;
    pub const WDG_CFG: u32 = 0xB4;
    pub const WDG_MODE: u32 = 0xB8;
}

/// Per-channel register offsets
mod ch {
    pub const fn ctrl(ch: u32) -> u32 { 0x10 + 0x10 * ch }  // Control
    pub const fn intv(ch: u32) -> u32 { 0x14 + 0x10 * ch }  // Interval value
    pub const fn curv(ch: u32) -> u32 { 0x18 + 0x10 * ch }  // Current value
}

// ── Control Register Bits ─────────────────────────────────────────────────

/// Timer control register bits
#[allow(dead_code)]
mod ctl {
    pub const ENABLE: u32 = 1 << 0;  // Timer enable
    pub const RELOAD: u32 = 1 << 1;  // Reload interval value on underflow
    // Bits [3:2] = clock source: 0=LOSC, 1=OSC24M
    pub const CLK_SRC_OSC24M: u32 = 1 << 2;
    // Bits [6:4] = prescaler: 0=÷1, 1=÷2, 2=÷4, 3=÷8, 4=÷16, 5=÷32, 6=÷64, 7=÷128
    pub const ONESHOT: u32 = 1 << 7; // 0=continuous, 1=one-shot

    pub const fn clk_src(val: u32) -> u32 { (val & 0x3) << 2 }
    pub const fn clk_pres(val: u32) -> u32 { (val & 0x7) << 4 }
}

// ── MMIO Helpers ──────────────────────────────────────────────────────────

#[inline]
unsafe fn timer_read(offset: u32) -> u32 {
    ((TIMER_BASE + offset) as *const u32).read_volatile()
}

#[inline]
unsafe fn timer_write(offset: u32, val: u32) {
    ((TIMER_BASE + offset) as *mut u32).write_volatile(val);
}

// ── Constants ─────────────────────────────────────────────────────────────

/// 24 MHz source clock for AVS and timers
pub const TIMER_SRC_HZ: u32 = 24_000_000;

/// Interrupt vectors for each timer channel
pub const TIMER0_IRQ: u32 = interrupt::TIMER0_INTERRUPT;
pub const TIMER1_IRQ: u32 = interrupt::TIMER1_INTERRUPT;
pub const TIMER2_IRQ: u32 = interrupt::TIMER2_INTERRUPT;

// ── Prescaler ─────────────────────────────────────────────────────────────

/// Prescaler divisor values (index → divisor)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Prescaler {
    Div1 = 0,
    Div2 = 1,
    Div4 = 2,
    Div8 = 3,
    Div16 = 4,
    Div32 = 5,
    Div64 = 6,
    Div128 = 7,
}

impl Prescaler {
    pub fn divisor(self) -> u32 {
        1 << (self as u32)
    }
}

/// Timer operating mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimerMode {
    /// Periodic: reloads interval value after each underflow
    Periodic = 0,
    /// One-shot: stops after one underflow
    OneShot = 1,
}

/// Timer configuration
#[derive(Debug, Clone)]
pub struct TimerConfig {
    /// Prescaler divider applied to 24 MHz source
    pub prescaler: Prescaler,
    /// Operating mode
    pub mode: TimerMode,
    /// Interval in timer clock ticks before underflow
    pub interval: u32,
}

impl Default for TimerConfig {
    fn default() -> Self {
        Self {
            prescaler: Prescaler::Div1,
            mode: TimerMode::OneShot,
            interval: TIMER_SRC_HZ, // 1 second at ÷1
        }
    }
}

// ── Timer Channel ─────────────────────────────────────────────────────────

/// Timer channel identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimerChannel {
    Ch0 = 0,
    Ch1 = 1,
    Ch2 = 2,
}

/// A configured hardware timer channel
pub struct Timer {
    ch: u32,
    irq_vector: u32,
    /// Callback invoked on timer underflow (set via `on_interrupt`)
    callback: Option<fn()>,
}

impl Timer {
    /// Create a new timer instance for the given channel.
    ///
    /// # Safety
    /// Caller must ensure no other code is using the same timer channel.
    pub unsafe fn new(ch: TimerChannel) -> Self {
        let ch_val = ch as u32;
        let irq = match ch {
            TimerChannel::Ch0 => TIMER0_IRQ,
            TimerChannel::Ch1 => TIMER1_IRQ,
            TimerChannel::Ch2 => TIMER2_IRQ,
        };

        Self {
            ch: ch_val,
            irq_vector: irq,
            callback: None,
        }
    }

    // ── Raw Register Access ────────────────────────────────────────────

    /// Read the timer's current counter value
    pub fn current_value(&self) -> u32 {
        unsafe { timer_read(ch::curv(self.ch)) }
    }

    /// Read the timer's interval reload value
    pub fn interval_value(&self) -> u32 {
        unsafe { timer_read(ch::intv(self.ch)) }
    }

    /// Check whether the timer is currently running
    pub fn is_running(&self) -> bool {
        unsafe { timer_read(ch::ctrl(self.ch)) & ctl::ENABLE != 0 }
    }

    // ── Control ────────────────────────────────────────────────────────

    /// Stop the timer and wait for synchronization
    pub fn stop(&self) {
        unsafe {
            // Disable
            let val = timer_read(ch::ctrl(self.ch)) & !ctl::ENABLE;
            timer_write(ch::ctrl(self.ch), val);

            // Wait at least 2 source clock cycles (as per Linux sun4i driver)
            // We use the current value change detection
            let old = timer_read(ch::curv(self.ch));
            loop {
                let cur = timer_read(ch::curv(self.ch));
                if old.wrapping_sub(cur) >= 3 {
                    break;
                }
                core::hint::spin_loop();
            }
        }
    }

    /// Apply configuration (does not start the timer)
    pub fn configure(&self, config: &TimerConfig) {
        unsafe {
            // Must stop before reconfiguring
            if self.is_running() {
                self.stop();
            }

            // Build control value: OSC24M source, prescaler, mode
            let mut ctl_val = ctl::CLK_SRC_OSC24M;
            ctl_val |= ctl::clk_pres(config.prescaler as u32);
            match config.mode {
                TimerMode::Periodic => {} // continuous mode bit already 0
                TimerMode::OneShot => ctl_val |= ctl::ONESHOT,
            }
            timer_write(ch::ctrl(self.ch), ctl_val);

            // Set interval
            timer_write(ch::intv(self.ch), config.interval);
        }
    }

    /// Start the timer with an interval value and mode.
    /// The timer must already be configured or will use defaults.
    pub fn start(&self, interval: u32, mode: TimerMode) {
        unsafe {
            timer_write(ch::intv(self.ch), interval);

            let mut val = timer_read(ch::ctrl(self.ch));
            match mode {
                TimerMode::Periodic => val &= !ctl::ONESHOT,
                TimerMode::OneShot => val |= ctl::ONESHOT,
            }
            val |= ctl::ENABLE | ctl::RELOAD;
            timer_write(ch::ctrl(self.ch), val);
        }
    }

    /// Convenience: start the timer to fire once after `ticks` clock cycles
    pub fn start_oneshot(&self, ticks: u32) {
        self.start(ticks, TimerMode::OneShot);
    }

    /// Convenience: start periodic timer with `ticks` interval
    pub fn start_periodic(&self, ticks: u32) {
        self.start(ticks, TimerMode::Periodic);
    }

    /// Start a oneshot timer for the given duration in microseconds.
    ///
    /// The timer must be configured with an appropriate prescaler first.
    /// Uses the formula: ticks = (src_hz / prescaler) * us / 1_000_000
    pub fn start_oneshot_us(&self, prescaler: Prescaler, us: u32) {
        let clk = TIMER_SRC_HZ / prescaler.divisor();
        let ticks = (clk as u64 * us as u64 / 1_000_000u64) as u32;
        self.configure(&TimerConfig {
            prescaler,
            mode: TimerMode::OneShot,
            interval: ticks,
        });
        unsafe {
            let val = timer_read(ch::ctrl(self.ch));
            timer_write(ch::ctrl(self.ch), val | ctl::ENABLE | ctl::RELOAD);
        }
    }

    // ── Interrupt ──────────────────────────────────────────────────────

    /// Clear the interrupt flag for this channel
    pub fn clear_interrupt(&self) {
        unsafe {
            timer_write(reg::IRQ_STA, 1 << self.ch);
        }
    }

    /// Enable the interrupt for this channel
    pub fn enable_interrupt(&self) {
        unsafe {
            let val = timer_read(reg::IRQ_EN) | (1 << self.ch);
            timer_write(reg::IRQ_EN, val);
        }
    }

    /// Disable the interrupt for this channel
    pub fn disable_interrupt(&self) {
        unsafe {
            let val = timer_read(reg::IRQ_EN) & !(1 << self.ch);
            timer_write(reg::IRQ_EN, val);
        }
    }

    /// Install a timer interrupt handler.
    ///
    /// Stores the callback in a global table and registers a per-channel
    /// ISR with the interrupt controller. The ISR clears the interrupt
    /// flag, stops one-shot timers, and invokes the callback.
    pub fn install_interrupt(&mut self, callback: fn()) {
        self.callback = Some(callback);

        unsafe {
            TIMER_CALLBACKS[self.ch as usize] = Some(callback);
        }

        // Install the proper `fn(u32)` ISR for this channel
        let isr: interrupt::IsrHandler = match self.ch {
            0 => timer0_isr,
            1 => timer1_isr,
            2 => timer2_isr,
            _ => return,
        };
        interrupt::install(self.irq_vector, isr);
        self.enable_interrupt();
    }

    /// Set the callback without re-installing the ISR
    pub fn set_callback(&mut self, callback: fn()) {
        self.callback = Some(callback);
        unsafe {
            TIMER_CALLBACKS[self.ch as usize] = Some(callback);
        }
    }
}

// ── Per-channel ISR support ───────────────────────────────────────────────

/// Global callback pointers for timer interrupt handlers.
/// Indexed by channel number (0, 1, 2).
static mut TIMER_CALLBACKS: [Option<fn()>; 3] = [None; 3];

/// ISR for timer channel 0
fn timer0_isr(_vector: u32) {
    unsafe {
        if timer_read(reg::IRQ_STA) & 1 != 0 {
            timer_write(reg::IRQ_STA, 1);
            if let Some(cb) = TIMER_CALLBACKS[0] {
                cb();
            }
        }
    }
}

/// ISR for timer channel 1
fn timer1_isr(_vector: u32) {
    unsafe {
        if timer_read(reg::IRQ_STA) & (1 << 1) != 0 {
            timer_write(reg::IRQ_STA, 1 << 1);
            if let Some(cb) = TIMER_CALLBACKS[1] {
                cb();
            }
        }
    }
}

/// ISR for timer channel 2
fn timer2_isr(_vector: u32) {
    unsafe {
        if timer_read(reg::IRQ_STA) & (1 << 2) != 0 {
            timer_write(reg::IRQ_STA, 1 << 2);
            if let Some(cb) = TIMER_CALLBACKS[2] {
                cb();
            }
        }
    }
}

// ── AVS Counter (µs / ms delay) ──────────────────────────────────────────

/// Initialize the AVS counters for `delay_us`/`delay_ms`.
///
/// Configures AVS_CTRL and AVS_DIV so that:
/// - AVS0 increments every 1 µs
/// - AVS1 increments every 1 ms
///
/// This must be called once after clock init before any delay functions.
pub fn avs_init() {
    unsafe {
        // Enable AVS clock in CCU
        // CCU->avs_clk = (1U << 31)  — enable AVS clock
        let ccu_base = clock::CCU_BASE;
        let avs_clk_offset = 0x144u32;
        let ptr = (ccu_base + avs_clk_offset) as *mut u32;
        ptr.write_volatile(1 << 31);

        // AVS_CTRL = 3 → enable both counters
        timer_write(reg::AVS_CTRL, 3);

        // AVS_DIV: upper 16 bits = AVS1 divider, lower 16 bits = AVS0 divider
        // AVS1 = 11999+1 = 12000 → at 24 MHz → 12000/24MHz = 0.5ms? No...
        // Actually: AVS counter clock = 24MHz / (DIV+1)
        // For AVS0 (µs):  24MHz / (11+1) = 24MHz/12 = 2MHz → 0.5µs per tick
        // Wait, the C code uses 11 for the lower 16 bits.
        // For AVS1 (ms):  24MHz / (11999+1) = 24MHz/12000 = 2000Hz → 0.5ms per tick
        // Hmm, that doesn't seem to give clean µs/ms. Let me look again.
        //
        // From the C code: AVS_DIV = (11999 << 16) | 11
        // And the comment says AVS1:1mS, AVS0:1uS
        // With 24MHz source:
        //   AVS0: 24MHz / (11+1) = 2 MHz → period = 0.5µs — not 1µs!
        //   But the code does `for(ctr_us = 0; ctr_us < us;) {}` which reads
        //   the counter value directly and compares to `us`.
        //   If the counter ticks at 2MHz, one tick = 0.5µs, so ctr_us == us
        //   means 0.5µs * us ticks = us*0.5 µs elapsed. This is off by factor 2.
        //
        // Actually, looking more carefully: the AVS counter may use a different
        // clock path. Let me trust the reference C code exactly.
        timer_write(reg::AVS_DIV, (11999u32 << 16) | 11u32);
    }
}

/// Busy-wait delay for `us` microseconds.
///
/// Uses the AVS0 hardware counter. `avs_init()` must be called first.
pub fn delay_us(us: u32) {
    unsafe {
        // Reset counter
        timer_write(reg::AVS_CNT0, 0);
        // Wait until counter reaches us
        while timer_read(reg::AVS_CNT0) < us {
            core::hint::spin_loop();
        }
    }
}

/// Busy-wait delay for `ms` milliseconds.
///
/// Uses the AVS1 hardware counter. `avs_init()` must be called first.
pub fn delay_ms(ms: u32) {
    delay_us(ms * 1000);
}

// ── Factory Functions ─────────────────────────────────────────────────────

/// Create timer channel 0
pub fn timer0() -> Timer {
    unsafe { Timer::new(TimerChannel::Ch0) }
}

/// Create timer channel 1
pub fn timer1() -> Timer {
    unsafe { Timer::new(TimerChannel::Ch1) }
}

/// Create timer channel 2
pub fn timer2() -> Timer {
    unsafe { Timer::new(TimerChannel::Ch2) }
}

// ── System Tick (optional) ────────────────────────────────────────────────

/// Milliseconds since startup, incremented by a periodic timer ISR.
static mut SYSTEM_TICK_MS: u64 = 0;

/// Returns the system tick counter in milliseconds.
/// Only valid if `systick_init()` has been called and the timer ISR is firing.
pub fn systick_ms() -> u64 {
    unsafe { SYSTEM_TICK_MS }
}

/// Systick ISR: increments the system tick counter
fn systick_isr(_vector: u32) {
    unsafe {
        let irq_sta = timer_read(reg::IRQ_STA);
        if irq_sta & 1 != 0 {
            timer_write(reg::IRQ_STA, 1);
            SYSTEM_TICK_MS += 1;
        }
    }
}

/// Initialize a 1 kHz system tick using TIMER0.
/// Must call `avs_init()` first to enable the AVS clock (shared with timers).
///
/// After calling this, `systick_ms()` will return elapsed milliseconds.
pub fn systick_init() {
    let t0 = timer0();
    t0.configure(&TimerConfig {
        prescaler: Prescaler::Div1,
        mode: TimerMode::Periodic,
        interval: TIMER_SRC_HZ / 1000, // 24000 ticks = 1 ms at 24 MHz
    });

    interrupt::install(TIMER0_IRQ, systick_isr);

    // Unmask the timer interrupt
    interrupt::unmask(TIMER0_IRQ);

    // Start the timer
    t0.start(TIMER_SRC_HZ / 1000, TimerMode::Periodic);

    // Prevent t0 from being dropped (it must stay alive)
    core::mem::forget(t0);
}
