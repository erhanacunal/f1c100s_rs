//! SPI DMA self-test for f1c100s (SPI0 boot NOR flash)
//!
//! Reports on UART2 (PE7 TX / PE8 RX, 115200 8N1):
//! 1. JEDEC ID and flash size
//! 2. 256 KiB read from address 0: PIO vs DMA, compared byte for byte,
//!    with throughput
//! 3. Unaligned DMA read (odd buffer offset and flash address), with guard
//!    bytes around the buffer to catch cache-line / overrun damage
//! 4. DMA write: the LAST 4 KiB sector of the flash is saved to RAM, erased,
//!    programmed with a pattern (256-byte page programs go through TX DMA),
//!    read back via DMA, and then restored and verified. The boot image at
//!    the start of the flash is not touched.
//!
//! Then waits for console keys: [f] FEL mode, [r] reset.

#![no_std]
#![no_main]

use core::fmt::Write;
use f1c100s::dma::{self, Channel};
use f1c100s::mmu::{self, MemDesc, RW_CB, RW_NCNB};
use f1c100s::spi::{self, BitOrder, ChipSelect, SpiMode};
use f1c100s::spi_flash::SpiFlash;
use f1c100s::{cpu, interrupt, timer, uart};

const READ_LEN: usize = 256 * 1024;
const SECTOR: usize = 4096;

#[repr(C, align(32))]
struct Buf<const N: usize>([u8; N]);

static mut BUF_A: Buf<READ_LEN> = Buf([0; READ_LEN]);
static mut BUF_B: Buf<READ_LEN> = Buf([0; READ_LEN]);
static mut SAVED: Buf<SECTOR> = Buf([0; SECTOR]);

/// AVS counter 0 (1 µs ticks, started by `timer::avs_init`)
fn avs_us() -> u32 {
    unsafe { (0x01C2_0C84 as *const u32).read_volatile() }
}

struct Out(uart::Uart);
impl Write for Out {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.0.write_str(s);
        Ok(())
    }
}

fn report(out: &mut Out, name: &str, ok: bool) {
    let _ = writeln!(out, "[{}] {}", if ok { "PASS" } else { "FAIL" }, name);
}

fn kib_per_s(bytes: usize, us: u32) -> u32 {
    if us == 0 { 0 } else { (bytes as u64 * 1_000_000 / 1024 / us as u64) as u32 }
}

#[no_mangle]
pub extern "C" fn rust_main() -> ! {
    mmu::init(&[
        MemDesc::new(0x0000_0000, 0xFFFF_FFFF, 0x0000_0000, RW_NCNB),
        MemDesc::new(0x8000_0000, 0x81FF_FFFF, 0x8000_0000, RW_CB),
    ]);
    interrupt::init();
    timer::avs_init();
    let mut out = Out(uart::init_uart2());
    let _ = writeln!(out, "\r\nSPI DMA self-test");

    dma::init();
    cpu::interrupt_enable(cpu::MODE_SVC);

    let mut bus = spi::spi0();
    bus.configure(SpiMode::Mode0, BitOrder::MsbFirst, ChipSelect::Ss0, 50_000_000);

    let (a, b, saved) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(BUF_A.0),
            &mut *core::ptr::addr_of_mut!(BUF_B.0),
            &mut *core::ptr::addr_of_mut!(SAVED.0),
        )
    };

    // 1. Identify
    let (id, size) = {
        let flash = SpiFlash::new(&bus);
        (flash.read_jedec_id(), flash.size_bytes())
    };
    let size = match (id, size) {
        (Ok(id), Ok(Some(size))) => {
            let _ = writeln!(out, "JEDEC {:02X} {:02X} {:02X}, {} KiB", id[0], id[1], id[2], size / 1024);
            size
        }
        (id, _) => {
            let _ = writeln!(out, "[FAIL] no valid JEDEC ID: {:?}", id);
            idle(&out.0)
        }
    };

    // 2. PIO read (reference), then DMA read of the same range
    let t0 = avs_us();
    let pio_ok = SpiFlash::new(&bus).read(0, a).is_ok();
    let t_pio = avs_us().wrapping_sub(t0);

    bus.set_dma(Channel::N2, Channel::N3).unwrap();
    b.fill(0);
    let t0 = avs_us();
    let dma_ok = SpiFlash::new(&bus).read(0, b).is_ok();
    let t_dma = avs_us().wrapping_sub(t0);
    report(&mut out, "256 KiB read, DMA == PIO", pio_ok && dma_ok && a[..] == b[..]);
    let _ = writeln!(
        out,
        "  PIO {} us ({} KiB/s), DMA {} us ({} KiB/s)",
        t_pio, kib_per_s(READ_LEN, t_pio), t_dma, kib_per_s(READ_LEN, t_dma)
    );

    // 3. Unaligned DMA read with guard bytes: flash[123 .. 123+5000] -> b[3 ..]
    b[..5100].fill(0xA5);
    let ok = SpiFlash::new(&bus).read(123, &mut b[3..5003]).is_ok()
        && b[3..5003] == a[123..5123]
        && b[..3].iter().all(|&x| x == 0xA5)
        && b[5003..5100].iter().all(|&x| x == 0xA5);
    report(&mut out, "unaligned 5000 B DMA read + guards", ok);

    // 4. DMA write on the last sector, then restore it
    let addr = size - SECTOR as u32;
    let flash = SpiFlash::new(&bus);
    let mut ok = flash.read(addr, saved).is_ok();
    for (i, x) in a[..SECTOR].iter_mut().enumerate() {
        *x = (i as u8).wrapping_mul(7) ^ 0x5A;
    }
    ok &= flash.sector_erase_4k(addr).is_ok();
    ok &= flash.write(addr, &a[..SECTOR]).is_ok();
    b[..SECTOR].fill(0);
    ok &= flash.read(addr, &mut b[..SECTOR]).is_ok();
    let write_ok = ok && a[..SECTOR] == b[..SECTOR];
    report(&mut out, "4 KiB DMA page-program + readback (last sector)", write_ok);

    let mut restored = flash.sector_erase_4k(addr).is_ok() && flash.write(addr, saved).is_ok();
    restored &= flash.read(addr, &mut b[..SECTOR]).is_ok() && b[..SECTOR] == saved[..];
    report(&mut out, "last sector restored", restored);

    idle(&out.0)
}

fn idle(u: &uart::Uart) -> ! {
    u.write_str("Keys: [f] FEL mode  [r] reset\r\n");
    loop {
        u.poll_dev_keys();
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
