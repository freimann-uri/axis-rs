#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::psram::{PsramConfig, SpiRamFreq, SpiTimingConfigCoreClock};
use log::*;

esp_bootloader_esp_idf::esp_app_desc!();

unsafe extern "C" {
    static rom_spiflash_legacy_data: *mut u8;
    fn esp_rom_spiflash_read_user_cmd(status: *mut u32, cmd: u8) -> i32;
    fn esp_rom_spiflash_write_enable(spi: *mut u8) -> i32;
    fn esp_rom_spiflash_write_status(spi: *mut u8, status: u32) -> i32;
    fn esp_rom_spiflash_wait_idle(spi: *mut u8) -> i32;
}

const CMD_RDID: u8 = 0x9F;
const CMD_RDSR1: u8 = 0x05;
const CMD_RDCR: u8 = 0x15;
const QE_BIT: u32 = 1 << 6;

#[esp_hal::ram]
fn rd(cmd: u8) -> u32 {
    let mut v: u32 = 0;
    unsafe { esp_rom_spiflash_read_user_cmd(&mut v, cmd) };
    v
}

/// 0 = QE уже был, 1 = выставили, -1 = не Macronix (не трогали), -2 = запись не удалась
#[esp_hal::ram]
fn ensure_flash_qe() -> i32 {
    critical_section::with(|_| unsafe {
        let spi = rom_spiflash_legacy_data;

        let id = rd(CMD_RDID);
        if (id & 0xFF) != 0xC2 {
            return -1; // порядок байт в id проверь по логу; при ошибке просто не пишем
        }

        let sr1 = rd(CMD_RDSR1) & 0xFF;
        if sr1 & QE_BIT != 0 {
            return 0;
        }
        let cr = rd(CMD_RDCR) & 0xFF;

        esp_rom_spiflash_wait_idle(spi);
        esp_rom_spiflash_write_enable(spi);
        esp_rom_spiflash_write_status(spi, (sr1 | QE_BIT) | (cr << 8));
        esp_rom_spiflash_wait_idle(spi);

        if rd(CMD_RDSR1) & QE_BIT != 0 { 1 } else { -2 }
    })
}

extern crate alloc;
use alloc::vec::Vec;

fn psram_test(size: usize) -> bool {
    let mut buf: Vec<u32> = Vec::with_capacity(size / 4);
    let p = buf.as_mut_ptr();
    let addr = p as usize;
    // PSRAM на S3 отображается в 0x3C00_0000..0x3E00_0000
    info!(
        "buf @ {:#x}, in PSRAM = {}",
        addr,
        (0x3C00_0000..0x3E00_0000).contains(&addr)
    );

    let n = size / 4;
    let mut x: u32 = 0x1234_5678;
    for i in 0..n {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5; // xorshift: псевдослучайный паттерн
        unsafe { p.add(i).write_volatile(x ^ (i as u32)) };
    }
    let mut x: u32 = 0x1234_5678;
    let mut bad = 0u32;
    for i in 0..n {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        if unsafe { p.add(i).read_volatile() } != (x ^ (i as u32)) {
            bad += 1;
        }
    }
    info!("psram test {} KB: {} errors", size / 1024, bad);
    bad == 0
}

#[esp_hal::main]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let id = rd(CMD_RDID);
    info!("flash ID raw = {:#08x}", id);
    let r = ensure_flash_qe();
    info!("ensure QE -> {}, SR1 = {:#04x}", r, rd(CMD_RDSR1) & 0xFF);
    info!("CR = {:#04x}", rd(CMD_RDCR) & 0xFF);

    let peripherals = esp_hal::init(esp_hal::Config::default());

    info!("Initializing PSRAM allocator...");
    // esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    esp_alloc::psram_allocator!(
        peripherals.PSRAM,
        esp_hal::psram,
        PsramConfig {
            ram_frequency: SpiRamFreq::Freq80m,
            core_clock: Some(SpiTimingConfigCoreClock::SpiTimingConfigCoreClock160m),
            ..Default::default()
        }
    );
    info!("Done"); // never printed on this board

    psram_test(1024 * 1024);
    psram_test(6 * 1024 * 1024);

    loop {}
}
