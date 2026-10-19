#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

use esp_backtrace as _;
use esp_hal::psram::{Psram, PsramConfig};
use esp_println::println;

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());

    let psram = Psram::new(p.PSRAM, PsramConfig::default());
    let (psram_base, size) = psram.raw_parts();
    println!("PSRAM base {:p}, size {}", psram_base, size);

    unsafe {
        println!("writing...");

        for i in 0..64usize {
            core::ptr::write_volatile(psram_base.add(i), (i % 256) as u8);
        }

        println!("write done, reading...");

        let mut ok = true;
        for i in 0..64usize {
            let v = core::ptr::read_volatile(psram_base.add(i));
            if v != (i % 256) as u8 {
                println!("mismatch at {}: {}", i, v);
                ok = false;
            }
        }
        println!("CPU write/read PSRAM: {}", if ok { "OK" } else { "FAILED" });
    }

    loop {}
}
