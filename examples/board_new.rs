#![no_std]
#![no_main]

use axis_rs::Esp32S3Eye;
use axis_rs::sensors::ov5640::Ov5640;
use esp_backtrace as _;
use esp_hal::main;
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());

    match Esp32S3Eye::init::<Ov5640>(p) {
        Ok((_board, _sensor)) => {
            println!("[EXAMPLE] ESP32-S3-EYE fully initialized!");
            // board.i2c и board.camera доступны напрямую
        }
        Err(e) => println!("[EXAMPLE] Failed: {:?}", e),
    }

    loop {
        core::hint::spin_loop();
    }
}
