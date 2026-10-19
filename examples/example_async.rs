#![no_std]
#![no_main]

use axis_rs::Esp32S3Eye;
use axis_rs::sensors::ov5640::Ov5640;
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;

const EXPECTED_FRAME_SIZE: usize = 160 * 120 * 2;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());
    let peripherals = esp_hal::init(config);

    let (mut board, _sensor) = Esp32S3Eye::init::<Ov5640>(peripherals).expect("Init failed");

    let timg0 = unsafe { TimerGroup::new(esp_hal::peripherals::TIMG0::steal()) };
    let intr0 = unsafe { esp_hal::peripherals::FROM_CPU_INTR0::steal() };
    esp_rtos::start(timg0.timer0, intr0);

    println!("[MAIN] Starting reactive async/await loop...");

    loop {
        match board.capture_internal().await {
            Ok(frame) => {
                assert_eq!(frame.len(), EXPECTED_FRAME_SIZE, "unexpected frame size");
                println!(
                    "🎉 frame: {}x{} RGB565, {} bytes (matches expected size), addr {:p}",
                    160,
                    120,
                    frame.len(),
                    frame.as_ptr()
                );
                if frame.len() >= 4 {
                    println!("pixels[0..4]: {:02X?}", &frame[0..4]);
                }
            }
            Err(e) => println!("❌ capture error: {:?}", e),
        }
    }
}
