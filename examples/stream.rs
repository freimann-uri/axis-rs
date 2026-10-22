#![no_std]
#![no_main]

use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    lcd_cam::cam::EofMode,
    psram::{Psram, PsramConfig, SpiRamFreq, SpiTimingConfigCoreClock},
    timer::timg::TimerGroup,
};
use esp_println::println;
use rs_stream::{
    Esp32S3Eye, ensure_flash_qe, flash_sr1, jpeg_buffers, jpeg_slice,
    sensors::{Resolution, ov5640::Ov5640},
};
esp_bootloader_esp_idf::esp_app_desc!();

// const QSCALE: u8 = 0x0C;

use esp_hal::dma::DmaRxBuf;
use rs_stream::sensors::Sensor;

fn reinit(eye: &mut Esp32S3Eye<Ov5640>) {
    eye.sensor.init(&mut eye.i2c).unwrap();
}

// fn probe(label: impl core::fmt::Debug) {
//     use esp_hal::time::{Duration, Instant};
//     let gpio = unsafe { &*esp_hal::peripherals::GPIO::PTR };
//     let (mut vs, mut hr) = (0u32, 0u32);
//     let first = gpio.in_().read().bits();
//     let mut last = first;
//     let t0 = Instant::now();
//     while t0.elapsed() < Duration::from_millis(1500) {
//         let now = gpio.in_().read().bits();
//         let ch = now ^ last;
//         if ch & (1 << 6) != 0 {
//             vs += 1;
//         }
//         if ch & (1 << 7) != 0 {
//             hr += 1;
//         }
//         last = now;
//     }
//     println!(
//         "[probe {:?}] VSYNC edges={} (lvl {}), HREF edges={} (lvl {})",
//         label,
//         vs,
//         (first >> 6) & 1,
//         hr,
//         (first >> 7) & 1
//     );
// }

fn _probe2(label: &str, ms: u64) {
    use esp_hal::time::{Duration, Instant};
    let gpio = unsafe { &*esp_hal::peripherals::GPIO::PTR };
    let t0 = Instant::now();
    let mut last = gpio.in_().read().bits();
    let (mut samples, mut vs, mut hr) = (0u32, 0usize, 0u32);
    let mut vs_t = [0u64; 8]; // время первых фронтов VSYNC, мкс от старта
    while t0.elapsed() < Duration::from_millis(ms) {
        let now = gpio.in_().read().bits();
        samples += 1;
        let ch = now ^ last;
        if ch & (1 << 6) != 0 {
            if vs < 8 {
                vs_t[vs] = t0.elapsed().as_micros();
            }
            vs += 1;
        }
        if ch & (1 << 7) != 0 {
            hr += 1;
        }
        last = now;
    }
    println!(
        "[{}] {} ms: samples/ms={}, VSYNC edges={} at {:?} us, HREF edges={}, final lvl vs={} hr={}",
        label,
        ms,
        samples as u64 / ms,
        vs,
        &vs_t[..vs.min(8)],
        hr,
        (last >> 6) & 1,
        (last >> 7) & 1
    );
}

fn _probe_data_pins(ms: u64) {
    use esp_hal::time::{Duration, Instant};
    let gpio = unsafe { &*esp_hal::peripherals::GPIO::PTR };
    let pins: [u32; 9] = [11, 9, 8, 47, 12, 18, 17, 16, 10]; // d0..d7 как у тебя, плюс GPIO10
    let mut cnt = [0u32; 9];
    let mut last_lo = gpio.in_().read().bits();
    let mut last_hi = gpio.in1().read().bits();
    let t0 = Instant::now();
    while t0.elapsed() < Duration::from_millis(ms) {
        let lo = gpio.in_().read().bits();
        let hi = gpio.in1().read().bits();
        for (i, &p) in pins.iter().enumerate() {
            let changed = if p < 32 {
                (lo ^ last_lo) >> p & 1
            } else {
                (hi ^ last_hi) >> (p - 32) & 1
            };
            cnt[i] += changed;
        }
        last_lo = lo;
        last_hi = hi;
    }
    for (i, &p) in pins.iter().enumerate() {
        println!("[pins] GPIO{} edges={}", p, cnt[i]);
    }
}

// fn wr(i2c: &mut esp_hal::i2c::master::I2c<'static, esp_hal::Blocking>, regs: &[[u16; 2]]) {
//     for e in regs {
//         i2c.write(0x3C, &[(e[0] >> 8) as u8, e[0] as u8, e[1] as u8])
//             .unwrap();
//     }
// }
// fn rd8(i2c: &mut esp_hal::i2c::master::I2c<'static, esp_hal::Blocking>, reg: u16) -> u8 {
//     let mut b = [0u8];
//     i2c.write_read(0x3C, &[(reg >> 8) as u8, reg as u8], &mut b)
//         .unwrap();
//     b[0]
// }

fn dbg_frame(n: usize, buf: &DmaRxBuf) {
    let data = buf.as_slice();
    let rep = buf.number_of_received_bytes().min(data.len());
    let rx = &data[..rep];
    let first = |p: [u8; 2]| rx.windows(2).position(|w| w == p);
    let last = |p: [u8; 2]| rx.windows(2).rposition(|w| w == p);
    let zeros = rx.iter().filter(|&&b| b == 0).count();

    println!("[dbg #{}] rep={} head={:02X?}", n, rep, &rx[..16.min(rep)]);
    println!("[dbg #{}] tail={:02X?}", n, &rx[rep.saturating_sub(16)..]);
    println!(
        "[dbg #{}] FFD8@{:?} D8FF@{:?} FFD9@{:?} D9FF@{:?} zeros={}/{}",
        n,
        first([0xFF, 0xD8]),
        first([0xD8, 0xFF]),
        last([0xFF, 0xD9]),
        last([0xD9, 0xFF]),
        zeros,
        rep
    );
}

fn dbg_structure(n: usize, buf: &DmaRxBuf) {
    let data = buf.as_slice();
    let rep = buf.number_of_received_bytes().min(data.len());
    let rx = &data[..rep];

    let mut cnt = 0usize;
    for i in 0..rx.len().saturating_sub(1) {
        if rx[i] == 0xFF && rx[i + 1] != 0x00 && rx[i + 1] != 0xFF {
            if cnt < 14 {
                println!("[struct #{}] marker FF{:02X} @{}", n, rx[i + 1], i);
            }
            cnt += 1;
        }
    }
    println!("[struct #{}] total markers={}", n, cnt);

    match rx.iter().rposition(|&b| b != 0) {
        Some(e) => {
            let from = e.saturating_sub(15);
            println!(
                "[struct #{}] last non-zero @{} of {}: {:02X?}",
                n,
                e,
                rep,
                &rx[from..=e]
            );
        }
        None => println!("[struct #{}] all zeros", n),
    }
}

#[esp_rtos::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = esp_hal::init(esp_hal::Config::default());

    let r = ensure_flash_qe();
    println!("[main] ensure QE -> {}, SR1 = {:#04x}", r, flash_sr1());

    let timg0 = TimerGroup::new(p.TIMG0);
    esp_rtos::start(timg0.timer0);

    let psram = Psram::new(
        p.PSRAM,
        PsramConfig {
            ram_frequency: SpiRamFreq::Freq80m,
            core_clock: Some(SpiTimingConfigCoreClock::SpiTimingConfigCoreClock160m),
            ..Default::default()
        },
    );
    let (psram_base, psram_size) = psram.raw_parts();
    println!("[main] PSRAM: base {:p}, size {}", psram_base, psram_size);
    let [buf_a, buf_b] = jpeg_buffers(psram_base);

    let mut eye = Esp32S3Eye::<Ov5640>::init(
        p.LCD_CAM,
        p.DMA_CH0,
        EofMode::VsyncSignal,
        p.I2C0,
        p.GPIO15, // xclk
        p.GPIO13, // pclk
        p.GPIO6,  // vsync
        p.GPIO7,  // href
        p.GPIO11,
        p.GPIO9,
        p.GPIO8,
        p.GPIO10, // d0-d3
        p.GPIO12,
        p.GPIO18,
        p.GPIO17,
        p.GPIO16, // d4-d7
        p.GPIO4,  // sda
        p.GPIO5,  // scl
    )
    .unwrap();

    reinit(&mut eye);
    eye.set_jpeg(Resolution::Vga, 0x0C).unwrap(); // сначала малый режим, как в эталоне по умолчанию
    Timer::after_millis(300).await;

    // probe_data_pins(1000);
    // probe2("jpeg vga", 2000);

    // 5. Захват в два буфера по очереди
    let mut bufs = [Some(buf_a), Some(buf_b)];
    let mut n = 0usize;
    loop {
        let buf = bufs[n % 2].take().unwrap();
        let buf = match eye.capture(buf).await {
            Ok(buf) => {
                if n >= 1 && n <= 2 {
                    dbg_structure(n, &buf);
                }

                if n >= 1 && n <= 3 {
                    dbg_frame(n, &buf);
                }

                if n > 0 {
                    // первый кадр может начаться посреди кадра, пропускаем
                    match jpeg_slice(&buf) {
                        Some(j) => println!("frame #{}: jpeg {} bytes", n, j.len()),
                        None => println!(
                            "frame #{}: no SOI/EOI, got {}",
                            n,
                            buf.number_of_received_bytes()
                        ),
                    }
                }
                buf
            }
            Err((e, buf)) => {
                println!("capture error: {:?}", e);
                buf
            }
        };
        bufs[n % 2] = Some(buf);
        n += 1;
    }
}
