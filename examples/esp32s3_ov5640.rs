#![no_std]
#![no_main]

use axis_rs::Esp32S3Eye;
use axis_rs::sensors::ov5640::Ov5640;
use esp_backtrace as _;
use esp_hal::dma::{DmaDescriptor, DmaRxBuf, aligned::DmaAlignedMut};
use esp_hal::psram::{Psram, PsramConfig};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

const FRAME_SIZE: usize = 1920 * 1080 * 2;
const DESC_CHUNK: usize = 4032;
const DESC_COUNT: usize = 1029; // ceil(FRAME_SIZE / DESC_CHUNK)
const BUFFER_SIZE: usize = DESC_COUNT * DESC_CHUNK;

static mut DESCS_A: [DmaDescriptor; DESC_COUNT] = [DmaDescriptor::EMPTY; DESC_COUNT];
static mut DESCS_B: [DmaDescriptor; DESC_COUNT] = [DmaDescriptor::EMPTY; DESC_COUNT];

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());

    let mut board = Esp32S3Eye::<Ov5640>::init(
        p.LCD_CAM, p.DMA_CH0, FRAME_SIZE, p.I2C0, p.GPIO15, // mclk
        p.GPIO13, // pclk
        p.GPIO6,  // vsync
        p.GPIO7,  // href
        p.GPIO11, p.GPIO9, p.GPIO8, p.GPIO47, // data0-3
        p.GPIO12, p.GPIO18, p.GPIO17, p.GPIO16, // data4-7
        p.GPIO4,  // sda
        p.GPIO5,  // scl
    )
    .unwrap();

    board.set_fullhd().unwrap();

    let psram = Psram::new(p.PSRAM, PsramConfig::default());
    let (psram_base, psram_size) = psram.raw_parts();
    println!("[main] PSRAM: base {:p}, size {}", psram_base, psram_size);

    let mut use_slot_a = true;

    loop {
        let (desc_slice, data_slice): (&'static mut [DmaDescriptor], &'static mut [u8]) = unsafe {
            let offset = if use_slot_a { 0 } else { BUFFER_SIZE };
            let data = core::slice::from_raw_parts_mut(psram_base.add(offset), BUFFER_SIZE);
            let descs = if use_slot_a {
                &mut DESCS_A[..]
            } else {
                &mut DESCS_B[..]
            };
            use_slot_a = !use_slot_a;
            (descs, data)
        };

        let aligned_descs = DmaAlignedMut::new(desc_slice).unwrap();
        let aligned_data = DmaAlignedMut::new(data_slice).unwrap();
        let rx_buf = DmaRxBuf::new(aligned_descs, aligned_data).unwrap();

        // match board.capture(rx_buf) {
        //     Ok(buf) => {
        //         let frame = buf.as_slice();
        //         println!(
        //             "frame: {} bytes, addr {:p}, pixels[0..4]: {:02X?}",
        //             frame.len(),
        //             frame.as_ptr(),
        //             &frame[..4.min(frame.len())]
        //         );
        //     }
        //     Err(e) => println!("capture error: {:?}", e),
        // }

        let camera = board.camera.take().unwrap();
        let transfer = match camera.receive(rx_buf) {
            Ok(t) => t,
            Err((e, _camera, _buf)) => {
                println!("receive() failed immediately: {:?}", e);
                loop {}
            }
        };

        let mut spins = 0u32;
        loop {
            if transfer.is_done() {
                println!("is_done() after {} spins", spins);
                break;
            }
            spins += 1;
            if spins % 1_000_000 == 0 {
                println!("still waiting... {} spins", spins);
            }
        }
    }
}
