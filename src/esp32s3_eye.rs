use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_time::{Duration, with_timeout};
use esp_hal::Blocking;
use esp_hal::delay::Delay;
use esp_hal::dma::DmaRxBuf;
use esp_hal::dma::{DmaDescriptor, aligned::DmaAlignedMut};
use esp_hal::gpio::interconnect::{PeripheralInput, PeripheralOutput};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::lcd_cam::cam::{Camera, Config, EofMode};
use esp_hal::lcd_cam::{CamDmaRxChannel, LcdCam};
use esp_hal::peripherals::{I2C0, LCD_CAM};
use esp_hal::time::Rate;
use esp_println::println;

use crate::sensors::ov5640::Ov5640;
use crate::sensors::{Resolution, Sensor};
use crate::{BoardError, BoardResult};

pub struct Esp32S3Eye<S: Sensor> {
    pub i2c: I2c<'static, Blocking>,
    pub camera: Option<Camera<'static>>,
    pub sensor: S,
}

impl<S: Sensor> Esp32S3Eye<S> {
    pub fn init<DMA: CamDmaRxChannel<'static>>(
        lcd_cam_periph: LCD_CAM<'static>,
        dma_channel: DMA,
        eof_mode: EofMode,
        i2c0: I2C0<'static>,
        mclk: impl PeripheralOutput<'static>,
        pclk: impl PeripheralInput<'static>,
        vsync: impl PeripheralInput<'static>,
        href: impl PeripheralInput<'static>,
        data0: impl PeripheralInput<'static>,
        data1: impl PeripheralInput<'static>,
        data2: impl PeripheralInput<'static>,
        data3: impl PeripheralInput<'static>,
        data4: impl PeripheralInput<'static>,
        data5: impl PeripheralInput<'static>,
        data6: impl PeripheralInput<'static>,
        data7: impl PeripheralInput<'static>,
        sda: impl PeripheralInput<'static> + PeripheralOutput<'static>,
        scl: impl PeripheralInput<'static> + PeripheralOutput<'static>,
    ) -> BoardResult<Self> {
        let lcd_cam = LcdCam::new(lcd_cam_periph);

        let camera = Camera::new(
            lcd_cam.cam,
            dma_channel,
            Config::default()
                .with_frequency(Rate::from_mhz(20))
                .with_eof_mode(eof_mode),
        )
        .map_err(|_| BoardError::ChipInitFailed)?
        .with_master_clock(mclk)
        .with_pixel_clock(pclk)
        .with_vsync(vsync)
        .with_h_enable(href)
        .with_data0(data0)
        .with_data1(data1)
        .with_data2(data2)
        .with_data3(data3)
        .with_data4(data4)
        .with_data5(data5)
        .with_data6(data6)
        .with_data7(data7);

        println!("[board] Camera OK! (XCLK: 20 MHz, DVP 8-bit)");

        Delay::new().delay_millis(10);
        let mut i2c = I2c::new(
            i2c0,
            I2cConfig::default().with_frequency(Rate::from_khz(400)),
        )
        .map_err(|_| BoardError::ChipInitFailed)?
        .with_sda(sda)
        .with_scl(scl);

        println!("[board] SCCB OK! (Master mode, 400 kHz)");

        let mut sensor = S::new();
        sensor.init(&mut i2c)?;

        Ok(Self {
            i2c,
            camera: Some(camera),
            sensor,
        })
    }

    /// Один кадр до EOF. Буфер возвращается всегда, и при ошибке тоже.
    pub async fn capture(&mut self, rx_buf: DmaRxBuf) -> Result<DmaRxBuf, (BoardError, DmaRxBuf)> {
        let Some(camera) = self.camera.take() else {
            return Err((BoardError::CameraNotAvailable, rx_buf));
        };
        let camera = camera.into_async();

        let regs = unsafe { &*esp_hal::peripherals::LCD_CAM::PTR };
        println!(
            "cam_vs_eof_en = {}",
            regs.cam_ctrl().read().cam_vs_eof_en().bit()
        );

        let mut transfer = match camera.receive(rx_buf) {
            Ok(t) => t,
            Err((e, camera, buf)) => {
                self.camera = Some(camera.into_blocking());
                return Err((BoardError::DmaTransferError(e), buf));
            }
        };

        let res = with_timeout(Duration::from_secs(3), transfer.wait_for_dma_eof()).await;
        let (camera, buf) = transfer.stop(); // всегда, в том числе после таймаута
        self.camera = Some(camera.into_blocking());

        match res {
            Ok(Ok(())) => Ok(buf),
            Ok(Err(e)) => Err((BoardError::DmaTransferError(e), buf)),
            Err(_) => Err((BoardError::CaptureTimeout, buf)),
        }
    }
}

impl Esp32S3Eye<Ov5640> {
    pub fn set_fullhd(&mut self) -> BoardResult<()> {
        self.sensor.set_fullhd(&mut self.i2c)?;
        Ok(())
    }

    pub fn set_qqvga(&mut self) -> BoardResult<()> {
        self.sensor.set_qqvga(&mut self.i2c)?;
        Ok(())
    }

    pub fn set_jpeg(&mut self, res: Resolution, qscale: u8) -> BoardResult<()> {
        self.sensor.set_jpeg(&mut self.i2c, res, qscale)?;
        Ok(())
    }
}

pub const JPEG_BUF_SIZE: usize = 1536 * 1024;
const DESC_CHUNK: usize = 4032;
const DESC_COUNT: usize = JPEG_BUF_SIZE.div_ceil(DESC_CHUNK);
pub const BUFFER_SIZE: usize = DESC_COUNT * DESC_CHUNK;
const PSRAM_END: usize = 0x3C80_0000;

static mut DESCS_A: [DmaDescriptor; DESC_COUNT] = [DmaDescriptor::EMPTY; DESC_COUNT];
static mut DESCS_B: [DmaDescriptor; DESC_COUNT] = [DmaDescriptor::EMPTY; DESC_COUNT];
static BUFFERS_TAKEN: AtomicBool = AtomicBool::new(false);

pub fn jpeg_buffers(psram_base: *mut u8) -> [DmaRxBuf; 2] {
    assert!(
        !BUFFERS_TAKEN.swap(true, Ordering::SeqCst),
        "buffers already taken"
    );
    assert!(
        psram_base as usize + 2 * BUFFER_SIZE <= PSRAM_END,
        "not enough PSRAM"
    );
    let make = |offset: usize, descs: *mut [DmaDescriptor; DESC_COUNT]| unsafe {
        let data = core::slice::from_raw_parts_mut(psram_base.add(offset), BUFFER_SIZE);
        let descs = core::slice::from_raw_parts_mut(descs as *mut DmaDescriptor, DESC_COUNT);
        DmaRxBuf::new(
            DmaAlignedMut::new(descs).unwrap(),
            DmaAlignedMut::new(data).unwrap(),
        )
        .unwrap()
    };
    [
        make(0, addr_of_mut!(DESCS_A)),
        make(BUFFER_SIZE, addr_of_mut!(DESCS_B)),
    ]
}

pub fn jpeg_slice(buf: &DmaRxBuf) -> Option<&[u8]> {
    let data = buf.as_slice();
    let n = buf.number_of_received_bytes().min(data.len());
    let data = &data[..n];
    let start = data.windows(2).position(|w| w == [0xFF, 0xD8])?;
    let end = data.windows(2).rposition(|w| w == [0xFF, 0xD9])?;
    (end > start).then(|| &data[start..end + 2])
}

// ------- Временный обход: Flash QE для Macronix (esp-hal #6456 / #6492) -------
// Не имеет отношения к камере. Удалить целиком, когда поправят в апстриме.
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
fn flash_rd(cmd: u8) -> u32 {
    let mut v: u32 = 0;
    unsafe { esp_rom_spiflash_read_user_cmd(&mut v, cmd) };
    v
}

#[esp_hal::ram]
pub fn flash_sr1() -> u32 {
    flash_rd(CMD_RDSR1) & 0xFF
}

#[esp_hal::ram]
pub fn ensure_flash_qe() -> i32 {
    critical_section::with(|_| unsafe {
        let spi = rom_spiflash_legacy_data;
        if (flash_rd(CMD_RDID) & 0xFF) != 0xC2 {
            return -1;
        }
        let sr1 = flash_rd(CMD_RDSR1) & 0xFF;
        if sr1 & QE_BIT != 0 {
            return 0;
        }
        let cr = flash_rd(CMD_RDCR) & 0xFF;
        esp_rom_spiflash_wait_idle(spi);
        esp_rom_spiflash_write_enable(spi);
        esp_rom_spiflash_write_status(spi, (sr1 | QE_BIT) | (cr << 8));
        esp_rom_spiflash_wait_idle(spi);
        if flash_rd(CMD_RDSR1) & QE_BIT != 0 {
            1
        } else {
            -2
        }
    })
}
