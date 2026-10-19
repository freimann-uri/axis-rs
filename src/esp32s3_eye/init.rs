use esp_hal::delay::Delay;
use esp_hal::gpio::interconnect::{PeripheralInput, PeripheralOutput};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::lcd_cam::CamDmaRxChannel;
use esp_hal::lcd_cam::LcdCam;
use esp_hal::lcd_cam::cam::{Camera, Config, EofMode};
use esp_hal::peripherals::{I2C0, LCD_CAM};
use esp_hal::time::Rate;
use esp_println::println;

use super::Esp32S3Eye;
use crate::sensors::Sensor;
use crate::{BoardError, BoardResult};

impl<S: Sensor> Esp32S3Eye<S> {
    pub fn init<DMA: CamDmaRxChannel<'static>>(
        lcd_cam_periph: LCD_CAM<'static>,
        dma_channel: DMA,
        frame_size: usize,
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
                .with_eof_mode(EofMode::ByteLen((frame_size - 1) as u16)),
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
}
