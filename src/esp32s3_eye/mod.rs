pub mod capture;
// pub mod capture_internal;
pub mod init;
use crate::BoardResult;

use crate::sensors::Sensor;
use esp_hal::Blocking;
use esp_hal::i2c::master::I2c;
use esp_hal::lcd_cam::cam::Camera;

pub struct Esp32S3Eye<S: Sensor> {
    pub i2c: I2c<'static, Blocking>,
    pub camera: Option<Camera<'static>>,
    pub sensor: S,
}

use crate::sensors::ov5640::Ov5640;

impl Esp32S3Eye<Ov5640> {
    pub fn set_fullhd(&mut self) -> BoardResult<()> {
        self.sensor.set_fullhd(&mut self.i2c)?;
        Ok(())
    }

    pub fn set_qqvga(&mut self) -> BoardResult<()> {
        self.sensor.set_qqvga(&mut self.i2c)?;
        Ok(())
    }
}
