pub mod ov5640;

use esp_hal::Blocking;
use esp_hal::i2c::master::I2c;

#[derive(Debug, Clone, Copy)]
pub enum SensorError {
    I2cError,
    SensorMismatch(u16),
}

pub type SensorResult<T> = Result<T, SensorError>;

impl<E: embedded_hal::i2c::Error> From<E> for SensorError {
    fn from(_: E) -> Self {
        SensorError::I2cError
    }
}

pub trait Sensor {
    fn new() -> Self;
    fn init(&mut self, i2c: &mut I2c<'_, Blocking>) -> SensorResult<()>;
    fn set_qqvga(&mut self, i2c: &mut I2c<'_, Blocking>) -> SensorResult<()>;
    fn set_fullhd(&mut self, i2c: &mut I2c<'_, Blocking>) -> SensorResult<()>;
}
