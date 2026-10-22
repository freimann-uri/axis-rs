pub mod ov5640;

use esp_hal::Blocking;
use esp_hal::i2c::master::I2c;

#[derive(Debug, Clone, Copy)]
pub enum SensorError {
    I2cError,
    SensorMismatch(u16),
}

#[derive(Clone, Copy, Debug)]
pub enum Resolution {
    Vga,    // 640x480
    Svga,   // 800x600
    Hd720,  // 1280x720
    FullHd, // 1920x1080
    Qxga,   // 2048x1536
    Qsxga,  // 2592x1944, родное разрешение матрицы
}

impl Resolution {
    pub const fn size(self) -> (u16, u16) {
        match self {
            Self::Vga => (640, 480),
            Self::Svga => (800, 600),
            Self::Hd720 => (1280, 720),
            Self::FullHd => (1920, 1080),
            Self::Qxga => (2048, 1536),
            Self::Qsxga => (2592, 1944),
        }
    }
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

    fn set_jpeg(
        &mut self,
        i2c: &mut I2c<'_, Blocking>,
        res: Resolution,
        qscale: u8,
    ) -> SensorResult<()>;
}
