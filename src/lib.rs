#![no_std]

// pub mod esp32_cam;
pub mod esp32s3_eye;
pub mod sensors;

pub use esp32s3_eye::{
    BUFFER_SIZE, Esp32S3Eye, ensure_flash_qe, flash_sr1, jpeg_buffers, jpeg_slice,
};
pub use sensors::SensorError;

#[derive(Debug, Clone, Copy)]
pub enum BoardError {
    Sensor(SensorError),
    InitFailed,
    CameraNotAvailable,
    ChipInitFailed,
    CaptureTimeout,
    DmaTransferError(esp_hal::dma::DmaError),
}

pub type BoardResult<T> = Result<T, BoardError>;

impl From<SensorError> for BoardError {
    fn from(e: SensorError) -> Self {
        BoardError::Sensor(e)
    }
}
