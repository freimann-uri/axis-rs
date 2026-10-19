use esp_hal::dma::DmaRxBuffer;

use super::Esp32S3Eye;
use crate::BoardError;
use crate::sensors::Sensor;

impl<S: Sensor> Esp32S3Eye<S> {
    /// Захватывает один кадр в переданный буфер. Буфер (и память, в которой
    /// он лежит — внутренняя SRAM или PSRAM) выбирает вызывающий код.
    pub fn capture<BUF: DmaRxBuffer>(&mut self, buf: BUF) -> Result<BUF::Final, BoardError> {
        let camera = self.camera.take().ok_or(BoardError::CameraNotAvailable)?;

        let transfer = match camera.receive(buf) {
            Ok(transfer) => transfer,
            Err((err, camera, _buf)) => {
                self.camera = Some(camera);
                return Err(BoardError::DmaTransferError(err));
            }
        };

        let (result, camera, final_buf) = transfer.wait();
        self.camera = Some(camera);

        result.map_err(BoardError::DmaTransferError)?;
        Ok(final_buf)
    }
}
