use super::Esp32S3Eye;
use crate::BoardError;
use esp_hal::dma::{DmaDescriptor, DmaRxBuf, aligned::DmaAlignedMut};

const FRAME_SIZE: usize = 160 * 120 * 2;
const DESC_COUNT: usize = (FRAME_SIZE + 4031) / 4032;

static mut FRAME_POOL_A: [u8; FRAME_SIZE] = [0; FRAME_SIZE];
static mut FRAME_POOL_B: [u8; FRAME_SIZE] = [0; FRAME_SIZE];
static mut DESCS_A: [DmaDescriptor; DESC_COUNT] = [DmaDescriptor::EMPTY; DESC_COUNT];
static mut DESCS_B: [DmaDescriptor; DESC_COUNT] = [DmaDescriptor::EMPTY; DESC_COUNT];

static mut USE_BUFFER_A: bool = true;

impl Esp32S3Eye {
    pub async fn capture_internal(&mut self) -> Result<&'static [u8], BoardError> {
        let camera = self.camera.take().ok_or(BoardError::CaptureTimeout)?;

        let (data_slice, desc_slice): (&'static mut [u8], &'static mut [DmaDescriptor]) = unsafe {
            let bufs = if USE_BUFFER_A {
                (&mut FRAME_POOL_A[..], &mut DESCS_A[..])
            } else {
                (&mut FRAME_POOL_B[..], &mut DESCS_B[..])
            };
            USE_BUFFER_A = !USE_BUFFER_A;
            bufs
        };

        let aligned_descs = DmaAlignedMut::new(desc_slice).map_err(|_| BoardError::InitFailed)?;
        let aligned_data = DmaAlignedMut::new(data_slice).map_err(|_| BoardError::InitFailed)?;
        let rx_buf =
            DmaRxBuf::new(aligned_descs, aligned_data).map_err(|_| BoardError::InitFailed)?;

        let lcd_cam_regs = unsafe { &*esp_hal::peripherals::LCD_CAM::PTR };
        esp_println::println!(
            "cam_vs_eof_en перед receive(): {}",
            lcd_cam_regs.cam_ctrl().read().cam_vs_eof_en().bit()
        );

        let transfer = camera.receive(rx_buf).map_err(|_| BoardError::InitFailed)?;

        transfer
            .wait_for_frame()
            .await
            // .map_err(BoardError::DmaTransferError)?;
            .unwrap();

        let (camera, buf) = transfer.stop();
        self.camera = Some(camera);

        let ptr = buf.as_slice().as_ptr();
        let len = buf.as_slice().len();

        // SAFETY: `ptr`/`len` указывают в FRAME_POOL_A/FRAME_POOL_B — статическую
        // память, которая переживает эту функцию, так что 'static здесь корректно,
        // хотя сам `buf` (обёртка DmaRxBuf) как локальная переменная уже дропнется.
        Ok(unsafe { core::slice::from_raw_parts(ptr, len) })
    }
}
