use crate::sensors::{Sensor, SensorResult};
use esp_hal::Blocking;
use esp_hal::delay::Delay;
use esp_hal::i2c::master::I2c;
use esp_println::println;

const I2C_ADDR: u8 = 0x3C;
const FORMAT_RGB565: &[[u16; 2]] = &[[0x501F, 0x01], [0x4300, 0x61]];

pub struct Ov5640 {
    // публичные поля состояния, юзер читает/пишет напрямую
}

impl Sensor for Ov5640 {
    fn new() -> Self {
        Self {}
    }

    fn init(&mut self, i2c: &mut I2c<'_, Blocking>) -> SensorResult<()> {
        let mut id_bytes = [0u8; 2];
        i2c.write_read(I2C_ADDR, &[0x30, 0x0A], &mut id_bytes)?;

        let read_id = ((id_bytes[0] as u16) << 8) | (id_bytes[1] as u16);
        if read_id != 0x5640 {
            return Err(crate::sensors::SensorError::SensorMismatch(read_id));
        }
        println!("[sensor] OV5640 ID match: 0x{:04X}", read_id);

        const REG_DLY: u16 = 0xFFFF;

        const DEFAULT_REGS: &[[u16; 2]] = &[
            [0x3008, 0x82], // software reset
            [REG_DLY, 10],  // delay 10ms
            [0x3008, 0x42], // power down
            // enable pll
            [0x3103, 0x13],
            // io direction
            [0x3017, 0xFF],
            [0x3018, 0xFF],
            [0x3020, 0xC3], // DRIVE_CAPABILITY
            [0x3021, 0x21], // CLOCK_POL_CONTROL
            [0x4713, 0x02], // jpg mode select
            [0x5001, 0x83], // ISP_CONTROL_01: turn color matrix, awb and SDE
            // sys reset
            [0x3000, 0x20], // reset MCU
            [REG_DLY, 10],  // delay 10ms
            [0x3002, 0x1C],
            // clock enable
            [0x3004, 0xFF],
            [0x3006, 0xC3],
            // isp control
            [0x5000, 0xA7],
            [0x5001, 0xA3], // ISP_CONTROL_01 + scaling?
            [0x5003, 0x08], // special_effect
            // unknown
            [0x370C, 0x02], // !!IMPORTANT
            [0x3634, 0x40], // !!IMPORTANT
            // AEC/AGC
            [0x3A02, 0x03],
            [0x3A03, 0xD8],
            [0x3A08, 0x01],
            [0x3A09, 0x27],
            [0x3A0A, 0x00],
            [0x3A0B, 0xF6],
            [0x3A0D, 0x04],
            [0x3A0E, 0x03],
            [0x3A0F, 0x30], // ae_level
            [0x3A10, 0x28], // ae_level
            [0x3A11, 0x60], // ae_level
            [0x3A13, 0x43],
            [0x3A14, 0x03],
            [0x3A15, 0xD8],
            [0x3A18, 0x00], // gainceiling
            [0x3A19, 0xF8], // gainceiling
            [0x3A1B, 0x30], // ae_level
            [0x3A1E, 0x26], // ae_level
            [0x3A1F, 0x14], // ae_level
            // vcm debug
            [0x3600, 0x08],
            [0x3601, 0x33],
            // 50/60Hz
            [0x3C01, 0xA4],
            [0x3C04, 0x28],
            [0x3C05, 0x98],
            [0x3C06, 0x00],
            [0x3C07, 0x08],
            [0x3C08, 0x00],
            [0x3C09, 0x1C],
            [0x3C0A, 0x9C],
            [0x3C0B, 0x40],
            [0x460C, 0x22], // disable jpeg footer
            // BLC
            [0x4001, 0x02],
            [0x4004, 0x02],
            // AWB
            [0x5180, 0xFF],
            [0x5181, 0xF2],
            [0x5182, 0x00],
            [0x5183, 0x14],
            [0x5184, 0x25],
            [0x5185, 0x24],
            [0x5186, 0x09],
            [0x5187, 0x09],
            [0x5188, 0x09],
            [0x5189, 0x75],
            [0x518a, 0x54],
            [0x518b, 0xE0],
            [0x518c, 0xB2],
            [0x518d, 0x42],
            [0x518e, 0x3D],
            [0x518f, 0x56],
            [0x5190, 0x46],
            [0x5191, 0xF8],
            [0x5192, 0x04],
            [0x5193, 0x70],
            [0x5194, 0xF0],
            [0x5195, 0xF0],
            [0x5196, 0x03],
            [0x5197, 0x01],
            [0x5198, 0x04],
            [0x5199, 0x12],
            [0x519a, 0x04],
            [0x519b, 0x00],
            [0x519c, 0x06],
            [0x519d, 0x82],
            [0x519e, 0x38],
            // color matrix (Saturation)
            [0x5381, 0x1E],
            [0x5382, 0x5B],
            [0x5383, 0x08],
            [0x5384, 0x0A],
            [0x5385, 0x7E],
            [0x5386, 0x88],
            [0x5387, 0x7C],
            [0x5388, 0x6C],
            [0x5389, 0x10],
            [0x538a, 0x01],
            [0x538b, 0x98],
            // CIP control (Sharpness)
            [0x5300, 0x10], // sharpness
            [0x5301, 0x10], // sharpness
            [0x5302, 0x18], // sharpness
            [0x5303, 0x19], // sharpness
            [0x5304, 0x10],
            [0x5305, 0x10],
            [0x5306, 0x08], // denoise
            [0x5307, 0x16],
            [0x5308, 0x40],
            [0x5309, 0x10], // sharpness
            [0x530a, 0x10], // sharpness
            [0x530b, 0x04], // sharpness
            [0x530c, 0x06], // sharpness
            // GAMMA
            [0x5480, 0x01],
            [0x5481, 0x00],
            [0x5482, 0x1E],
            [0x5483, 0x3B],
            [0x5484, 0x58],
            [0x5485, 0x66],
            [0x5486, 0x71],
            [0x5487, 0x7D],
            [0x5488, 0x83],
            [0x5489, 0x8F],
            [0x548a, 0x98],
            [0x548b, 0xA6],
            [0x548c, 0xB8],
            [0x548d, 0xCA],
            [0x548e, 0xD7],
            [0x548f, 0xE3],
            [0x5490, 0x1D],
            // Special Digital Effects (SDE) (UV adjust)
            [0x5580, 0x06], // enable brightness and contrast
            [0x5583, 0x40], // special_effect
            [0x5584, 0x10], // special_effect
            [0x5586, 0x20], // contrast
            [0x5587, 0x00], // brightness
            [0x5588, 0x00], // brightness
            [0x5589, 0x10],
            [0x558a, 0x00],
            [0x558b, 0xF8],
            [0x501D, 0x40], // enable manual offset of contrast
            // power on
            [0x3008, 0x02],
            // 50Hz
            [0x3C00, 0x04],
            [REG_DLY, 300],
            // [REGLIST_TAIL, 0x00],
        ];

        let delay = Delay::new();

        for entry in DEFAULT_REGS {
            let reg = entry[0];
            let val = entry[1] as u8;

            if reg == REG_DLY {
                delay.delay_millis(entry[1] as u32);
            } else {
                let bytes = [(reg >> 8) as u8, reg as u8, val];
                i2c.write(I2C_ADDR, &bytes)?;
            }
        }

        // i2c.write(0x3C, &[0x30, 0x08, 0x42])?;
        // i2c.write(0x3C, &[0x30, 0x17, 0xFF])?;
        // i2c.write(0x3C, &[0x30, 0x08, 0x02])?;

        Ok(())
    }

    fn set_qqvga(&mut self, i2c: &mut I2c<'_, Blocking>) -> SensorResult<()> {
        const FORMAT_RGB565: &[[u16; 2]] = &[[0x501F, 0x01], [0x4300, 0x61]];

        const QQVGA_WINDOW: &[[u16; 2]] = &[
            [0x3800, 0x00],
            [0x3801, 0x00], // X start
            [0x3802, 0x00],
            [0x3803, 0x00], // Y start
            [0x3804, 0x0A],
            [0x3805, 0x3F], // X end = 2623
            [0x3806, 0x07],
            [0x3807, 0x9F], // Y end = 1951
            [0x3808, 0x00],
            [0x3809, 0xA0], // output width = 160
            [0x380A, 0x00],
            [0x380B, 0x78], // output height = 120
            [0x380C, 0x08],
            [0x380D, 0x0C], // total X = 2060
            [0x380E, 0x03],
            [0x380F, 0xD8], // total Y = 984
            [0x3810, 0x00],
            [0x3811, 0x10], // offset X = 16
            [0x3812, 0x00],
            [0x3813, 0x08], // offset Y = 8
            [0x3820, 0x01], // reg20 (binning, no flip)
            [0x3821, 0x01], // reg21 (binning, no mirror)
            [0x4514, 0xAA],
            [0x4520, 0x0B],
            [0x3814, 0x31], // X increment
            [0x3815, 0x31], // Y increment
        ];

        const QQVGA_PLL: &[[u16; 2]] = &[
            [0x3039, 0x00],
            [0x3034, 0x1A],
            [0x3035, 0x11],
            [0x3036, 0x14],
            [0x3037, 0x01],
            [0x3108, 0x16],
            [0x3824, 0x08],
            [0x460C, 0x22],
            [0x3103, 0x13],
        ];

        for entry in FORMAT_RGB565.iter().chain(QQVGA_WINDOW).chain(QQVGA_PLL) {
            let bytes = [(entry[0] >> 8) as u8, entry[0] as u8, entry[1] as u8];
            i2c.write(I2C_ADDR, &bytes)?;
        }
        Ok(())
    }

    fn set_fullhd(&mut self, i2c: &mut I2c<'_, Blocking>) -> SensorResult<()> {
        const FULLHD_WINDOW: &[[u16; 2]] = &[
            [0x3800, 0x00],
            [0x3801, 0x00], // X start = 0
            [0x3802, 0x00],
            [0x3803, 0xF0], // Y start = 240
            [0x3804, 0x0A],
            [0x3805, 0x3F], // X end = 2623
            [0x3806, 0x06],
            [0x3807, 0xAF], // Y end = 1711
            [0x3808, 0x07],
            [0x3809, 0x80], // output width = 1920
            [0x380A, 0x04],
            [0x380B, 0x38], // output height = 1080
            [0x380C, 0x0B],
            [0x380D, 0x1C], // total X = 2844 (без биннинга — полное значение)
            [0x380E, 0x05],
            [0x380F, 0xD0], // total Y = 1488
            [0x3810, 0x00],
            [0x3811, 0x20], // offset X = 32
            [0x3812, 0x00],
            [0x3813, 0x10], // offset Y = 16
            [0x3820, 0x40], // reg20 (без биннинга)
            [0x3821, 0x00], // reg21 (без биннинга)
            [0x4514, 0x88], // normal, без биннинга
            [0x4520, 0x10],
            [0x3814, 0x11], // X increment (без биннинга: odd=1, even=1)
            [0x3815, 0x11], // Y increment
        ];

        const FULLHD_PLL: &[[u16; 2]] = &[
            [0x3039, 0x00],
            [0x3034, 0x1A],
            [0x3035, 0x11],
            [0x3036, 0x0A], // multiplier = 10
            [0x3037, 0x02], // pre_div = 2
            [0x3108, 0x16],
            [0x3824, 0x02], // pclk_div = 2
            [0x460C, 0x22],
            [0x3103, 0x13],
        ];

        for entry in FORMAT_RGB565.iter().chain(FULLHD_WINDOW).chain(FULLHD_PLL) {
            let bytes = [(entry[0] >> 8) as u8, entry[0] as u8, entry[1] as u8];
            i2c.write(I2C_ADDR, &bytes)?;
        }
        Ok(())
    }
}
