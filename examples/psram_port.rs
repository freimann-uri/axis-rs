//! Isolated octal PSRAM init for ESP32-S3 (esp-hal 1.2.2, no_std).
//!
//! Port of `psram_manual_init.c` (ESP-IDF v6.0.1 `esp_psram_impl_octal.c` + `esp_psram.c` + `mspi_timing_*`).
//! The same 8 stages, one function per stage, **without stage 4 (MSPI timing tuning)**: the stage numbers are kept
//! on purpose, so they match the C code. Nothing from esp-hal peripherals/PSRAM driver is used: registers are
//! accessed by address (same as in the C code), only three ROM functions are called (as IDF and esp-hal do).
//!
//! Needs in the application: `esp-hal` 1.2.2 (for `ram` and `time`) and `esp-println` (console).
//! Do NOT enable esp-hal's own PSRAM init (`Config::with_psram`) at the same time.
//!
//! IMPORTANT: `FLASH_FREQUENCY_MHZ` must match the real flash frequency of the image (espflash `--flash-freq`),
//! stage 5 sets the flash clock to it.
//!
//! Stages:
//!   1 `psram_stage1_pins`       pins, CS timing, ECC off
//!   2 `psram_stage2_low_speed`  20 MHz mode, VAR_DUMMY on SPI1, DTR swap
//!   3 `psram_stage3_chip`       mode registers, connection check, size, chip info log
//!   4 (tuning)                  not ported: zero tuning (din mode 0, din num 0, extra dummy 0) is applied in 2 and 5
//!   5 `psram_stage5_high_speed` core clock, flash/PSRAM clocks, tuning regs, flash regs
//!   6 `psram_stage6_spi_phases` SPI0 cmd/addr/dummy/DDR/OCT for the cache
//!   7 `psram_stage7_mapping`    MMU entries, CPU DBUS enable
//!   8 `psram_stage8_memtest`    one pass pseudo-random memory test
//!
//! Usage:
//! ```ignore
//! let psram = psram_manual_init::xesp_psram_init().expect("PSRAM init failed");
//! let ptr = psram.vaddr as *mut u32;   // psram.mapped_size bytes
//! ```

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::main;
use esp_println::println;

// mod psram_manual_init;

esp_bootloader_esp_idf::esp_app_desc!();

use core::ptr::{read_volatile, write_volatile};

// ---- configuration (80 MHz PSRAM needs the 160 MHz MSPI core clock, like in IDF) ----
const FLASH_FREQUENCY_MHZ: u32 = 80;
const PSRAM_FREQUENCY_MHZ: u32 = 80;
const MSPI_TIMING_CORE_CLOCK_MHZ: u32 = if PSRAM_FREQUENCY_MHZ == 80 { 160 } else { 80 };
const MSPI_TIMING_LL_CORE_CLOCK_MHZ_DEFAULT: u32 = 80;

// ---- PSRAM chip constants (esp_psram_impl_octal.c) ----
const OPI_PSRAM_SYNC_READ: u32 = 0x0000;
const OPI_PSRAM_SYNC_WRITE: u32 = 0x8080;
const OPI_PSRAM_REG_READ: u32 = 0x4040;
const OPI_PSRAM_REG_WRITE: u32 = 0xC0C0;
const OCT_PSRAM_RD_CMD_BITLEN: u32 = 16;
const OCT_PSRAM_WR_CMD_BITLEN: u32 = 16;
const OCT_PSRAM_ADDR_BITLEN: u32 = 32;
const OCT_PSRAM_RD_DUMMY_BITLEN: u32 = 2 * (10 - 1);
const OCT_PSRAM_RD_REG_DUMMY_BITLEN: u32 = 2 * (5 - 1);
const OCT_PSRAM_WR_DUMMY_BITLEN: u32 = 2 * (5 - 1);
const OCT_PSRAM_VENDOR_ID_AP: u8 = 0xD;
const OCT_PSRAM_VENDOR_ID_UNILC: u8 = 0x1A;
const OCT_PSRAM_CS_SETUP_TIME: u32 = 3;
const OCT_PSRAM_CS_HOLD_TIME: u32 = 3;
const OCT_PSRAM_CS_HOLD_DELAY: u32 = 2;
const OCT_PSRAM_REF_DATA: u32 = 0x5a6b7c8d;
const ESP_ROM_SPIFLASH_OPI_DTR_MODE: u8 = 7;

const MMU_PAGE_SIZE: usize = 0x10000;
const PSRAM_MAP_MAX_SIZE: usize = 16 * 1024 * 1024;
// const SOC_DRAM0_CACHE_ADDRESS_HIGH: usize = 0x3E00_0000; // PSRAM is mapped to the top of the DBUS window
const SOC_MMU_VADDR_MASK: usize = 0x1FF_FFFF;
const SOC_MMU_ACCESS_SPIRAM: u32 = 1 << 15;
const SOC_MMU_VALID: u32 = 0;

// ---- register addresses (soc/esp32s3) ----
const DR_REG_SPI0_BASE: usize = 0x6000_3000;
const DR_REG_MMU_TABLE: usize = 0x600C_5000;
const DR_REG_EXTMEM_BASE: usize = 0x600C_4000;
// const DR_REG_SYSCON_BASE: usize = 0x6002_6000;
const IO_MUX_GPIO26_REG: usize = 0x6000_9000 + 0x6c; // CS1 pin (SPICS1)
const EXTMEM_DCACHE_CTRL1_REG: usize = DR_REG_EXTMEM_BASE + 0x4;
// const SYSCON_SRAM_ACE0_ATTR_REG: usize = DR_REG_SYSCON_BASE + 0x58;

/// SPI_MEM register of SPI0 (i = 0) or SPI1 (i = 1): `REG_SPI_MEM_BASE(i) + offset`
const fn spi_mem(i: usize, offset: usize) -> usize {
    DR_REG_SPI0_BASE - i * 0x1000 + offset
}
const fn spi_mem_clock_reg(i: usize) -> usize {
    spi_mem(i, 0x14)
}
const fn spi_mem_cache_fctrl_reg(i: usize) -> usize {
    spi_mem(i, 0x3C)
}
const fn spi_mem_cache_sctrl_reg(i: usize) -> usize {
    spi_mem(i, 0x40)
}
const fn spi_mem_sram_cmd_reg(i: usize) -> usize {
    spi_mem(i, 0x44)
}
const fn spi_mem_sram_drd_cmd_reg(i: usize) -> usize {
    spi_mem(i, 0x48)
}
const fn spi_mem_sram_dwr_cmd_reg(i: usize) -> usize {
    spi_mem(i, 0x4C)
}
const fn spi_mem_sram_clk_reg(i: usize) -> usize {
    spi_mem(i, 0x50)
}
const fn spi_mem_smem_timing_cali_reg(i: usize) -> usize {
    spi_mem(i, 0xBC)
}
const fn spi_mem_smem_din_mode_reg(i: usize) -> usize {
    spi_mem(i, 0xC0)
}
const fn spi_mem_smem_din_num_reg(i: usize) -> usize {
    spi_mem(i, 0xC4)
}
const fn spi_mem_smem_ac_reg(i: usize) -> usize {
    spi_mem(i, 0xDC)
}
const fn spi_mem_ddr_reg(i: usize) -> usize {
    spi_mem(i, 0xE0)
}
const fn spi_mem_smem_ddr_reg(i: usize) -> usize {
    spi_mem(i, 0xE4)
}
const fn spi_mem_core_clk_sel_reg(i: usize) -> usize {
    spi_mem(i, 0xEC)
}
const fn spi_mem_date_reg(i: usize) -> usize {
    spi_mem(i, 0x3FC)
}

const BIT0: u32 = 1 << 0;
const BIT1: u32 = 1 << 1;
const BIT2: u32 = 1 << 2;
const BIT3: u32 = 1 << 3;
const BIT4: u32 = 1 << 4;
const BIT5: u32 = 1 << 5;
// const BIT8: u32 = 1 << 8;
const BIT18: u32 = 1 << 18;
const BIT19: u32 = 1 << 19;
const BIT20: u32 = 1 << 20;
const BIT21: u32 = 1 << 21;
const BIT22: u32 = 1 << 22;
const BIT31: u32 = 1 << 31;

// SMEM din mode: 9 fields x 3 bits (DIN0..DIN7, DINS); din num: 9 fields x 2 bits
const SMEM_DIN_MODE_MASK: u32 = 0x07FF_FFFF;
const SMEM_DIN_MODE_ONES: u32 = 0x0124_9249;
const SMEM_DIN_NUM_MASK: u32 = 0x0003_FFFF;
const SMEM_DIN_NUM_ONES: u32 = 0x0001_5555;

// ---- ROM functions (same as IDF/esp-hal use) ----
unsafe extern "C" {
    fn esp_rom_opiflash_pin_config();
    fn esp_rom_opiflash_exec_cmd(
        spi_num: u32,
        mode: u8,
        cmd: u32,
        cmd_bit_len: u32,
        addr: u32,
        addr_bit_len: u32,
        dummy_bits: u32,
        mosi_data: *const u8,
        mosi_bit_len: u32,
        miso_data: *mut u8,
        miso_bit_len: u32,
        cs_mask: u32,
        is_write_erase_operation: bool,
    );
    fn esp_rom_spi_set_dtr_swap_mode(spi: u32, wr_swap: bool, rd_swap: bool);
    #[allow(non_snake_case)]
    fn Cache_Resume_DCache(param: u32);
}

// ---- console: same format as the ESP-IDF log ("I (284) octal_psram: ...") ----
macro_rules! psram_log {
    ($level:literal, $tag:expr, $($arg:tt)*) => {{
        let ms = esp_hal::time::Instant::now().duration_since_epoch().as_millis();
        esp_println::println!("{} ({}) {}: {}", $level, ms, $tag, format_args!($($arg)*));
    }};
}

// ---- tiny register helpers (SET_PERI_REG_MASK & co); the addresses are fixed MMIO registers ----
#[inline(always)]
fn rd(addr: usize) -> u32 {
    unsafe { read_volatile(addr as *const u32) }
}
#[inline(always)]
fn wr(addr: usize, val: u32) {
    unsafe { write_volatile(addr as *mut u32, val) }
}
#[inline(always)]
fn set_mask(addr: usize, mask: u32) {
    wr(addr, rd(addr) | mask);
}
#[inline(always)]
fn clr_mask(addr: usize, mask: u32) {
    wr(addr, rd(addr) & !mask);
}
/// SET_PERI_REG_BITS: `mask` is the unshifted field mask
#[inline(always)]
fn set_field(addr: usize, mask: u32, shift: u32, val: u32) {
    wr(
        addr,
        (rd(addr) & !(mask << shift)) | ((val & mask) << shift),
    );
}

/// Calls `esp_rom_opiflash_exec_cmd` for the PSRAM (SPI1, OPI DTR mode, CS1)
#[inline(always)]
fn opi_exec(
    cmd: u32,
    cmd_len: u32,
    addr: u32,
    addr_len: u32,
    dummy: u32,
    mosi: *const u8,
    mosi_bits: u32,
    miso: *mut u8,
    miso_bits: u32,
) {
    unsafe {
        esp_rom_opiflash_exec_cmd(
            1,
            ESP_ROM_SPIFLASH_OPI_DTR_MODE,
            cmd,
            cmd_len,
            addr,
            addr_len,
            dummy,
            mosi,
            mosi_bits,
            miso,
            miso_bits,
            BIT1,
            false,
        );
    }
}

// ---- MSPI clocks (mspi_timing_ll_* / mspi_timing_config.c): core clock 80/120/160/240 MHz, module clock = core / div ----
#[inline(always)]
fn mspi_set_core_clock(core_clk_mhz: u32) {
    let val = match core_clk_mhz {
        80 => 0,
        120 => 1,
        160 => 2,
        _ => 3,
    };
    set_field(spi_mem_core_clk_sel_reg(0), 0x3, 0, val);
}

#[inline(always)]
fn mspi_set_flash_clock(spi_num: usize, freqdiv: u32) {
    if freqdiv == 1 {
        wr(spi_mem_clock_reg(spi_num), BIT31); // SPI_MEM_CLK_EQU_SYSCLK
    } else {
        wr(
            spi_mem_clock_reg(spi_num),
            ((freqdiv - 1) << 16) | ((freqdiv / 2 - 1) << 8) | (freqdiv - 1),
        );
    }
}

#[inline(always)]
fn mspi_set_psram_clock(spi_num: usize, freqdiv: u32) {
    if freqdiv == 1 {
        wr(spi_mem_sram_clk_reg(spi_num), BIT31); // SPI_MEM_SCLK_EQU_SYSCLK
    } else {
        wr(
            spi_mem_sram_clk_reg(spi_num),
            ((freqdiv - 1) << 16) | ((freqdiv / 2 - 1) << 8) | (freqdiv - 1),
        );
    }
}

/// PSRAM input timing of SPI0: din mode / din num of all data lines + extra dummy cycles
struct Tuning {
    din_mode: u32,
    din_num: u32,
    extra_dummy: u32,
}

/// No tuning (stage 4 is not ported): the same as the zero config in the IDF tuning table
const NO_TUNING: Tuning = Tuning {
    din_mode: 0,
    din_num: 0,
    extra_dummy: 0,
};

#[inline(always)]
fn mspi_set_psram_tuning(t: &Tuning) {
    let mode_reg = spi_mem_smem_din_mode_reg(0);
    wr(
        mode_reg,
        (rd(mode_reg) & !SMEM_DIN_MODE_MASK) | (t.din_mode * SMEM_DIN_MODE_ONES),
    );
    let num_reg = spi_mem_smem_din_num_reg(0);
    wr(
        num_reg,
        (rd(num_reg) & !SMEM_DIN_NUM_MASK) | (t.din_num * SMEM_DIN_NUM_ONES),
    );
    // octal PSRAM extra dummy
    let cali = spi_mem_smem_timing_cali_reg(0);
    if t.extra_dummy > 0 {
        set_mask(cali, BIT1); // SPI_MEM_SPI_SMEM_TIMING_CALI
        set_field(cali, 0x7, 2, t.extra_dummy);
    } else {
        clr_mask(cali, BIT1);
        set_field(cali, 0x7, 2, 0);
    }
}

/// mspi_timing_enter_low_speed_mode(true): everything to 20 MHz (core 80 MHz / 4), tuning regs cleared
#[inline(always)]
fn mspi_enter_low_speed() {
    let div = MSPI_TIMING_LL_CORE_CLOCK_MHZ_DEFAULT / 20;
    mspi_set_core_clock(MSPI_TIMING_LL_CORE_CLOCK_MHZ_DEFAULT);
    mspi_set_flash_clock(0, div);
    mspi_set_flash_clock(1, div);
    mspi_set_psram_clock(0, div);
    mspi_set_psram_tuning(&NO_TUNING);
}

/// mspi_timing_enter_high_speed_mode(true): flash and PSRAM at the selected frequencies, tuning regs applied
#[inline(always)]
fn mspi_enter_high_speed() {
    mspi_set_core_clock(MSPI_TIMING_CORE_CLOCK_MHZ);
    mspi_set_flash_clock(0, MSPI_TIMING_CORE_CLOCK_MHZ / FLASH_FREQUENCY_MHZ);
    mspi_set_flash_clock(1, MSPI_TIMING_CORE_CLOCK_MHZ / FLASH_FREQUENCY_MHZ);
    mspi_set_psram_clock(0, MSPI_TIMING_CORE_CLOCK_MHZ / PSRAM_FREQUENCY_MHZ);
    mspi_set_psram_tuning(&NO_TUNING);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PsramError {
    /// ESP_ERR_NOT_SUPPORTED: chip not connected, or wrong PSRAM line mode
    NotSupported,
    /// ESP_ERR_INVALID_STATE: unknown density
    InvalidState,
    /// ESP_FAIL: memory test failed
    MemtestFailed,
}

/// State shared between the stages
#[derive(Debug, Clone, Copy, Default)]
pub struct PsramState {
    /// physical PSRAM size in bytes (s_psram_size)
    pub size: usize,
    /// virtual address of the mapped region
    pub vaddr: usize,
    /// size of the mapped region in bytes
    pub mapped_size: usize,
}

/*---------------------------------------------------------------------------------
 * Stage 1: pins, CS timing, ECC
 *-------------------------------------------------------------------------------*/
#[esp_hal::ram]
pub fn psram_stage1_pins(_st: &mut PsramState) -> Result<(), PsramError> {
    const TAG: &str = "octal_psram";
    const fn io(pin: usize) -> usize {
        0x6000_9000 + 0x04 + pin * 4
    }
    const AC: usize = spi_mem_smem_ac_reg(0);
    const DATE: usize = spi_mem_date_reg(0);

    // (регистр, маска поля без сдвига, сдвиг, значение)
    const REGS: [(usize, u32, u32, u32); 19] = [
        (IO_MUX_GPIO26_REG, 0x7, 12, 0), // CS1: функция SPICS1
        (IO_MUX_GPIO26_REG, 0x3, 10, 3), // CS1: drive
        (DATE, 1, 4, 1),                 // CLK pad drive control
        (DATE, 0x3, 0, 3),               // PSRAM CLK drive
        (DATE, 0x3, 2, 3),               // flash CLK drive
        (io(27), 0x3, 10, 3),
        (io(28), 0x3, 10, 3),
        (io(31), 0x3, 10, 3),
        (io(32), 0x3, 10, 3),
        (io(33), 0x3, 10, 3),
        (io(34), 0x3, 10, 3),
        (io(35), 0x3, 10, 3),
        (io(36), 0x3, 10, 3),
        (io(37), 0x3, 10, 3),
        (AC, 1, 1, 1), // CS_HOLD
        (AC, 1, 0, 1), // CS_SETUP
        (AC, 0x1F, 7, OCT_PSRAM_CS_HOLD_TIME),
        (AC, 0x1F, 2, OCT_PSRAM_CS_SETUP_TIME),
        (AC, 0x3F, 25, OCT_PSRAM_CS_HOLD_DELAY),
    ];

    unsafe { esp_rom_opiflash_pin_config() };

    for &(addr, mask, shift, val) in REGS.iter() {
        set_field(addr, mask, shift, val);
    }

    // отдельный проход: rd() = read_volatile, читается сама периферия
    for &(addr, mask, shift, val) in REGS.iter() {
        if (rd(addr) >> shift) & mask != val {
            psram_log!("E", TAG, "stage 1: запись регистров не подтвердилась");
            return Err(PsramError::NotSupported);
        }
    }

    psram_log!("I", TAG, "stage 1: OK");
    Ok(())
}

/*---------------------------------------------------------------------------------
 * Stage 2: slow mode (20 MHz) to talk to the PSRAM chip
 *-------------------------------------------------------------------------------*/
#[esp_hal::ram]
pub fn psram_stage2_low_speed(_st: &mut PsramState) -> Result<(), PsramError> {
    mspi_enter_low_speed();

    // set to variable dummy mode (SPI_MEM_SPI_FMEM_VAR_DUMMY of SPI1)
    set_mask(spi_mem_ddr_reg(1), BIT1);
    unsafe { esp_rom_spi_set_dtr_swap_mode(1, false, false) };
    psram_log!("D", "dbg", "s2 clocks done");
    Ok(())
}

// static FLASH_PROBE: [u32; 256] = [0xA5C3_5A3C; 256];

/*---------------------------------------------------------------------------------
 * Stage 3: PSRAM chip: mode registers, connection check, size
 *-------------------------------------------------------------------------------*/
/// Mode registers of the PSRAM, same layout as `opi_psram_mode_reg_t` in IDF (6 consecutive bytes)
#[repr(C)]
#[derive(Default)]
struct OpiPsramModeReg {
    mr0: u8,
    mr1: u8,
    mr2: u8,
    mr3: u8,
    mr4: u8,
    mr8: u8,
}

#[esp_hal::ram]
pub fn psram_stage3_chip(st: &mut PsramState) -> Result<(), PsramError> {
    const TAG: &str = "octal_psram";
    let null = core::ptr::null();
    let null_mut = core::ptr::null_mut();

    // s_init_psram_mode_reg: lt = 1 (fixed latency), read_latency = 2 (10 cycles), drive_str = 0
    // read MR0~1, modify, write
    let mut mr = OpiPsramModeReg::default();
    wr(spi_mem(1, 0x58), 0xA5A5_A5A5);
    opi_exec(
        OPI_PSRAM_REG_READ,
        16,
        0x0,
        32,
        OCT_PSRAM_RD_REG_DUMMY_BITLEN,
        null,
        0,
        &mut mr.mr0,
        16,
    );
    mr.mr0 = (mr.mr0 & !0x3F) | (1 << 5) | (2 << 2) | 0;
    opi_exec(
        OPI_PSRAM_REG_WRITE,
        16,
        0x0,
        32,
        0,
        &mr.mr0,
        16,
        null_mut,
        0,
    );

    // s_check_psram_connected: write and read back a reference word
    let ref_data: u32 = OCT_PSRAM_REF_DATA;
    let mut exp_data: u32 = 0;

    opi_exec(
        OPI_PSRAM_SYNC_WRITE,
        OCT_PSRAM_WR_CMD_BITLEN,
        0x0,
        OCT_PSRAM_ADDR_BITLEN,
        OCT_PSRAM_WR_DUMMY_BITLEN,
        &ref_data as *const u32 as *const u8,
        32,
        null_mut,
        0,
    );
    wr(spi_mem(1, 0x58), 0xA5A5_A5A5); // SPI1 W0
    opi_exec(
        OPI_PSRAM_SYNC_READ,
        OCT_PSRAM_WR_CMD_BITLEN,
        0x0,
        OCT_PSRAM_ADDR_BITLEN,
        OCT_PSRAM_RD_DUMMY_BITLEN,
        null,
        0,
        &mut exp_data as *mut u32 as *mut u8,
        32,
    );

    if exp_data != ref_data {
        psram_log!(
            "E",
            TAG,
            "check: wrote {:08X}, read {:08X}",
            ref_data,
            exp_data
        );
        psram_log!(
            "E",
            TAG,
            "PSRAM chip is not connected, or wrong PSRAM line mode"
        );
        return Err(PsramError::NotSupported);
    }

    // s_get_psram_mode_reg
    opi_exec(
        OPI_PSRAM_REG_READ,
        16,
        0x0,
        32,
        OCT_PSRAM_RD_REG_DUMMY_BITLEN,
        null,
        0,
        &mut mr.mr0,
        16,
    );
    opi_exec(
        OPI_PSRAM_REG_READ,
        16,
        0x2,
        32,
        OCT_PSRAM_RD_REG_DUMMY_BITLEN,
        null,
        0,
        &mut mr.mr2,
        16,
    );
    opi_exec(
        OPI_PSRAM_REG_READ,
        16,
        0x4,
        32,
        OCT_PSRAM_RD_REG_DUMMY_BITLEN,
        null,
        0,
        &mut mr.mr4,
        8,
    );
    opi_exec(
        OPI_PSRAM_REG_READ,
        16,
        0x8,
        32,
        OCT_PSRAM_RD_REG_DUMMY_BITLEN,
        null,
        0,
        &mut mr.mr8,
        8,
    );

    let drive_str = mr.mr0 & 0x3;
    let read_latency = (mr.mr0 >> 2) & 0x7;
    let lt = (mr.mr0 >> 5) & 0x1;
    let vendor_id = mr.mr1 & 0x1F;
    let density = mr.mr2 & 0x7;
    let dev_id = (mr.mr2 >> 3) & 0x3;
    let gb = (mr.mr2 >> 7) & 0x1;
    let srf = (mr.mr3 >> 5) & 0x1;
    let vcc = (mr.mr3 >> 6) & 0x1;
    let bl = mr.mr8 & 0x3;
    let bt = (mr.mr8 >> 2) & 0x1;

    if vendor_id != OCT_PSRAM_VENDOR_ID_AP && vendor_id != OCT_PSRAM_VENDOR_ID_UNILC {
        psram_log!(
            "W",
            TAG,
            "PSRAM ID read error: 0x{:08x}, fallback to use default driver pattern",
            vendor_id
        );
    }

    // let mut probe_bad = 0u32;
    // for i in 0..FLASH_PROBE.len() {
    //     if unsafe { core::ptr::read_volatile(&FLASH_PROBE[i]) } != 0xA5C3_5A3C {
    //         probe_bad += 1;
    //     }
    // }
    psram_log!("D", "dbg", "****This log prevents panic****");
    // psram_log!("D", "dbg", "flash probe bad = {} of 256", probe_bad);

    psram_log!(
        "I",
        TAG,
        "vendor id    : 0x{:02x} ({})",
        vendor_id,
        if vendor_id == 0x0d {
            "AP"
        } else if vendor_id == 0x1a {
            "UnilC"
        } else {
            "UNKNOWN"
        }
    );
    psram_log!(
        "I",
        TAG,
        "dev id       : 0x{:02x} (generation {})",
        dev_id,
        dev_id + 1
    );
    psram_log!(
        "I",
        TAG,
        "density      : 0x{:02x} ({} Mbit)",
        density,
        match density {
            0x1 => 32,
            0x3 => 64,
            0x5 => 128,
            0x7 => 256,
            _ => 0,
        }
    );
    psram_log!(
        "I",
        TAG,
        "good-die     : 0x{:02x} ({})",
        gb,
        if gb == 1 { "Pass" } else { "Fail" }
    );
    psram_log!(
        "I",
        TAG,
        "Latency      : 0x{:02x} ({})",
        lt,
        if lt == 1 { "Fixed" } else { "Variable" }
    );
    psram_log!(
        "I",
        TAG,
        "VCC          : 0x{:02x} ({})",
        vcc,
        if vcc == 1 { "3V" } else { "1.8V" }
    );
    psram_log!(
        "I",
        TAG,
        "SRF          : 0x{:02x} ({} Refresh)",
        srf,
        if srf == 1 { "Fast" } else { "Slow" }
    );
    psram_log!(
        "I",
        TAG,
        "BurstType    : 0x{:02x} ({} Wrap)",
        bt,
        if bt == 1 && bl != 3 { "Hybrid" } else { "" }
    );
    psram_log!(
        "I",
        TAG,
        "BurstLen     : 0x{:02x} ({} Byte)",
        bl,
        match bl {
            0x00 => 16,
            0x01 => 32,
            0x10 => 64,
            _ => 1024,
        }
    );
    psram_log!(
        "I",
        TAG,
        "Readlatency  : 0x{:02x} ({} cycles@{})",
        read_latency,
        read_latency * 2 + 6,
        if lt == 1 { "Fixed" } else { "Variable" }
    );
    psram_log!(
        "I",
        TAG,
        "DriveStrength: 0x{:02x} (1/{})",
        drive_str,
        match drive_str {
            0x00 => 1,
            0x01 => 2,
            0x02 => 4,
            _ => 8,
        }
    );

    st.size = match density {
        0x1 => 4 * 1024 * 1024,
        0x3 => 8 * 1024 * 1024,
        0x5 => 16 * 1024 * 1024,
        0x7 => 32 * 1024 * 1024,
        0x6 => 64 * 1024 * 1024,
        _ => 0,
    };

    if st.size == 0 {
        return Err(PsramError::InvalidState);
    }

    Ok(())
}

// Stage 4 (MSPI timing tuning) is not ported: it is only needed for 80 MHz on some boards. Zero tuning is used.

/*---------------------------------------------------------------------------------
 * Stage 5: high speed mode: clocks, tuning regs, flash regs
 *-------------------------------------------------------------------------------*/
#[esp_hal::ram]
pub fn psram_stage5_high_speed(_st: &mut PsramState) -> Result<(), PsramError> {
    mspi_enter_high_speed();

    // spi_flash_set_vendor_required_regs(): set back MSPI registers of the (non-octal) flash after PSRAM init
    clr_mask(spi_mem_cache_fctrl_reg(1), BIT1); // SPI_MEM_CACHE_USR_CMD_4BYTE
    Ok(())
}

/*---------------------------------------------------------------------------------
 * Stage 6: SPI0 phases used by the cache to access PSRAM
 *-------------------------------------------------------------------------------*/
#[esp_hal::ram]
pub fn psram_stage6_spi_phases(_st: &mut PsramState) -> Result<(), PsramError> {
    let sctrl = spi_mem_cache_sctrl_reg(0);

    // Config Write CMD phase for SPI0 to access PSRAM
    set_mask(sctrl, BIT20); // SPI_MEM_CACHE_SRAM_USR_WCMD
    set_field(
        spi_mem_sram_dwr_cmd_reg(0),
        0xF,
        28,
        OCT_PSRAM_WR_CMD_BITLEN - 1,
    );
    set_field(spi_mem_sram_dwr_cmd_reg(0), 0xFFFF, 0, OPI_PSRAM_SYNC_WRITE);

    // Config Read CMD phase for SPI0 to access PSRAM
    set_mask(sctrl, BIT5); // SPI_MEM_CACHE_SRAM_USR_RCMD
    set_field(
        spi_mem_sram_drd_cmd_reg(0),
        0xF,
        28,
        OCT_PSRAM_RD_CMD_BITLEN - 1,
    );
    set_field(spi_mem_sram_drd_cmd_reg(0), 0xFFFF, 0, OPI_PSRAM_SYNC_READ);

    // Config ADDR phase
    set_field(sctrl, 0x3F, 14, OCT_PSRAM_ADDR_BITLEN - 1); // SPI_MEM_SRAM_ADDR_BITLEN
    set_mask(sctrl, BIT0); // SPI_MEM_CACHE_USR_SCMD_4BYTE

    // Config RD/WR Dummy phase
    set_mask(sctrl, BIT4 | BIT3); // SPI_MEM_USR_RD_SRAM_DUMMY | SPI_MEM_USR_WR_SRAM_DUMMY
    set_field(sctrl, 0x3F, 6, OCT_PSRAM_RD_DUMMY_BITLEN - 1); // SPI_MEM_SRAM_RDUMMY_CYCLELEN
    set_mask(spi_mem_smem_ddr_reg(0), BIT1); // SPI_MEM_SPI_SMEM_VAR_DUMMY
    set_field(sctrl, 0x3F, 22, OCT_PSRAM_WR_DUMMY_BITLEN - 1); // SPI_MEM_SRAM_WDUMMY_CYCLELEN

    clr_mask(spi_mem_smem_ddr_reg(0), BIT3 | BIT2); // SPI_MEM_SPI_SMEM_DDR_WDAT_SWP | SPI_MEM_SPI_SMEM_DDR_RDAT_SWP
    set_mask(spi_mem_smem_ddr_reg(0), BIT0); // SPI_MEM_SPI_SMEM_DDR_EN

    // SDUMMY_OUT | SCMD_OCT | SADDR_OCT | SDOUT_OCT | SDIN_OCT
    set_mask(
        spi_mem_sram_cmd_reg(0),
        BIT22 | BIT21 | BIT20 | BIT19 | BIT18,
    );
    set_mask(sctrl, BIT21); // SPI_MEM_SRAM_OCT

    unsafe { Cache_Resume_DCache(0) };
    Ok(())
}

/*---------------------------------------------------------------------------------
 * Stage 7: MMU mapping of the PSRAM and enabling the CPU DBUS
 *-------------------------------------------------------------------------------*/
#[esp_hal::ram]
pub fn psram_stage7_mapping(st: &mut PsramState) -> Result<(), PsramError> {
    let start_page: u32 = 0;
    let size_to_map = if st.size < PSRAM_MAP_MAX_SIZE {
        st.size
    } else {
        PSRAM_MAP_MAX_SIZE
    };

    // let vaddr = SOC_DRAM0_CACHE_ADDRESS_HIGH - size_to_map; // 64 KB aligned
    let vaddr = 0x3C02_0000; // как у esp-hal: записи MMU 2..129

    // mmu_hal_map_region(): entry = psram page | SOC_MMU_ACCESS_SPIRAM | SOC_MMU_VALID
    for i in 0..(size_to_map / MMU_PAGE_SIZE) {
        let entry_id = ((vaddr + i * MMU_PAGE_SIZE) & SOC_MMU_VADDR_MASK) >> 16;
        wr(
            DR_REG_MMU_TABLE + entry_id * 4,
            (start_page + i as u32) | SOC_MMU_ACCESS_SPIRAM | SOC_MMU_VALID,
        );
    }
    // cache_ll_l1_enable_bus() for both cores: without this the CPU DBUS stays shut and PSRAM access hangs
    clr_mask(EXTMEM_DCACHE_CTRL1_REG, BIT0 | BIT1); // EXTMEM_DCACHE_SHUT_CORE0_BUS | EXTMEM_DCACHE_SHUT_CORE1_BUS

    st.vaddr = vaddr;
    st.mapped_size = size_to_map;
    Ok(())
}

/*---------------------------------------------------------------------------------
 * Stage 8: strict memory test of the whole mapped region, one pass, every word: write all, then verify all.
 * Pseudo-random data (xorshift32): every word is different, so it catches data bit, address line and timing errors.
 * The region (8 MB) is much bigger than the data cache, so the verify reads come from PSRAM, not from cache.
 *-------------------------------------------------------------------------------*/
// #[esp_hal::ram]
pub fn psram_stage8_memtest(st: &mut PsramState) -> Result<(), PsramError> {
    const TAG: &str = "esp_psram";
    let spiram = st.vaddr as *mut u32;
    let words = st.mapped_size / core::mem::size_of::<u32>();
    let mut errct: usize = 0;
    let mut first_err: usize = 0;
    let mut first_exp: u32 = 0;
    let mut first_got: u32 = 0;

    let mut rnd: u32 = 0x2545F491;
    for p in 0..words {
        rnd ^= rnd << 13;
        rnd ^= rnd >> 17;
        rnd ^= rnd << 5;
        unsafe { write_volatile(spiram.add(p), rnd) };
    }

    rnd = 0x2545F491;
    for p in 0..words {
        rnd ^= rnd << 13;
        rnd ^= rnd >> 17;
        rnd ^= rnd << 5;
        let got = unsafe { read_volatile(spiram.add(p)) };

        if got != rnd {
            if errct == 0 {
                first_err = p;
                first_exp = rnd;
                first_got = got;
            }
            errct += 1;
        }
    }
    if errct != 0 {
        psram_log!(
            "E",
            TAG,
            "SPI SRAM memory test fail. {}/{} words failed, first @ {:X} (expected {:08X}, got {:08X})",
            errct,
            words,
            st.vaddr + first_err * 4,
            first_exp,
            first_got
        );
        return Err(PsramError::MemtestFailed);
    }
    psram_log!("I", TAG, "SPI SRAM memory test OK");
    Ok(())
}

/*---------------------------------------------------------------------------------
 * Whole init: Ok only if all stages (incl. the memtest) passed
 *-------------------------------------------------------------------------------*/
#[esp_hal::ram]
pub fn xesp_psram_init() -> Result<PsramState, PsramError> {
    const TAG: &str = "esp_psram";
    let mut st = PsramState::default();

    psram_stage1_pins(&mut st)?;
    // psram_stage2_low_speed(&mut st)?;
    psram_stage3_chip(&mut st)?;
    // return Err(PsramError::InvalidState);
    // stage 4 (tuning): skipped
    psram_stage5_high_speed(&mut st)?;
    psram_stage6_spi_phases(&mut st)?;

    psram_log!("I", TAG, "Found {}MB PSRAM device", st.size / (1024 * 1024));
    psram_log!("I", TAG, "Speed: {}MHz", PSRAM_FREQUENCY_MHZ);

    psram_stage7_mapping(&mut st)?;

    dump_all("port-after-map");

    psram_stage8_memtest(&mut st)?;

    Ok(st)
}

macro_rules! emit { ($($t:tt)*) => { esp_println::println!($($t)*) } }
fn dump_block(name: &str, base: usize, len: usize) {
    let mut off = 0usize;
    while off < len {
        let v = unsafe { core::ptr::read_volatile((base + off) as *const u32) };
        if v != 0 {
            emit!("{}+{:03x} = {:08x}", name, off, v);
        }
        off += 4;
    }
}
pub(crate) fn dump_all(tag: &str) {
    emit!("===== DUMP {} =====", tag);
    dump_block("SPI0", 0x6000_3000, 0x400);
    dump_block("SPI1", 0x6000_2000, 0x400);
    dump_block("EXTMEM", 0x600C_4000, 0x400);
    dump_block("SYSTEM", 0x600C_0000, 0x200);
    dump_block("SENSITIVE", 0x600C_1000, 0x400);
    dump_block("SYSCON", 0x6002_6000, 0x200);
    // IO_MUX пины 26..37 (адрес 0x60009004 + 4*n)
    let mut pin = 26usize;
    while pin <= 37 {
        let v = unsafe { core::ptr::read_volatile((0x6000_9004 + pin * 4) as *const u32) };
        emit!("IO_MUX+{:03x} = {:08x}", pin, v);
        pin += 1;
    }
    // только валидные записи MMU
    let mut i = 0usize;
    while i < 512 {
        let e = unsafe { core::ptr::read_volatile((0x600C_5000 + i * 4) as *const u32) };
        if e & 0x4000 == 0 {
            emit!("MMU+{:03} = {:04x}", i, e);
        }
        i += 1;
    }
    emit!("===== END {} =====", tag);
}

#[main]
fn main() -> ! {
    let _peripherals = esp_hal::init(esp_hal::Config::default());

    match xesp_psram_init() {
        Ok(psram) => {
            let ptr = psram.vaddr as *mut u32;
            unsafe {
                ptr.write_volatile(0x1234_5678);
                println!(
                    "main: PSRAM at {:p}, {} bytes, buf[0]=0x{:08x}",
                    ptr,
                    psram.mapped_size,
                    ptr.read_volatile()
                );
            }
        }
        Err(e) => println!("main: PSRAM init failed: {:?}", e),
    }

    loop {}
}
