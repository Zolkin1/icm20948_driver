/// The i2c module holds all of the driver implementations when using an I2C bus to communicate with the device
pub mod i2c;

/// The SPI module holds all of the driver implementations when using an SPI bus to communicate with the device
pub mod spi;

use defmt::{Format, Formatter};

const READ_REG: bool = true;
const WRITE_REG: bool = false;

const INT_ENABLED: bool = true;
const INT_NOT_ENABLED: bool = false;

const ACCEL_SEN_0: u16 = 16_384;
const ACCEL_SEN_1: u16 = 8_192;
const ACCEL_SEN_2: u16 = 4_096;
const ACCEL_SEN_3: u16 = 2_048;

const GYRO_SEN_0: f32 = 131.0;
const GYRO_SEN_1: f32 = 65.5;
const GYRO_SEN_2: f32 = 32.8;
const GYRO_SEN_3: f32 = 16.4;

const REG_BANK_0: u8 = 0x00;
//const REG_BANK_1: u8 = 0x10;
const REG_BANK_2: u8 = 0x20;
//const REG_BANK_3: u8 = 0x30;

#[derive(PartialEq, Format)]
pub enum AccStates {
    AccOn,
    AccOff,
}

#[derive(PartialEq, Format)]
pub enum GyroStates {
    GyroOn,
    GyroOff,
}

#[derive(PartialEq, Format)]
pub enum MagStates {
    MagOn,
    MagOff,
}

pub enum AccSensitivity {
    Sen2g,
    Sen4g,
    Sen8g,
    Sen16g,
}

pub enum GyroSensitivity {
    Sen250dps,
    Sen500dps,
    Sen1000dps,
    Sen2000dps,
}

pub enum AccLPF {
    BW246,
    BW111,
    BW50,
    BW24,
    BW11,
    BW6,
    Bw473,
}

pub enum GyroLPF {
    BW197,
    BW152,
    BW119,
    BW51,
    BW24,
    BW12,
    BW6,
    BW361,
}

pub enum IcmError<E> {
    BusError(E),
    InvalidInput,
}

enum RegistersBank0 {
    Wai,
    PwrMgmt1,
    PwrMgmt2,
    IntEnable1,
    AccelXOutH,
    AccelXOutL,
    AccelYOutH,
    AccelYOutL,
    AccelZOutH,
    AccelZOutL,
    GyroXOutH,
    GyroXOutL,
    GyroYOutH,
    GyroYOutL,
    GyroZOutH,
    GyroZOutL,
    RegBankSel,
}
/*
enum RegistersBank1 {
    XAOffsH,
    XAOffsL,
    YAOffsH,
    YAOffsL,
    ZAOffsH,
    ZAOffsL,
}*/

enum RegistersBank2 {
    GyroConfig1,
    AccelConfig,
    AccelSmplrtDiv1,
    AccelSmplrtDiv2,
    GyroSmplrtDiv,
}

impl<E> From<E> for IcmError<E> {
    fn from(error: E) -> Self {
        IcmError::BusError(error)
    }
}

// TODO: Update with specific error
impl<E> Format for IcmError<E> {
    fn format(&self, fmt: Formatter) {
        match *self {
            IcmError::BusError(_) => defmt::write!(fmt, "Bus Error!"),
            IcmError::InvalidInput => defmt::write!(fmt, "Invalid input in the function!"),
        }
    }
}

impl RegistersBank0 {
    fn get_addr(&self, is_read: bool) -> u8 {
        if is_read {
            match *self {
                RegistersBank0::Wai => 1 << 7 | 0x0,
                RegistersBank0::PwrMgmt1 => 1 << 7 | 0x06,
                RegistersBank0::PwrMgmt2 => 1 << 7 | 0x07,
                RegistersBank0::IntEnable1 => 1 << 7 | 0x11,
                RegistersBank0::AccelXOutH => 1 << 7 | 0x2D,
                RegistersBank0::AccelXOutL => 1 << 7 | 0x2E,
                RegistersBank0::AccelYOutH => 1 << 7 | 0x2F,
                RegistersBank0::AccelYOutL => 1 << 7 | 0x30,
                RegistersBank0::AccelZOutH => 1 << 7 | 0x31,
                RegistersBank0::AccelZOutL => 1 << 7 | 0x32,
                RegistersBank0::GyroXOutH => 1 << 7 | 0x33,
                RegistersBank0::GyroXOutL => 1 << 7 | 0x34,
                RegistersBank0::GyroYOutH => 1 << 7 | 0x35,
                RegistersBank0::GyroYOutL => 1 << 7 | 0x36,
                RegistersBank0::GyroZOutH => 1 << 7 | 0x37,
                RegistersBank0::GyroZOutL => 1 << 7 | 0x38,
                RegistersBank0::RegBankSel => 1 << 7 | 0x7F,
            }
        } else {
            match *self {
                RegistersBank0::Wai => 0x0,
                RegistersBank0::PwrMgmt1 => 0x06,
                RegistersBank0::PwrMgmt2 => 0x07,
                RegistersBank0::IntEnable1 => 0x11,
                RegistersBank0::AccelXOutH => 0x2D,
                RegistersBank0::AccelXOutL => 0x2E,
                RegistersBank0::AccelYOutH => 0x2F,
                RegistersBank0::AccelYOutL => 0x30,
                RegistersBank0::AccelZOutH => 0x31,
                RegistersBank0::AccelZOutL => 0x32,
                RegistersBank0::GyroXOutH => 0x33,
                RegistersBank0::GyroXOutL => 0x34,
                RegistersBank0::GyroYOutH => 0x35,
                RegistersBank0::GyroYOutL => 0x36,
                RegistersBank0::GyroZOutH => 0x37,
                RegistersBank0::GyroZOutL => 0x38,
                RegistersBank0::RegBankSel => 0x7F,
            }
        }
    }
}

impl RegistersBank2 {
    fn get_addr(&self, is_read: bool) -> u8 {
        if is_read {
            match *self {
                RegistersBank2::GyroSmplrtDiv => 1 << 7 | 0x00,
                RegistersBank2::GyroConfig1 => 1 << 7 | 0x01,
                RegistersBank2::AccelSmplrtDiv1 => 1 << 7 | 0x10,
                RegistersBank2::AccelSmplrtDiv2 => 1 << 7 | 0x11,
                RegistersBank2::AccelConfig => 1 << 7 | 0x14,
            }
        } else {
            match *self {
                RegistersBank2::GyroSmplrtDiv => 0x00,
                RegistersBank2::GyroConfig1 => 0x01,
                RegistersBank2::AccelSmplrtDiv1 => 0x10,
                RegistersBank2::AccelSmplrtDiv2 => 0x11,
                RegistersBank2::AccelConfig => 0x14,
            }
        }
    }
}

/*
impl RegistersBank1 {
    fn get_addr(&self, is_read: bool) -> u8 {
        if is_read {
            match *self {
                RegistersBank1::XAOffsH => 1 << 7 | 0x14,
                RegistersBank1::XAOffsL => 1 << 7 | 0x15,
                RegistersBank1::YAOffsH => 1 << 7 | 0x17,
                RegistersBank1::YAOffsL => 1 << 7 | 0x18,
                RegistersBank1::ZAOffsH => 1 << 7 | 0x1A,
                RegistersBank1::ZAOffsL => 1 << 7 | 0x1B,
            }
        } else {
            match *self {
                RegistersBank1::XAOffsH => 0x14,
                RegistersBank1::XAOffsL => 0x15,
                RegistersBank1::YAOffsH => 0x17,
                RegistersBank1::YAOffsL => 0x18,
                RegistersBank1::ZAOffsH => 0x1A,
                RegistersBank1::ZAOffsL => 0x1B,
            }
        }
    }
}*/
