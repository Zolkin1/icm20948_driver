// Copyright (c) 2022, Zachary D. Olkin.
// This code is provided under the MIT license.

use defmt::{Format, Formatter};

/// The i2c module holds all of the driver implementations when using an I2C bus to communicate with the device
pub mod i2c;
/// The SPI module holds all of the driver implementations when using an SPI bus to communicate with the device
pub mod spi;

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
const REG_BANK_3: u8 = 0x30;

const TEMP_SEN: f32 = 333.87;

const MAG_ADDR: u8 = 0x0C;
const MAG_SEN: f32 = 0.15;

/// States of the accelerometer: On or Off
#[derive(PartialEq, Format)]
pub enum AccStates {
    /// On
    AccOn,
    /// Off
    AccOff,
}

/// States of the gyro: On or Off
#[derive(PartialEq, Format)]
pub enum GyroStates {
    /// On
    GyroOn,
    /// Off
    GyroOff,
}

/// States of the magnetometer: On or Off
#[derive(PartialEq, Format)]
pub enum MagStates {
    /// On
    MagOn,
    /// Off
    MagOff,
}

/// States of the temperature sensor: On or Off
#[derive(PartialEq, Format)]
pub enum TempStates {
    /// On
    TempOn,
    /// Off
    TempOff,
}

/// Magnetometer data rate options
pub enum MagRates {
    /// 10Hz measurement
    Mag10Hz,
    /// 20Hz measurement
    Mag20Hz,
    /// 50Hz measurement
    Mag50Hz,
    /// 100Hz measurement
    Mag100Hz,
    /// The magnetometer makes one measurement then goes to power down mode.
    MagSingle,
}

/// Accelerometer sensitivity options as specified in the data sheet in g's.
pub enum AccSensitivity {
    /// 2g of sensitivity
    Sen2g,
    /// 4g of sensitivity
    Sen4g,
    /// 8g of sensitivity
    Sen8g,
    /// 16g of sensitivity
    Sen16g,
}

/// Gyro sensitivity options as specified in the data sheet in degrees per second (dps).
pub enum GyroSensitivity {
    /// 250 dps of sensitivity
    Sen250dps,
    /// 500 dps of sensitivity
    Sen500dps,
    /// 1000 dps of sensitivity
    Sen1000dps,
    /// 2000 dps of sensitivity
    Sen2000dps,
}

/// Accelerometer Low Pass Filter (LPF) options as specified in the data sheet.
/// All values are in the 3DB BW of the LPF.
pub enum AccLPF {
    /// 246.0Hz 3DB BW
    BW246,
    /// 111.4Hz 3DB BW
    BW111,
    /// 50.4Hz 3DB BW
    BW50,
    /// 23.9Hz 3DB BW
    BW24,
    /// 11.5Hz 3DB BW
    BW11,
    /// 5.7Hz 3DB BW
    BW6,
    /// 473.0Hz 3DB BW
    Bw473,
}

/// Gyro Low Pass Filter (LPF) options as specified in the data sheet.
/// All values are in the 3DB BW of the LPF.
pub enum GyroLPF {
    /// 196.6Hz 3DB BW
    BW197,
    /// 151.8Hz 3DB BW
    BW152,
    /// 119.5Hz 3DB BW
    BW119,
    /// 51.2Hz 3DB BW
    BW51,
    /// 23.9Hz 3DB BW
    BW24,
    /// 11.6Hz 3DB BW
    BW12,
    /// 5.7Hz 3DB BW
    BW6,
    /// 361.4Hz 3DB BW
    BW361,
}

/// The possible errors that the driver can return.
///
/// The `BusError` option is for when a HAL function using either the SPI or I2C bus fails.
/// This may be caused by a number of reasons. For example, using the wrong 7-bit address with the I2C bus will cause a bus error.
///
/// `InvalidInput` is for when an input to a driver function is unacceptable.
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum IcmError<E> {
    /// An error occurred when using the bus
    BusError(E),
    /// An invalid input was passed to the function
    InvalidInput,
}

enum RegistersBank0 {
    Wai,
    UserCtrl,
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
    TempOutL,
    TempOutH,
    ExtSlvSensData00,
    ExtSlvSensData01,
    ExtSlvSensData02,
    ExtSlvSensData03,
    ExtSlvSensData04,
    ExtSlvSensData05,
    ExtSlvSensData06,
    ExtSlvSensData07,
    ExtSlvSensData08,
    ExtSlvSensData09,
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
    OdrAlignEn,
}

enum RegistersBank3 {
    I2cMstCtrl,
    I2cMstDelayCtrl,
    I2cSlv0Addr,
    I2cSlv0Reg,
    I2cSlv0Ctrl,
    I2cSlvDo,
}

impl<E> From<E> for IcmError<E> {
    fn from(error: E) -> Self {
        IcmError::BusError(error)
    }
}

#[cfg(feature = "defmt")]
impl<E: Format> Format for IcmError<E> {
    fn format(&self, fmt: Formatter) {
        match &*self {
            IcmError::BusError(e) => defmt::write!(fmt, "Bus Error: {}", e),
            IcmError::InvalidInput => defmt::write!(fmt, "Invalid input in the function"),
        }
    }
}

impl RegistersBank0 {
    fn get_addr(&self, is_read: bool) -> u8 {
        if is_read {
            match *self {
                RegistersBank0::Wai => 1 << 7 | 0x0,
                RegistersBank0::UserCtrl => 1 << 7 | 0x03,
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
                RegistersBank0::TempOutH => 1 << 7 | 0x39,
                RegistersBank0::TempOutL => 1 << 7 | 0x3A,
                RegistersBank0::ExtSlvSensData00 => 1 << 7 | 0x3B,
                RegistersBank0::ExtSlvSensData01 => 1 << 7 | 0x3C,
                RegistersBank0::ExtSlvSensData02 => 1 << 7 | 0x3D,
                RegistersBank0::ExtSlvSensData03 => 1 << 7 | 0x3E,
                RegistersBank0::ExtSlvSensData04 => 1 << 7 | 0x3F,
                RegistersBank0::ExtSlvSensData05 => 1 << 7 | 0x40,
                RegistersBank0::ExtSlvSensData06 => 1 << 7 | 0x41,
                RegistersBank0::ExtSlvSensData07 => 1 << 7 | 0x42,
                RegistersBank0::ExtSlvSensData08 => 1 << 7 | 0x43,
                RegistersBank0::ExtSlvSensData09 => 1 << 7 | 0x44,
                RegistersBank0::RegBankSel => 1 << 7 | 0x7F,
            }
        } else {
            match *self {
                RegistersBank0::Wai => 0x0,
                RegistersBank0::UserCtrl => 0x03,
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
                RegistersBank0::TempOutH => 0x39,
                RegistersBank0::TempOutL => 0x3A,
                RegistersBank0::ExtSlvSensData00 => 0x3B,
                RegistersBank0::ExtSlvSensData01 => 0x3C,
                RegistersBank0::ExtSlvSensData02 => 0x3D,
                RegistersBank0::ExtSlvSensData03 => 0x3E,
                RegistersBank0::ExtSlvSensData04 => 0x3F,
                RegistersBank0::ExtSlvSensData05 => 0x40,
                RegistersBank0::ExtSlvSensData06 => 0x41,
                RegistersBank0::ExtSlvSensData07 => 0x42,
                RegistersBank0::ExtSlvSensData08 => 0x43,
                RegistersBank0::ExtSlvSensData09 => 0x44,
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
                RegistersBank2::OdrAlignEn => 1 << 7 | 0x09,
            }
        } else {
            match *self {
                RegistersBank2::GyroSmplrtDiv => 0x00,
                RegistersBank2::GyroConfig1 => 0x01,
                RegistersBank2::AccelSmplrtDiv1 => 0x10,
                RegistersBank2::AccelSmplrtDiv2 => 0x11,
                RegistersBank2::AccelConfig => 0x14,
                RegistersBank2::OdrAlignEn => 0x09,
            }
        }
    }
}

impl RegistersBank3 {
    fn get_addr(&self, is_read: bool) -> u8 {
        if is_read {
            match *self {
                RegistersBank3::I2cMstCtrl => 1 << 7 | 0x01,
                RegistersBank3::I2cMstDelayCtrl => 1 << 7 | 0x02,
                RegistersBank3::I2cSlv0Addr => 1 << 7 | 0x03,
                RegistersBank3::I2cSlv0Reg => 1 << 7 | 0x04,
                RegistersBank3::I2cSlv0Ctrl => 1 << 7 | 0x05,
                RegistersBank3::I2cSlvDo => 1 << 7 | 0x06,
            }
        } else {
            match *self {
                RegistersBank3::I2cMstCtrl => 0x01,
                RegistersBank3::I2cMstDelayCtrl => 0x02,
                RegistersBank3::I2cSlv0Addr => 0x03,
                RegistersBank3::I2cSlv0Reg => 0x04,
                RegistersBank3::I2cSlv0Ctrl => 0x05,
                RegistersBank3::I2cSlvDo => 0x06,
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
