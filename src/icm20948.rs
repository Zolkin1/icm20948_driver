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

/// register bank #0
#[allow(missing_docs)]
#[derive(Clone, Copy, strum::EnumIter)]
#[repr(u8)]
pub enum RegistersBank0 {
    Wai = 0x00,
    PwrMgmt1 = 0x06,
    PwrMgmt2 = 0x07,
    IntPinCfg = 0x0F,
    IntEnable = 0x10,
    IntEnable1 = 0x11,
    IntStatus = 0x19,
    AccelXOutH = 0x2D,
    AccelXOutL = 0x2E,
    AccelYOutH = 0x2F,
    AccelYOutL = 0x30,
    AccelZOutH = 0x31,
    AccelZOutL = 0x32,
    GyroXOutH = 0x33,
    GyroXOutL = 0x34,
    GyroYOutH = 0x35,
    GyroYOutL = 0x36,
    GyroZOutH = 0x37,
    GyroZOutL = 0x38,
    RegBankSel = 0x7F,
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

/// register bank #2
#[allow(missing_docs)]
#[derive(Clone, Copy, strum::EnumIter)]
#[repr(u8)]
pub enum RegistersBank2 {
    GyroSmplrtDiv = 0x00,
    GyroConfig1 = 0x01,
    OdrAlignEn = 0x09,
    AccelSmplrtDiv1 = 0x10,
    AccelSmplrtDiv2 = 0x11,
    AccelIntelCtrl = 0x12,
    AccelWomThr = 0x13,
    AccelConfig = 0x14,
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

/// Bank trait
pub trait Bank: Into<u8> + Copy + strum::IntoEnumIterator {
    /// get bank id
    fn id() -> u8;

    /// generate address suitable for read/write
    fn get_addr(&self, is_read: bool) -> u8 {
        if is_read {
            self.read()
        } else {
            self.write()
        }
    }

    /// generate address suitable for write
    fn write(self) -> u8 {
        self.into()
    }

    /// generate address suitable for read
    fn read(self) -> u8 {
        self.into() | 1 << 7
    }
}

impl From<RegistersBank0> for u8 {
    fn from(value: RegistersBank0) -> Self {
        value as u8
    }
}

impl Bank for RegistersBank0 {
    fn id() -> u8 {
        0
    }
}

impl From<RegistersBank2> for u8 {
    fn from(value: RegistersBank2) -> Self {
        value as u8
    }
}

impl Bank for RegistersBank2 {
    fn id() -> u8 {
        2
    }
}
/*
impl RegistersBank0 {
    fn get_addr(&self, is_read: bool) -> u8 {
        if is_read {
            *self as u8 | 1 << 7
        } else {
            *self as u8
        }
    }

    fn write(self) -> u8 {
        self as u8
    }

    fn read(self) -> u8 {
        self as u8 | 1 << 7
    }
}

impl RegistersBank2 {
    fn get_addr(&self, is_read: bool) -> u8 {
        if is_read {
            *self as u8 | 1 << 7
        } else {
            *self as u8
        }
    }

    fn write(self) -> u8 {
        self as u8
    }

    fn read(self) -> u8 {
        self as u8 | 1 << 7
    }
}
*/

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
