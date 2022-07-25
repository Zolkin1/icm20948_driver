/// Device agnostic driver for the icm-20948
/// The driver assumes control of the bus and thus it is recommended to use shared-bus
/// Due to limitations in embedded-hal and thus how much shared-bus can do, SPI is not suppoerted in
/// shared-bus in a multi-threaded context.
/// Thus you cannot use this driver with SPI in a multithreaded context.

// TODO: Break the functions out into an IMU (or similar) trait
// TODO: Consider typestating again

use defmt::{Format, Formatter};
use stm32h7xx_hal::hal::{self, blocking::spi};

use crate::icm20948::AccStates::*;
use crate::icm20948::GyroStates::*;
use crate::icm20948::MagStates::*;

use crate::icm20948::AccSensitivity::*;

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
const REG_BANK_1: u8 = 0x10;
const REG_BANK_2: u8 = 0x20;
const REG_BANK_3: u8 = 0x30;

#[derive(PartialEq)]
#[derive(Format)]
pub enum AccStates {
    AccOn,
    AccOff,
}

#[derive(PartialEq)]
#[derive(Format)]
pub enum GyroStates {
    GyroOn,
    GyroOff,
}

#[derive(PartialEq)]
#[derive(Format)]
pub enum MagStates {
    MagOn,
    MagOff,
}

pub enum AccSensitivity {
    Sen2g = ACCEL_SEN_0 as isize,
    Sen4g = ACCEL_SEN_1 as isize,
    Sen8g = ACCEL_SEN_2 as isize,
    Sen16g = ACCEL_SEN_3 as isize,
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

enum RegistersBank2 {
    GyroConfig1,
    AccelConfig,
}

pub struct IcmImu<BUS, PIN> {
    bus: BUS,
    acc_en: AccStates,
    gyro_en: GyroStates,
    mag_en: MagStates,

    databuf: [u8; 10],

    accel_sen: u16,
    gyro_sen: f32,

    int_enabled: bool,

    #[cfg(not(feature = "i2c"))]
    cs: PIN,
}

#[cfg(not(feature = "i2c"))]
impl<BUS, E, PIN> IcmImu<BUS, PIN>
    where BUS: spi::Transfer<u8, Error = E>,
    PIN: hal::digital::v2::OutputPin {

    pub fn new(mut bus: BUS, mut cs: PIN) -> Result<Self, IcmError<E>> {
        let mut buf = [RegistersBank0::PwrMgmt1.get_addr(WRITE_REG), 0x01];

        cs.set_low().ok();
        bus.transfer(&mut buf)?;
        cs.set_high().ok();

        defmt::info!("Setting up IMU...");

        Ok(IcmImu {
            bus,
            acc_en: AccOn,
            gyro_en: GyroOn,
            mag_en: MagOff,
            cs,
            accel_sen: ACCEL_SEN_0,
            gyro_sen: GYRO_SEN_0,
            int_enabled: INT_NOT_ENABLED,
            databuf: [0; 10],
        })
    }

    pub fn wai(&mut self) -> Result<u8, IcmError<E>> {
        self.cs.set_low().ok();

        self.databuf[0] = RegistersBank0::Wai.get_addr(READ_REG);
        self.databuf[1] = 0;
        self.bus.transfer(&mut self.databuf[0..2])?;

        self.cs.set_high().ok();

        Ok(self.databuf[1])
    }

    pub fn enable_int(&mut self) -> Result<(), IcmError<E>> {
        self.cs.set_low().ok();

        self.databuf[0] = RegistersBank0::IntEnable1.get_addr(WRITE_REG);
        self.databuf[1] = 0x01;
        self.bus.transfer(&mut self.databuf[0..2])?;

        self.cs.set_high().ok();

        self.int_enabled = INT_ENABLED;
        Ok(())
    }

    pub fn disable_int(&mut self) -> Result<(), IcmError<E>> {
        self.cs.set_low().ok();

        self.databuf[0] = RegistersBank0::IntEnable1.get_addr(WRITE_REG);
        self.databuf[1] = 0x00;
        self.bus.transfer(&mut self.databuf[0..2])?;

        self.cs.set_high().ok();

        self.int_enabled = INT_NOT_ENABLED;
        Ok(())
    }

    pub fn enable_acc(&mut self) -> Result<(), IcmError<E>> {
        self.cs.set_low().ok();

        self.databuf[0] = RegistersBank0::PwrMgmt2.get_addr(READ_REG);
        self.bus.transfer(&mut self.databuf[0..0])?;
        self.databuf[1] = self.databuf[0] & 0x07;
        self.databuf[0] = RegistersBank0::PwrMgmt2.get_addr(WRITE_REG);
        self.bus.transfer(&mut self.databuf[0..1])?;

        self.cs.set_high().ok();

        self.acc_en = AccOn;
        defmt::trace!("Accelerometer: On");

        Ok(())
    }

    pub fn disable_acc(&mut self) -> Result<(), IcmError<E>> {
        self.cs.set_low().ok();

        self.databuf[0] = RegistersBank0::PwrMgmt2.get_addr(READ_REG);
        self.bus.transfer(&mut self.databuf[0..0])?;
        self.databuf[1] = self.databuf[0] | 0x38;
        self.databuf[0] = RegistersBank0::PwrMgmt2.get_addr(WRITE_REG);
        self.bus.transfer(&mut self.databuf[0..1])?;

        self.cs.set_high().ok();

        self.acc_en = AccOff;
        defmt::trace!("Accelerometer: Off");

        Ok(())
    }

    pub fn enable_gyro(&mut self) -> Result<(), IcmError<E>> {
        self.cs.set_low().ok();

        self.databuf[0] = RegistersBank0::PwrMgmt2.get_addr(READ_REG);
        self.bus.transfer(&mut self.databuf[0..0])?;
        self.databuf[1] = self.databuf[0] & 0x38;
        self.databuf[0] = RegistersBank0::PwrMgmt2.get_addr(WRITE_REG);
        self.bus.transfer(&mut self.databuf[0..1])?;

        self.cs.set_high().ok();

        self.gyro_en = GyroOn;
        defmt::trace!("Gyro: On");

        Ok(())
    }

    pub fn disable_gyro(&mut self) -> Result<(), IcmError<E>> {
        self.cs.set_low().ok();

        self.databuf[0] = RegistersBank0::PwrMgmt2.get_addr(READ_REG);
        self.bus.transfer(&mut self.databuf[0..0])?;
        self.databuf[1] = self.databuf[0] | 0x07;
        self.databuf[0] = RegistersBank0::PwrMgmt2.get_addr(WRITE_REG);
        self.bus.transfer(&mut self.databuf[0..1])?;

        self.cs.set_high().ok();

        self.gyro_en = GyroOff;
        defmt::trace!("Gyro: Off");

        Ok(())
    }

    pub fn gyro_on(&self) -> bool {
        self.gyro_en == GyroOn
    }

    pub fn acc_on(&self) -> bool {
        self.acc_en == AccOn
    }

    pub fn mag_on(&self) -> bool {
        self.mag_en == MagOn
    }

    pub fn read_acc_x(&mut self) -> Result<f32, IcmError<E>> {
        if self.acc_en != AccOn {
            self.enable_acc()?;
        }

        self.databuf[0] = RegistersBank0::AccelXOutH.get_addr(READ_REG);
        self.databuf[1] = RegistersBank0::AccelXOutL.get_addr(READ_REG);

        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[0..3])?;
        self.cs.set_high().ok();

        Ok((((self.databuf[1] as i16) << 8) | (self.databuf[2] as i16)) as f32 / self.accel_sen as f32)
    }

    pub fn read_acc_y(&mut self) -> Result<f32, IcmError<E>> {
        if self.acc_en != AccOn {
            self.enable_acc()?;
        }


        self.databuf[0] = RegistersBank0::AccelYOutH.get_addr(READ_REG);
        self.databuf[1] = RegistersBank0::AccelYOutL.get_addr(READ_REG);

        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[0..3])?;
        self.cs.set_high().ok();

        Ok((((self.databuf[1] as i16) << 8) | (self.databuf[2] as i16)) as f32 / self.accel_sen as f32)
    }

    pub fn read_acc_z(&mut self) -> Result<f32, IcmError<E>> {
        if self.acc_en != AccOn {
            self.enable_acc()?;
        }


        self.databuf[0] = RegistersBank0::AccelZOutH.get_addr(READ_REG);
        self.databuf[1] = RegistersBank0::AccelZOutL.get_addr(READ_REG);

        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[0..3])?;
        self.cs.set_high().ok();

        Ok((((self.databuf[1] as i16) << 8) | (self.databuf[2] as i16)) as f32 / self.accel_sen as f32)
    }

    pub fn read_acc(&mut self) -> Result<[f32; 3], IcmError<E>> {
        let mut res = [0.0; 3];
        res[0] = self.read_acc_x()?;
        res[1] = self.read_acc_y()?;
        res[2] = self.read_acc_z()?;

        Ok(res)
    }

    pub fn read_gyro_x(&mut self) -> Result<f32, IcmError<E>> {
        if self.gyro_en != GyroOn {
            self.enable_acc()?;
        }


        self.databuf[0] = RegistersBank0::GyroXOutH.get_addr(READ_REG);
        self.databuf[1] = RegistersBank0::GyroXOutL.get_addr(READ_REG);

        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[0..3])?;
        self.cs.set_high().ok();

        Ok((((self.databuf[1] as i16) << 8) | (self.databuf[2] as i16)) as f32 / self.gyro_sen)
    }

    pub fn read_gyro_y(&mut self) -> Result<f32, IcmError<E>> {
        if self.gyro_en != GyroOn {
            self.enable_acc()?;
        }

        self.databuf[0] = RegistersBank0::GyroYOutH.get_addr(READ_REG);
        self.databuf[1] = RegistersBank0::GyroYOutL.get_addr(READ_REG);

        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[0..3])?;
        self.cs.set_high().ok();

        Ok((((self.databuf[1] as i16) << 8) | (self.databuf[2] as i16)) as f32 / self.gyro_sen)
    }

    pub fn read_gyro_z(&mut self) -> Result<f32, IcmError<E>> {
        if self.gyro_en != GyroOn {
            self.enable_acc()?;
        }

        self.databuf[0] = RegistersBank0::GyroZOutH.get_addr(READ_REG);
        self.databuf[1] = RegistersBank0::GyroZOutL.get_addr(READ_REG);

        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[0..3])?;
        self.cs.set_high().ok();

        Ok((((self.databuf[1] as i16) << 8) | (self.databuf[2] as i16)) as f32 / self.gyro_sen)
    }

    pub fn read_gyro(&mut self) -> Result<[f32; 3], IcmError<E>> {
        let mut res = [0.0; 3];
        res[0] = self.read_gyro_x()?;
        res[1] = self.read_gyro_y()?;
        res[2] = self.read_gyro_z()?;

        Ok(res)
    }

    pub fn set_acc_sen(&mut self, acc_sen: AccSensitivity) -> Result<(), IcmError<E>> {
        self.change_bank(REG_BANK_2)?;

        self.databuf[2] = RegistersBank2::AccelConfig.get_addr(READ_REG);

        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[2..4])?;
        self.cs.set_high().ok();

        match acc_sen {
            Sen2g => self.databuf[1] = self.databuf[3] & 0xF9,
            Sen4g => self.databuf[1] = (self.databuf[3] & 0xFB) | 0x02,
            Sen8g => self.databuf[1] = (self.databuf[3] & 0xFD) | 0x04,
            Sen16g => self.databuf[1] = self.databuf[3] | 0x06,
        };

        self.databuf[0] = RegistersBank2::AccelConfig.get_addr(WRITE_REG);

        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[0..2])?;
        self.cs.set_high().ok();

        self.change_bank(REG_BANK_0)?;

        self.accel_sen = acc_sen as u16;

        Ok(())
    }

    pub fn set_gyro_sen(&mut self, gyro_sen: GyroSensitivity) -> Result<(), IcmError<E>> {
        self.change_bank(REG_BANK_2)?;

        self.databuf[2] = RegistersBank2::GyroConfig1.get_addr(READ_REG);

        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[2..4])?;
        self.cs.set_high().ok();

        match gyro_sen {
            GyroSensitivity::Sen250dps => self.databuf[1] = self.databuf[3] & 0xF9,
            GyroSensitivity::Sen500dps => self.databuf[1] = (self.databuf[3] & 0xFB) | 0x02,
            GyroSensitivity::Sen1000dps => self.databuf[1] = (self.databuf[3] & 0xFD) | 0x04,
            GyroSensitivity::Sen2000dps => self.databuf[1] = self.databuf[3] | 0x06,
        };

        self.databuf[0] = RegistersBank2::GyroConfig1.get_addr(WRITE_REG);

        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[0..2])?;
        self.cs.set_high().ok();

        self.change_bank(REG_BANK_0)?;

        match gyro_sen {
            GyroSensitivity::Sen250dps => self.gyro_sen = GYRO_SEN_0,
            GyroSensitivity::Sen500dps => self.gyro_sen = GYRO_SEN_1,
            GyroSensitivity::Sen1000dps => self.gyro_sen = GYRO_SEN_2,
            GyroSensitivity::Sen2000dps => self.gyro_sen = GYRO_SEN_3,
        }

        Ok(())
    }

    pub fn config_acc_lpf(&mut self, bw: AccLPF) -> Result<(), IcmError<E>> {
        self.change_bank(REG_BANK_2)?;

        self.databuf[0] = RegistersBank2::AccelConfig.get_addr(READ_REG);

        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[0..2])?;
        self.cs.set_high().ok();

        let mask: u8 = 0x38;

        match bw {
            AccLPF::BW6 => self.databuf[1] = (self.databuf[1] & !mask) | ((6 << 3) &  mask),
            AccLPF::BW11 => self.databuf[1] = (self.databuf[1] & !mask) | ((5 << 3) &  mask),
            AccLPF::BW24 => self.databuf[1] = (self.databuf[1] & !mask) | ((4 << 3) &  mask),
            AccLPF::BW50 => self.databuf[1] = (self.databuf[1] & !mask) | ((3 << 3) &  mask),
            AccLPF::BW111 => self.databuf[1] = (self.databuf[1] & !mask) | ((2 << 3) &  mask),
            AccLPF::BW246 => self.databuf[1] = (self.databuf[1] & !mask) | ((0 << 3) &  mask),
            AccLPF::Bw473 => self.databuf[1] = (self.databuf[1] & !mask) | ((7 << 3) &  mask),
        }

        self.databuf[1] = self.databuf[1] | 0x01;

        self.databuf[0] = RegistersBank2::AccelConfig.get_addr(WRITE_REG);

        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[0..2])?;
        self.cs.set_high().ok();

        self.change_bank(REG_BANK_0)?;
        Ok(())
    }

    pub fn config_gyro_lpf(&mut self, bw: GyroLPF) -> Result<(), IcmError<E>> {
        self.change_bank(REG_BANK_2)?;

        self.databuf[0] = RegistersBank2::AccelConfig.get_addr(READ_REG);

        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[0..2])?;
        self.cs.set_high().ok();

        let mask: u8 = 0x38;

        match bw {
            GyroLPF::BW6 => self.databuf[1] = (self.databuf[1] & !mask) | ((6 << 3) &  mask),
            GyroLPF::BW12 => self.databuf[1] = (self.databuf[1] & !mask) | ((5 << 3) &  mask),
            GyroLPF::BW24 => self.databuf[1] = (self.databuf[1] & !mask) | ((4 << 3) &  mask),
            GyroLPF::BW51 => self.databuf[1] = (self.databuf[1] & !mask) | ((3 << 3) &  mask),
            GyroLPF::BW119 => self.databuf[1] = (self.databuf[1] & !mask) | ((2 << 3) &  mask),
            GyroLPF::BW152 => self.databuf[1] = (self.databuf[1] & !mask) | ((1 << 3) &  mask),
            GyroLPF::BW197 => self.databuf[1] = (self.databuf[1] & !mask) | ((0 << 3) &  mask),
            GyroLPF::BW361 => self.databuf[1] = (self.databuf[1] & !mask) | ((7 << 3) &  mask),
        }

        self.databuf[1] = self.databuf[1] | 0x01;

        self.databuf[0] = RegistersBank2::AccelConfig.get_addr(WRITE_REG);

        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[0..2])?;
        self.cs.set_high().ok();

        self.change_bank(REG_BANK_0)?;
        Ok(())
    }

    // TODO: Test
    pub fn disable_acc_lpf(&mut self) -> Result<(), IcmError<E>> {
        self.change_bank(REG_BANK_2)?;

        self.databuf[0] = RegistersBank2::AccelConfig.get_addr(READ_REG);
        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[0..2])?;
        self.cs.set_high().ok();

        self.databuf[0] = RegistersBank2::AccelConfig.get_addr(WRITE_REG);
        self.databuf[1] = self.databuf[1] & 0xFE;

        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[0..2])?;
        self.cs.set_high().ok();

        self.change_bank(REG_BANK_0)?;

        Ok(())

    }

    // TODO: Test
    pub fn disable_gyro_lpf(&mut self) -> Result<(), IcmError<E>> {
        self.change_bank(REG_BANK_2)?;

        self.databuf[0] = RegistersBank2::GyroConfig1.get_addr(READ_REG);
        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[0..2])?;
        self.cs.set_high().ok();

        self.databuf[0] = RegistersBank2::GyroConfig1.get_addr(WRITE_REG);
        self.databuf[1] = self.databuf[1] & 0xFE;

        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[0..2])?;
        self.cs.set_high().ok();

        self.change_bank(REG_BANK_0)?;

        Ok(())

    }

    fn change_bank(&mut self, bank: u8) -> Result<(), IcmError<E>> {
        self.databuf[0] = RegistersBank0::RegBankSel.get_addr(WRITE_REG);
        self.databuf[1] = bank;

        self.cs.set_low().ok();
        self.bus.transfer(&mut self.databuf[0..2])?;
        self.cs.set_high().ok();

        Ok(())
    }
}

impl<E> From<E> for IcmError<E> {
    fn from(error: E) -> Self {
        IcmError::BusError(error)
    }
}

// TODO: Update this
impl<BUS, PIN> Format for IcmImu<BUS, PIN> {
    fn format(&self, fmt: Formatter) {
        defmt::write!(fmt, "IMU{}", self.databuf[0])
    }
}

// TODO: Update
impl<E> Format for IcmError<E> {
    fn format(&self, fmt: Formatter) {
        defmt::write!(fmt, "IMU Error!")
    }
}

#[cfg(feature = "i2c")]
impl<'a, BUS, const P: char, const N: u8> IcmImu<'a, BUS, P, N>
    where BUS: WriteRead<u8> {
    pub fn wai() -> u8 {
        4
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
                RegistersBank2::GyroConfig1 => 1 << 7 | 0x01,
                RegistersBank2::AccelConfig => 1 << 7 | 0x14,
            }
        } else {
            match *self {
                RegistersBank2::GyroConfig1 => 0x01,
                RegistersBank2::AccelConfig => 0x14
            }
        }
    }
}
