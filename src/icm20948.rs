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

use crate::icm20948::RegisterBanks::*;

const READ_REG: bool = true;
const WRITE_REG: bool = false;

const ACCEL_SEN_0: u16 = 16_384;
const ACCEL_SEN_1: u16 = 8_192;
const ACCEL_SEN_2: u16 = 4_096;
const ACCEL_SEN_3: u16 = 2_048;

const GYRO_SEN_0: f32 = 131.0;
const GYRO_SEN_1: f32 = 65.5;
const GYRO_SEN_2: f32 = 32.8;
const GYRO_SEN_3: f32 = 16.4;

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

pub enum IcmError<E> {
    BusError(E),
}

pub enum RegisterBanks {
    Bank0 = 0,
    Bank1 = 1,
    Bank2 = 2,
    Bank3 = 3,
}

enum Registers {
    Wai = 0x00,
    //UserCtl = 0x03,
    //LpConfig = 0x05,
    PwrMgmt1 = 0x06,
    PwrMgmt2 = 0x7,
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

pub struct IcmImu<BUS, PIN> {
    bus: BUS,
    acc_en: AccStates,
    gyro_en: GyroStates,
    mag_en: MagStates,

    databuf: [u8; 10],

    accel_sen: u16,
    gyro_sen: f32,

    #[cfg(not(feature = "i2c"))]
    cs: PIN,
}

#[cfg(not(feature = "i2c"))]
impl<BUS, E, PIN> IcmImu<BUS, PIN>
    where BUS: spi::Transfer<u8, Error = E>,
    PIN: hal::digital::v2::OutputPin {

    pub fn new(mut bus: BUS, mut cs: PIN) -> Result<Self, IcmError<E>> {
        let mut buf = [get_addr(Registers::PwrMgmt1, WRITE_REG), 0x01];

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
            databuf: [0; 10],
        })
    }

    pub fn wai(&mut self) -> Result<u8, IcmError<E>> {
        self.cs.set_low().ok();

        self.databuf[0] = get_addr(Registers::Wai, READ_REG);
        self.databuf[1] = 0;
        self.bus.transfer(&mut self.databuf[0..2])?;

        self.cs.set_high().ok();
        defmt::trace!("data: {}, {}", self.databuf[0], self.databuf[1]);

        Ok(self.databuf[1])
    }

    fn reg_bank_sel(&mut self, bank: RegisterBanks) -> Result<(), IcmError<E>> {
        self.cs.set_low().ok();

        self.databuf[0] = get_addr(Registers::RegBankSel, WRITE_REG);
        match bank {
            Bank0 => self.databuf[1] = 0x00,
            Bank1 => self.databuf[1] = 0x10,
            Bank2 => self.databuf[1] = 0x20,
            Bank3 => self.databuf[1] = 0x30,
        }
        self.bus.transfer(&mut self.databuf[0..2])?;

        self.cs.set_high().ok();

        Ok(())
    }

    pub fn enable_acc(&mut self) -> Result<(), IcmError<E>> {
        self.cs.set_low().ok();

        self.databuf[0] = get_addr(Registers::PwrMgmt2, READ_REG);
        self.bus.transfer(&mut self.databuf[0..0])?;
        self.databuf[1] = self.databuf[0] & 0x07;
        self.databuf[0] = get_addr(Registers::PwrMgmt2, WRITE_REG);
        self.bus.transfer(&mut self.databuf[0..1])?;

        self.cs.set_high().ok();

        self.acc_en = AccOn;
        defmt::trace!("Accelerometer: On");

        Ok(())
    }

    pub fn disable_acc(&mut self) -> Result<(), IcmError<E>> {
        self.cs.set_low().ok();

        self.databuf[0] = get_addr(Registers::PwrMgmt2, READ_REG);
        self.bus.transfer(&mut self.databuf[0..0])?;
        self.databuf[1] = self.databuf[0] | 0x38;
        self.databuf[0] = get_addr(Registers::PwrMgmt2, WRITE_REG);
        self.bus.transfer(&mut self.databuf[0..1])?;

        self.cs.set_high().ok();

        self.acc_en = AccOff;
        defmt::trace!("Accelerometer: Off");

        Ok(())
    }

    pub fn enable_gyro(&mut self) -> Result<(), IcmError<E>> {
        self.cs.set_low().ok();

        self.databuf[0] = get_addr(Registers::PwrMgmt2, READ_REG);
        self.bus.transfer(&mut self.databuf[0..0])?;
        self.databuf[1] = self.databuf[0] & 0x38;
        self.databuf[0] = get_addr(Registers::PwrMgmt2, WRITE_REG);
        self.bus.transfer(&mut self.databuf[0..1])?;

        self.cs.set_high().ok();

        self.gyro_en = GyroOn;
        defmt::trace!("Gyro: On");

        Ok(())
    }

    pub fn disable_gyro(&mut self) -> Result<(), IcmError<E>> {
        self.cs.set_low().ok();

        self.databuf[0] = get_addr(Registers::PwrMgmt2, READ_REG);
        self.bus.transfer(&mut self.databuf[0..0])?;
        self.databuf[1] = self.databuf[0] | 0x07;
        self.databuf[0] = get_addr(Registers::PwrMgmt2, WRITE_REG);
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

        self.cs.set_low().ok();

        self.databuf[0] = get_addr(Registers::AccelXOutH, READ_REG);
        self.databuf[1] = get_addr(Registers::AccelXOutL, READ_REG);
        self.bus.transfer(&mut self.databuf[0..3])?;

        self.cs.set_high().ok();

        Ok((((self.databuf[1] as i16) << 8) | (self.databuf[2] as i16)) as f32 / self.accel_sen as f32)
    }

    pub fn read_acc_y(&mut self) -> Result<f32, IcmError<E>> {
        if self.acc_en != AccOn {
            self.enable_acc()?;
        }

        self.cs.set_low().ok();

        self.databuf[0] = get_addr(Registers::AccelYOutH, READ_REG);
        self.databuf[1] = get_addr(Registers::AccelYOutL, READ_REG);
        self.bus.transfer(&mut self.databuf[0..3])?;

        self.cs.set_high().ok();

        Ok((((self.databuf[1] as i16) << 8) | (self.databuf[2] as i16)) as f32 / self.accel_sen as f32)
    }

    pub fn read_acc_z(&mut self) -> Result<f32, IcmError<E>> {
        if self.acc_en != AccOn {
            self.enable_acc()?;
        }

        self.cs.set_low().ok();

        self.databuf[0] = get_addr(Registers::AccelZOutH, READ_REG);
        self.databuf[1] = get_addr(Registers::AccelZOutL, READ_REG);
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

        self.cs.set_low().ok();

        self.databuf[0] = get_addr(Registers::GyroXOutH, READ_REG);
        self.databuf[1] = get_addr(Registers::GyroXOutL, READ_REG);
        self.bus.transfer(&mut self.databuf[0..3])?;

        self.cs.set_high().ok();

        Ok((((self.databuf[1] as i16) << 8) | (self.databuf[2] as i16)) as f32 / self.gyro_sen)
    }

    pub fn read_gyro_y(&mut self) -> Result<f32, IcmError<E>> {
        if self.gyro_en != GyroOn {
            self.enable_acc()?;
        }

        self.cs.set_low().ok();

        self.databuf[0] = get_addr(Registers::GyroYOutH, READ_REG);
        self.databuf[1] = get_addr(Registers::GyroYOutL, READ_REG);
        self.bus.transfer(&mut self.databuf[0..3])?;

        self.cs.set_high().ok();

        Ok((((self.databuf[1] as i16) << 8) | (self.databuf[2] as i16)) as f32 / self.gyro_sen)
    }

    pub fn read_gyro_z(&mut self) -> Result<f32, IcmError<E>> {
        if self.gyro_en != GyroOn {
            self.enable_acc()?;
        }

        self.cs.set_low().ok();

        self.databuf[0] = get_addr(Registers::GyroZOutH, READ_REG);
        self.databuf[1] = get_addr(Registers::GyroZOutL, READ_REG);
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

fn get_addr(reg: Registers, is_read: bool) -> u8 {
    if is_read {
        1 << 7 | reg as u8
    } else {
        reg as u8
    }
}