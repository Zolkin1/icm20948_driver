/// Device agnostic driver for the icm-20948
/// The driver assumes control of the bus and thus it is recommended to use shared-bus
/// Due to limitations in embedded-hal and thus how much shared-bus can do, SPI is not suppoerted in
/// shared-bus in a multi-threaded context.
/// Thus you cannot use this driver with SPI in a multithreaded context.


use defmt::{Format, Formatter, Str};
use stm32h7xx_hal::hal::{self, blocking::i2c, blocking::spi};
use stm32h7xx_hal::{gpio};
use stm32h7xx_hal::spi::Error;

use crate::icm20948::AccStates::*;
use crate::icm20948::GyroStates::*;
use crate::icm20948::MagStates::*;

const READ_REG: bool = true;
const WRITE_REG: bool = false;

#[derive(Format)]
pub enum AccStates {
    AccOn,
    AccOff,
}

#[derive(Format)]
pub enum GyroStates {
    GyroOn,
    GyroOff,
}

#[derive(Format)]
pub enum MagStates {
    MagOn,
    MagOff,
}

pub enum IcmError<E> {
    BusError(E),
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
}

pub struct IcmImu<BUS, PIN> {
    bus: BUS,
    acc_en: AccStates,
    gyro_en: GyroStates,
    mag_en: MagStates,

    databuf: [u8; 10],

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
            acc_en: AccOff,
            gyro_en: GyroOff,
            mag_en: MagOff,
            cs,
            databuf: [0; 10],
        })
    }

    pub fn wai(&mut self) -> Result<u8, IcmError<E>> {
        self.cs.set_low().ok();

        self.databuf[0] = get_addr(Registers::Wai, READ_REG);
        self.databuf[1] = 0;
        self.bus.transfer(&mut self.databuf[0..2])?;

        self.cs.set_high().ok();
        defmt::debug!("data: {}, {}", self.databuf[0], self.databuf[1]);

        Ok(self.databuf[1])
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