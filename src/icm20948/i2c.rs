// Copyright (c) 2022, Zachary D. Olkin.
// This code is provided under the MIT license.

use crate::icm20948::{AccLPF, TEMP_SEN};
use crate::icm20948::AccSensitivity;
use crate::icm20948::AccStates::{self, *};
use crate::icm20948::TempStates::{self, *};
use crate::icm20948::MagRates;
use crate::icm20948::GyroLPF;
use crate::icm20948::GyroSensitivity;
use crate::icm20948::GyroStates::{self, *};
use crate::icm20948::IcmError;
use crate::icm20948::MagStates::{self, *};
use crate::icm20948::{RegistersBank0, RegistersBank2, RegistersBank3};
use crate::icm20948::{
    ACCEL_SEN_0, ACCEL_SEN_1, ACCEL_SEN_2, ACCEL_SEN_3, GYRO_SEN_0, GYRO_SEN_1, GYRO_SEN_2,
    GYRO_SEN_3, INT_ENABLED, INT_NOT_ENABLED, WRITE_REG, MAG_ADDR, MAG_SEN,
};
use crate::icm20948::{REG_BANK_0, REG_BANK_2, REG_BANK_3};
use defmt::{Format, Formatter};

use embedded_hal::blocking::i2c;

/// The ICM IMU struct is the base of the driver. Instantiate this struct in your application code then use
/// it to interact with the IMU.
pub struct IcmImu<BUS> {
    bus: BUS,
    acc_en: AccStates,
    gyro_en: GyroStates,
    mag_en: MagStates,
    temp_en: TempStates,

    databuf: [u8; 5],

    accel_sen: u16,
    gyro_sen: f32,

    int_enabled: bool,

    addr: u8,
}

impl<BUS, E> IcmImu<BUS>
where
    BUS: i2c::WriteRead<u8, Error = E> + i2c::Write<u8, Error = E>,
{
    /// Create and initialize a new IMU driver.
    ///
    /// The I2C bus is given as `bus`
    ///
    /// The 7-bit address is specified as `addr`
    pub fn new(mut bus: BUS, addr: u8) -> Result<Self, IcmError<E>> {
        let mut buf = [0; 3];
        buf[0] = RegistersBank0::PwrMgmt1.get_addr(WRITE_REG);
        buf[1] = 0x01;
        bus.write(addr, &buf[0..2])?;

        Ok(IcmImu {
            bus,
            acc_en: AccOn,
            gyro_en: GyroOn,
            mag_en: MagOff,
            temp_en: TempOn,
            accel_sen: ACCEL_SEN_0,
            gyro_sen: GYRO_SEN_0,
            int_enabled: INT_NOT_ENABLED,
            databuf: [0; 5],
            addr,
        })
    }

    /// Who Am I? Reads the wai register and reports the value.
    ///
    /// Useful for testing that the IMU is properly connected. See the data sheet for the expected return value.
    pub fn wai(&mut self) -> Result<u8, IcmError<E>> {
        let mut buf = [0];
        self.databuf[0] = RegistersBank0::Wai.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf)?;
        Ok(buf[0])
    }

    /// Enables the accelerometer.
    pub fn enable_acc(&mut self) -> Result<(), IcmError<E>> {
        let mut buf = [0];
        self.databuf[0] = RegistersBank0::PwrMgmt2.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf)?;
        self.databuf[1] = buf[0] & 0x07;
        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.acc_en = AccOn;
        Ok(())
    }

    /// Disables the accelerometer.
    pub fn disable_acc(&mut self) -> Result<(), IcmError<E>> {
        let mut buf = [0];
        self.databuf[0] = RegistersBank0::PwrMgmt2.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf)?;
        self.databuf[1] = buf[0] | 0x38;
        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.acc_en = AccOff;
        Ok(())
    }

    /// Enables the gyro.
    pub fn enable_gyro(&mut self) -> Result<(), IcmError<E>> {
        let mut buf = [0];
        self.databuf[0] = RegistersBank0::PwrMgmt2.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..2], &mut buf)?;
        self.databuf[1] = buf[0] & 0x38;
        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.gyro_en = GyroOn;
        Ok(())
    }

    /// Disables the gyro.
    pub fn disable_gyro(&mut self) -> Result<(), IcmError<E>> {
        let mut buf = [0];
        self.databuf[0] = RegistersBank0::PwrMgmt2.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..2], &mut buf)?;
        self.databuf[1] = buf[0] | 0x07;
        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.gyro_en = GyroOff;
        Ok(())
    }

    /// Enable the raw data ready interrupt.
    pub fn enable_int(&mut self) -> Result<(), IcmError<E>> {
        self.databuf[0] = RegistersBank0::IntEnable1.get_addr(WRITE_REG);
        self.databuf[1] = 0x01;
        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.int_enabled = INT_ENABLED;
        Ok(())
    }

    /// Disable the raw data ready interrupt.
    pub fn disable_int(&mut self) -> Result<(), IcmError<E>> {
        self.databuf[0] = RegistersBank0::IntEnable1.get_addr(WRITE_REG);
        self.databuf[1] = 0x00;
        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.int_enabled = INT_NOT_ENABLED;
        Ok(())
    }

    /// Checks if the interrupt is enabled. Returns true if enabled, false if otherwise.
    pub fn int_on(&self) -> bool {
        self.int_enabled == INT_ENABLED
    }

    /// Checks if the gyro is enabled. Returns true if enabled, false if otherwise.
    pub fn gyro_on(&self) -> bool {
        self.gyro_en == GyroOn
    }

    /// Checks if the accelerometer is enabled. Returns true if enabled, false if otherwise.
    pub fn acc_on(&self) -> bool {
        self.acc_en == AccOn
    }

    /// Checks if the magnetometer is enabled. Returns true if enabled, false if otherwise.
    pub fn mag_on(&self) -> bool {
        self.mag_en == MagOn
    }

    /// Gets the accelerometer reading in the X direction.
    /// If the accelerometer is off, this function will first turn it on.
    pub fn read_acc_x(&mut self) -> Result<f32, IcmError<E>> {
        if self.acc_en == AccOff {
            self.enable_acc()?;
        }

        let mut buf = [0, 0];
        self.databuf[0] = RegistersBank0::AccelXOutH.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf[0..1])?;

        self.databuf[0] = RegistersBank0::AccelXOutL.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf[1..2])?;

        Ok((((buf[0] as i16) << 8) | (buf[1] as i16)) as f32 / self.accel_sen as f32)
    }

    /// Gets the accelerometer reading in the Y direction.
    /// If the accelerometer is off, this function will first turn it on.
    pub fn read_acc_y(&mut self) -> Result<f32, IcmError<E>> {
        if self.acc_en == AccOff {
            self.enable_acc()?;
        }

        let mut buf = [0, 0];
        self.databuf[0] = RegistersBank0::AccelYOutH.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf[0..1])?;

        self.databuf[0] = RegistersBank0::AccelYOutL.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf[1..2])?;

        Ok((((buf[0] as i16) << 8) | (buf[1] as i16)) as f32 / self.accel_sen as f32)
    }

    /// Gets the accelerometer reading in the Z direction.
    /// If the accelerometer is off, this function will first turn it on.
    pub fn read_acc_z(&mut self) -> Result<f32, IcmError<E>> {
        if self.acc_en == AccOff {
            self.enable_acc()?;
        }

        let mut buf = [0, 0];
        self.databuf[0] = RegistersBank0::AccelZOutH.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf[0..1])?;

        self.databuf[0] = RegistersBank0::AccelZOutL.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf[1..2])?;

        Ok((((buf[0] as i16) << 8) | (buf[1] as i16)) as f32 / self.accel_sen as f32)
    }

    /// Reads all three accelerometer values and returns them as an array.
    /// If the accelerometer is off, this function will first turn it on.
    pub fn read_acc(&mut self) -> Result<[f32; 3], IcmError<E>> {
        let mut buf = [0.0, 0.0, 0.0];
        buf[0] = self.read_acc_x()?;
        buf[1] = self.read_acc_y()?;
        buf[2] = self.read_acc_z()?;

        Ok(buf)
    }

    /// Gets the gyro reading in the X direction.
    /// If the gyroscope is off, this function will first turn it on.
    pub fn read_gyro_x(&mut self) -> Result<f32, IcmError<E>> {
        if self.gyro_en == GyroOff {
            self.enable_gyro()?;
        }

        let mut buf = [0, 0];
        self.databuf[0] = RegistersBank0::GyroXOutH.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf[0..1])?;

        self.databuf[0] = RegistersBank0::GyroXOutL.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf[1..2])?;

        Ok((((buf[0] as i16) << 8) | (buf[1] as i16)) as f32 / self.accel_sen as f32)
    }

    /// Gets the gyro reading in the Y direction.
    /// If the gyroscope is off, this function will first turn it on.
    pub fn read_gyro_y(&mut self) -> Result<f32, IcmError<E>> {
        if self.gyro_en == GyroOff {
            self.enable_gyro()?;
        }

        let mut buf = [0, 0];
        self.databuf[0] = RegistersBank0::GyroYOutH.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf[0..1])?;

        self.databuf[0] = RegistersBank0::GyroYOutL.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf[1..2])?;

        Ok((((buf[0] as i16) << 8) | (buf[1] as i16)) as f32 / self.accel_sen as f32)
    }

    /// Gets the gyro reading in the Z direction.
    /// If the gyroscope is off, this function will first turn it on.
    pub fn read_gyro_z(&mut self) -> Result<f32, IcmError<E>> {
        if self.gyro_en == GyroOff {
            self.enable_gyro()?;
        }

        let mut buf = [0, 0];
        self.databuf[0] = RegistersBank0::GyroZOutH.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf[0..1])?;

        self.databuf[0] = RegistersBank0::GyroZOutL.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf[1..2])?;

        Ok((((buf[0] as i16) << 8) | (buf[1] as i16)) as f32 / self.accel_sen as f32)
    }

    /// Reads all three gyro values and returns them as an array.
    /// If the gyroscope is off, this function will first turn it on.
    pub fn read_gyro(&mut self) -> Result<[f32; 3], IcmError<E>> {
        let mut buf = [0.0; 3];
        buf[0] = self.read_gyro_x()?;
        buf[1] = self.read_gyro_y()?;
        buf[2] = self.read_gyro_z()?;

        Ok(buf)
    }

    /// Sets the sensitivity of the gyro.
    ///
    /// `gyro_sen` specifies the desired sensitivity. See the data sheet for more details.
    pub fn set_gyro_sen(&mut self, gyro_sen: GyroSensitivity) -> Result<(), IcmError<E>> {
        self.change_bank(REG_BANK_2)?;

        let mut buf = [0];

        self.databuf[0] = RegistersBank2::GyroConfig1.get_addr(WRITE_REG);
        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf)?;

        match gyro_sen {
            GyroSensitivity::Sen250dps => self.databuf[1] = buf[0] & 0xF9,
            GyroSensitivity::Sen500dps => self.databuf[1] = (buf[0] & 0xFB) | 0x02,
            GyroSensitivity::Sen1000dps => self.databuf[1] = (buf[0] & 0xFD) | 0x04,
            GyroSensitivity::Sen2000dps => self.databuf[1] = buf[0] | 0x06,
        };

        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.change_bank(REG_BANK_0)?;

        match gyro_sen {
            GyroSensitivity::Sen250dps => self.gyro_sen = GYRO_SEN_0,
            GyroSensitivity::Sen500dps => self.gyro_sen = GYRO_SEN_1,
            GyroSensitivity::Sen1000dps => self.gyro_sen = GYRO_SEN_2,
            GyroSensitivity::Sen2000dps => self.gyro_sen = GYRO_SEN_3,
        }

        Ok(())
    }

    /// Sets the sensitivity of the accelerometer.
    ///
    /// `acc_sen` specifies the desired sensitivity. See the data sheet for more details.
    pub fn set_acc_sen(&mut self, acc_sen: AccSensitivity) -> Result<(), IcmError<E>> {
        self.change_bank(REG_BANK_2)?;
        let mut buf = [0];

        self.databuf[0] = RegistersBank2::AccelConfig.get_addr(WRITE_REG);

        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf)?;

        match acc_sen {
            AccSensitivity::Sen2g => self.databuf[1] = buf[0] & 0xF9,
            AccSensitivity::Sen4g => self.databuf[1] = (buf[0] & 0xFB) | 0x02,
            AccSensitivity::Sen8g => self.databuf[1] = (buf[0] & 0xFD) | 0x04,
            AccSensitivity::Sen16g => self.databuf[1] = buf[0] | 0x06,
        };

        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.change_bank(REG_BANK_0)?;

        match acc_sen {
            AccSensitivity::Sen2g => self.accel_sen = ACCEL_SEN_0,
            AccSensitivity::Sen4g => self.accel_sen = ACCEL_SEN_1,
            AccSensitivity::Sen8g => self.accel_sen = ACCEL_SEN_2,
            AccSensitivity::Sen16g => self.accel_sen = ACCEL_SEN_3,
        }
        Ok(())
    }

    /// Configures the Low Pass Filter (LPF) for the accelerometer.
    ///
    /// `bw` is the 3DB bandwidth of the LPF. See the data sheet for more details.
    pub fn config_acc_lpf(&mut self, bw: AccLPF) -> Result<(), IcmError<E>> {
        self.change_bank(REG_BANK_2)?;

        let mut buf = [0];
        self.databuf[0] = RegistersBank2::AccelConfig.get_addr(WRITE_REG);

        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf)?;

        let mask: u8 = 0x38;

        match bw {
            AccLPF::BW6 => self.databuf[1] = (buf[0] & !mask) | ((6 << 3) & mask),
            AccLPF::BW11 => self.databuf[1] = (buf[0] & !mask) | ((5 << 3) & mask),
            AccLPF::BW24 => self.databuf[1] = (buf[0] & !mask) | ((4 << 3) & mask),
            AccLPF::BW50 => self.databuf[1] = (buf[0] & !mask) | ((3 << 3) & mask),
            AccLPF::BW111 => self.databuf[1] = (buf[0] & !mask) | ((2 << 3) & mask),
            AccLPF::BW246 => self.databuf[1] = (buf[0] & !mask) | ((0 << 3) & mask),
            AccLPF::Bw473 => self.databuf[1] = (buf[0] & !mask) | ((7 << 3) & mask),
        }

        self.databuf[1] = self.databuf[1] | 0x01;

        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.change_bank(REG_BANK_0)?;
        Ok(())
    }

    /// Configures the Low Pass Filter (LPF) for the gyro.
    ///
    /// `bw` is the 3DB bandwidth of the LPF. See the data sheet for more details.
    pub fn config_gyro_lpf(&mut self, bw: GyroLPF) -> Result<(), IcmError<E>> {
        self.change_bank(REG_BANK_2)?;

        let mut buf = [0];
        self.databuf[0] = RegistersBank2::GyroConfig1.get_addr(WRITE_REG);

        self.bus
            .write_read(self.addr, &self.databuf[0..2], &mut buf)?;

        let mask: u8 = 0x38;

        match bw {
            GyroLPF::BW6 => self.databuf[1] = (buf[0] & !mask) | ((6 << 3) & mask),
            GyroLPF::BW12 => self.databuf[1] = (buf[0] & !mask) | ((5 << 3) & mask),
            GyroLPF::BW24 => self.databuf[1] = (buf[0] & !mask) | ((4 << 3) & mask),
            GyroLPF::BW51 => self.databuf[1] = (buf[0] & !mask) | ((3 << 3) & mask),
            GyroLPF::BW119 => self.databuf[1] = (buf[0] & !mask) | ((2 << 3) & mask),
            GyroLPF::BW152 => self.databuf[1] = (buf[0] & !mask) | ((1 << 3) & mask),
            GyroLPF::BW197 => self.databuf[1] = (buf[0] & !mask) | ((0 << 3) & mask),
            GyroLPF::BW361 => self.databuf[1] = (buf[0] & !mask) | ((7 << 3) & mask),
        }

        self.databuf[1] = self.databuf[1] | 0x01;

        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.change_bank(REG_BANK_0)?;
        Ok(())
    }

    /// Disables the low pass filter for the accelerometer.
    pub fn disable_acc_lpf(&mut self) -> Result<(), IcmError<E>> {
        self.change_bank(REG_BANK_2)?;

        let mut buf = [0];
        self.databuf[0] = RegistersBank2::AccelConfig.get_addr(WRITE_REG);

        self.bus
            .write_read(self.addr, &self.databuf[0..1], &mut buf)?;

        self.databuf[1] = buf[0] & 0xFE;

        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.change_bank(REG_BANK_0)?;

        Ok(())
    }

    /// Disables the low pass filter for the gyro.
    pub fn disable_gyro_lpf(&mut self) -> Result<(), IcmError<E>> {
        self.change_bank(REG_BANK_2)?;

        let mut buf = [0];
        self.databuf[0] = RegistersBank2::GyroConfig1.get_addr(WRITE_REG);

        self.bus
            .write_read(self.addr, &self.databuf[0..2], &mut buf)?;

        self.databuf[1] = buf[0] & 0xFE;

        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.change_bank(REG_BANK_0)?;

        Ok(())
    }

    /// Configure the data rate for the accelerometer.
    ///
    /// Rate must be less than 1.125kHz since that is the maximum of the accelerometer. Rate is specified in Hz.
    /// If rate is > 1.125kHz then an InvalidInput will be returned.
    /// Since the divisor is specified as an integer, the exact rate may not be met if it would require a floating point divisor.
    ///
    /// Note that while the gyro is enabled the gyro ODR (output data rate) will determine the interrupt frequency.
    /// If the accelerometer frequency is lower than the gyro frequency then it will not be updated at every interrupt.
    pub fn config_acc_rate(&mut self, rate: u16) -> Result<(), IcmError<E>> {
        if rate < 1_125 {
            let div = 1_125 / (rate) - 1;

            self.change_bank(REG_BANK_2)?;

            self.databuf[0] = RegistersBank2::OdrAlignEn.get_addr(WRITE_REG);
            self.databuf[1] = 0x01;

            self.bus.write(self.addr, &self.databuf[0..2])?;

            self.databuf[0] = RegistersBank2::AccelSmplrtDiv1.get_addr(WRITE_REG);
            self.databuf[1] = div.to_be_bytes()[0];
            self.databuf[2] = RegistersBank2::AccelSmplrtDiv2.get_addr(WRITE_REG);
            self.databuf[3] = div.to_be_bytes()[1];

            self.bus.write(self.addr, &self.databuf[0..4])?;

            self.change_bank(REG_BANK_0)?;

            Ok(())
        } else {
            Err(IcmError::InvalidInput)
        }
    }

    /// Configure the data rate for the gyro.
    ///
    /// Rate must be less than 1.125kHz since that is the maximum of the gyro. Rate is specified in Hz.
    /// If rate is > 1.125kHz then an InvalidInput will be returned.
    /// Rate must be > 4 otherwise the divisor will not fit in a u8.
    ///
    /// Since the divisor is specified as an integer, the exact rate may not be met if it would require a floating point divisor.
    ///
    /// Note that while the gyro is enabled the gyro ODR (output data rate) will determine the interrupt frequency.
    pub fn config_gyro_rate(&mut self, rate: u16) -> Result<(), IcmError<E>> {
        if rate > 4 && rate < 1125 {
            let div: u8 = (1_125 / (rate) - 1) as u8;

            self.change_bank(REG_BANK_2)?;

            self.databuf[0] = RegistersBank2::OdrAlignEn.get_addr(WRITE_REG);
            self.databuf[1] = 0x01;

            self.bus.write(self.addr, &self.databuf[0..2])?;

            self.databuf[0] = RegistersBank2::GyroSmplrtDiv.get_addr(WRITE_REG);
            self.databuf[1] = div;

            self.bus.write(self.addr, &self.databuf[0..2])?;

            self.change_bank(REG_BANK_0)?;

            Ok(())
        } else {
            Err(IcmError::InvalidInput)
        }
    }

    /// Configure the accelerometer data rate by directly modifying the sample rate divider.
    ///
    /// The user should calculate what the resulting data rate will be before using this function.
    ///
    /// div specifies the divider and must be less than 4096.
    pub fn config_acc_rate_div(&mut self, mut div: u16) -> Result<(), IcmError<E>> {
        if div < 4096 {
            div = div - 1;

            self.change_bank(REG_BANK_2)?;

            self.databuf[0] = RegistersBank2::OdrAlignEn.get_addr(WRITE_REG);
            self.databuf[1] = 0x01;

            self.bus.write(self.addr, &self.databuf[0..2])?;

            self.databuf[0] = RegistersBank2::AccelSmplrtDiv1.get_addr(WRITE_REG);
            self.databuf[1] = div.to_be_bytes()[0];
            self.databuf[2] = RegistersBank2::AccelSmplrtDiv2.get_addr(WRITE_REG);
            self.databuf[3] = div.to_be_bytes()[1];

            self.bus.write(self.addr, &self.databuf[0..4])?;

            self.change_bank(REG_BANK_0)?;

            Ok(())
        } else {
            Err(IcmError::InvalidInput)
        }
    }

    /// Configure the gyroscope data rate by directly modifying the sample rate divider.
    ///
    /// The user should calculate what the resulting data rate will be before using this function.
    ///
    /// div specifies the divider to use.
    pub fn config_gyro_rate_div(&mut self, mut div: u8) -> Result<(), IcmError<E>> {
        div = div - 1;

        self.change_bank(REG_BANK_2)?;

        self.databuf[0] = RegistersBank2::OdrAlignEn.get_addr(WRITE_REG);
        self.databuf[1] = 0x01;

        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.databuf[0] = RegistersBank2::GyroSmplrtDiv.get_addr(WRITE_REG);
        self.databuf[1] = div;

        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.change_bank(REG_BANK_0)?;

        Ok(())
    }

    // ----------------- Temperature Sensor ----------------- //
    /// Enable the temperature sensor
    pub fn enable_temp(&mut self) -> Result<(), IcmError<E>> {
        self.databuf[0] = RegistersBank0::PwrMgmt1.get_addr(WRITE_REG);
        let mut buf = [0];
        self.bus.write_read(self.addr, &self.databuf[0..2], &mut buf[0..1])?;
        self.databuf[1] = buf[0] | 0x08;
        self.bus.write(self.addr, &self.databuf[0..2])?;
        self.temp_en = TempOn;
        Ok(())
    }

    /// Disable the temperature sensor
    pub fn disable_temp(&mut self) -> Result<(), IcmError<E>> {
        self.databuf[0] = RegistersBank0::PwrMgmt1.get_addr(WRITE_REG);
        let mut buf = [0];
        self.bus.write_read(self.addr, &self.databuf[0..2], &mut buf[0..1])?;
        self.databuf[1] = buf[0] & 0xF7;
        self.bus.write(self.addr, &self.databuf[0..2])?;
        self.temp_en = TempOff;
        Ok(())
    }

    /// Read the temperature sensor.
    ///
    /// Returns the temperature in Celsius.
    ///
    /// TODO: More research on the room_temp_offset
    pub fn read_temp(&mut self) -> Result<f32, IcmError<E>> {
        if self.temp_en ==  TempOff {
           self.enable_temp()?;
        }

        self.databuf[0] = RegistersBank0::TempOutH.get_addr(WRITE_REG);
        self.databuf[1] = RegistersBank0::TempOutL.get_addr(WRITE_REG);

        let mut buf = [0, 0];
        self.bus.write_read(self.addr, &self.databuf[0..2], &mut buf[0..2])?;

        let room_temp = 0;
        let temp = (((((buf[0] as u16) << 8) | buf[1] as u16) - room_temp) as f32)/TEMP_SEN + 21.0;
        Ok(temp)
    }

    // ----------------- Magnetometer ----------------- //
    // This driver will access the magnetometer through the I2C master feature
    // This is more for SPI
    // - Enable the I2C master feature
    // - May need to adjust the I2C master ODR if gyro and accelerometer are disabled
    // - Set the I2C_MST_CTRL register for bus speed and stop between read
    // - Set the I2C_MST_DELAY_CTRL register for SLV0
    // - Set I2C_SLV0_ADDR to point to the magnetometer (0x0C I think) and write mode so I can configure it
    // - Set I2C_SLV0_REG to point to the first register I want to read (CTRL2)
    // - Set I2C_SLV0_DO to the desired control setting
    // - Go to read mode now
    // - Set I2C_SLV0_REG to HXL
    // - Now EXT_SLV_DATA_00-06 will now be set with HXL-ST2. Need to read ST2 to trigger the data update in the axis registers

    // For I2C:
    // - Configure the Magnetometer using normal I2C methods. Then switch over.

    /// Enable the magnetometer for use.
    ///
    /// Data rate specifies how often the magnetometer makes measurements.
    pub fn enable_mag(&mut self, data_rate: MagRates) -> Result<(), IcmError<E>> {
        let mut buf = [0];
        self.databuf[0] = RegistersBank0::UserCtrl.get_addr(WRITE_REG);
        self.bus.write_read(self.addr, &self.databuf[0..1], &mut buf[0..1])?;

        self.databuf[1] = buf[0] | 0x20;
        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.change_bank(REG_BANK_3)?;

        self.databuf[0] = RegistersBank3::I2cMstCtrl.get_addr(WRITE_REG);
        self.databuf[1] = 0x17;
        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.databuf[0] = RegistersBank3::I2cMstDelayCtrl.get_addr(WRITE_REG);
        self.databuf[1] = 0x01;
        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.databuf[2] = RegistersBank3::I2cSlv0Addr.get_addr(WRITE_REG);
        self.databuf[3] = MAG_ADDR;
        self.bus.write(self.addr, &self.databuf[2..4])?;

        self.databuf[0] = RegistersBank3::I2cSlv0Reg.get_addr(WRITE_REG);
        self.databuf[1] = 0x31;
        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.databuf[0] = RegistersBank3::I2cSlvDo.get_addr(WRITE_REG);
        match data_rate {
            MagRates::Mag10Hz => self.databuf[1] = 0x02,
            MagRates::Mag20Hz => self.databuf[1] = 0x04,
            MagRates::Mag50Hz => self.databuf[1] = 0x06,
            MagRates::Mag100Hz => self.databuf[1] = 0x08,
            MagRates::MagSingle => self.databuf[1] = 0x01,
        }
        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.databuf[0] = RegistersBank3::I2cSlv0Ctrl.get_addr(WRITE_REG);
        self.databuf[1] = 0x89;
        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.databuf[3] = 1 << 7 | MAG_ADDR;
        self.bus.write(self.addr, &self.databuf[2..4])?;

        self.databuf[0] = RegistersBank3::I2cSlv0Reg.get_addr(WRITE_REG);
        self.databuf[1] = 0x10;
        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.change_bank(REG_BANK_0)?;
        self.mag_en = MagOn;
        Ok(())
    }

    /// Disable the magnetometer.
    pub fn disable_mag(&mut self) -> Result<(), IcmError<E>> {
        unimplemented!();
    }

    /// Reads the magnetometer x axis.
    /// If the magnetometer is off, this function will first turn it on with 50Hz sample rate.
    fn read_mag_x(&mut self) -> Result<f32, IcmError<E>> {
        if self.mag_en == MagOff {
            self.enable_mag(MagRates::Mag50Hz)?;
        }

        let mut buf = [0, 0];

        self.databuf[0] = RegistersBank0::ExtSlvSensData00.get_addr(WRITE_REG);
        self.bus.write_read(self.addr, &self.databuf[0..1], &mut buf[0..1])?;
        self.databuf[1] = RegistersBank0::ExtSlvSensData01.get_addr(WRITE_REG);
        self.bus.write_read(self.addr, &self.databuf[1..2], &mut buf[1..2])?;

        let res = ((((buf[1] as i16) << 8) | (buf[0] as i16)) as f32) * MAG_SEN;
        Ok(res)
    }

    /// Reads the magnetometer y axis.
    /// If the magnetometer is off, this function will first turn it on with 50Hz sample rate.
    fn read_mag_y(&mut self) -> Result<f32, IcmError<E>> {
        if self.mag_en == MagOff {
            self.enable_mag(MagRates::Mag50Hz)?;
        }

        let mut buf = [0, 0];

        self.databuf[0] = RegistersBank0::ExtSlvSensData02.get_addr(WRITE_REG);
        self.bus.write_read(self.addr, &self.databuf[0..1], &mut buf[0..1])?;
        self.databuf[1] = RegistersBank0::ExtSlvSensData03.get_addr(WRITE_REG);
        self.bus.write_read(self.addr, &self.databuf[1..2], &mut buf[1..2])?;

        let res = ((((buf[1] as i16) << 8) | (buf[0] as i16)) as f32) * MAG_SEN;
        Ok(res)
    }

    /// Reads the magnetometer z axis.
    /// If the magnetometer is off, this function will first turn it on with 50Hz sample rate.
    fn read_mag_z(&mut self) -> Result<f32, IcmError<E>> {
        if self.mag_en == MagOff {
            self.enable_mag(MagRates::Mag50Hz)?;
        }

        let mut buf = [0, 0];

        self.databuf[0] = RegistersBank0::ExtSlvSensData04.get_addr(WRITE_REG);
        self.bus.write_read(self.addr, &self.databuf[0..1], &mut buf[0..1])?;
        self.databuf[1] = RegistersBank0::ExtSlvSensData05.get_addr(WRITE_REG);
        self.bus.write_read(self.addr, &self.databuf[1..2], &mut buf[1..2])?;

        let res = ((((buf[1] as i16) << 8) | (buf[0] as i16)) as f32) * MAG_SEN;
        Ok(res)
    }

    /// Reads all axis of the magnetometer.
    /// If the magnetometer is off, this function will first turn it on with 50Hz sample rate.
    ///
    /// Returns the values in the order: x, y, z
    pub fn read_mag(&mut self) -> Result<[f32; 3], IcmError<E>> {
        self.change_bank(REG_BANK_3)?;
        self.databuf[0] = RegistersBank3::I2cSlv0Ctrl.get_addr(WRITE_REG);
        self.databuf[1] = 0x89;
        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.databuf[0] = RegistersBank3::I2cSlv0Addr.get_addr(WRITE_REG);
        self.databuf[1] = 1 << 7 | MAG_ADDR;
        self.bus.write(self.addr, &self.databuf[0..2])?;

        self.databuf[0] = RegistersBank3::I2cSlv0Reg.get_addr(WRITE_REG);
        self.databuf[1] = 0x11;
        self.bus.write(self.addr, &self.databuf[0..2])?;
        self.change_bank(REG_BANK_0)?;

        let mut res = [0.0, 0.0, 0.0];
        res[0] = self.read_mag_x()?;
        res[1] = self.read_mag_y()?;
        res[2] = self.read_mag_z()?;

        let mut buf = [0];
        self.databuf[0] = RegistersBank0::ExtSlvSensData06.get_addr(WRITE_REG);
        self.bus.write_read(self.addr, &self.databuf[0..1], &mut buf[0..1])?;
        self.databuf[0] = RegistersBank0::ExtSlvSensData07.get_addr(WRITE_REG);
        self.bus.write_read(self.addr, &self.databuf[0..1], &mut buf[0..1])?;
        self.databuf[0] = RegistersBank0::ExtSlvSensData08.get_addr(WRITE_REG);
        self.bus.write_read(self.addr, &self.databuf[0..1], &mut buf[0..1])?;
        self.databuf[0] = RegistersBank0::ExtSlvSensData09.get_addr(WRITE_REG);
        self.bus.write_read(self.addr, &self.databuf[0..1], &mut buf[0..1])?;
        Ok(res)
    }

    // ----------------- Reset ----------------- //
    /// Resets the IMU.
    /// After the reset a 20ms sleep is suggested.
    ///
    /// If the IMU is not reset after writing the registers then the registers keep their same value.
    /// This function is useful for testing code but currently should not be used in production.
    /// You can also reset the IMU by power cycling it.
    pub fn reset(&mut self) -> Result<(), IcmError<E>> {
        self.databuf[0] = RegistersBank0::PwrMgmt1.get_addr(WRITE_REG);
        self.databuf[1] = 0x80;

        self.bus.write(self.addr, &self.databuf[0..2])?;

        Ok(())
    }

    /// Wakes the IMU from sleep mode.
    pub fn wake(&mut self) -> Result<(), IcmError<E>> {
        self.databuf[0] = RegistersBank0::PwrMgmt1.get_addr(WRITE_REG);
        self.databuf[1] = 0x01;

        self.bus.write(self.addr, &mut self.databuf[0..2])?;

        Ok(())
    }

    fn change_bank(&mut self, bank: u8) -> Result<(), IcmError<E>> {
        self.databuf[0] = RegistersBank0::RegBankSel.get_addr(WRITE_REG);
        self.databuf[1] = bank;

        self.bus.write(self.addr, &self.databuf[0..2])?;

        Ok(())
    }
}

impl<BUS> Format for IcmImu<BUS> {
    fn format(&self, fmt: Formatter) {
        defmt::write!(fmt, "ICM-20948 IMU")
    }
}
