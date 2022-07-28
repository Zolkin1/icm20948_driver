// Copyright (c) 2022, Zachary D. Olkin.
// This code is provided under the MIT license.

//! Device agnostic driver for the icm-20948 IMU (inertial measurement unit).
//! The driver depends on embedded-hal, so as long as the HAL you use implements those traits, then
//! this driver should be compatible.
//!
//! The driver is split into two parts: an I2C part and an SPI part. All functions are implemented for both bus types.
//!
//! You can instantiate multiple objects if you have multiple IMUs.
//!
//! Extensive testing has not been done on the LPFs and data rates, although that will come soon.
//!
//! The driver assumes control of the bus and thus it is recommended to use shared-bus in a multi-threaded context
//!
//! Currently, there is no support for the magnetometer, the sensor acting like an I2C master, the FIFO, the FSYNC, or the DMP.
//! I am hoping to add support for the FIFO and magnetometer soon. Using the sensor as an I2C master over the auxiliary bus is unlikely to ever happen.
//!
//! The github repo can be found at <https://github.com/Zolkin1/icm20948_driver>.

#![deny(missing_docs)]
#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _;

/// Main module that holds the SPI and I2C sub modules.
/// Also holds many enums and constants shared between SPI and I2C
pub mod icm20948;
