// Copyright (c) 2022, Zachary D. Olkin.
// This code is provided under the MIT license.

//! Device agnostic driver for the icm-20948 IMU (inertial measurement unit).
//! The driver depends on embedded-hal, so as long as the HAL you use implements those traits, then
//! this driver should be compatible.
//!
//! The data sheet for this device can be found [here](https://invensense.tdk.com/download-pdf/icm-20948-datasheet/).
//!
//! The driver is split into two parts: an I2C part and an SPI part. All functions are implemented for both bus types.
//!
//! This package is still mostly experimental. By v0.2.0 I am hoping most the large kinks are ironed out.
//! Extensive testing has not been done on the LPFs and data rates, although that will come soon.
//!
//! You can instantiate multiple objects if you have multiple IMUs. The driver assumes control of
//! the bus and thus it is recommended to use shared-bus.
//!
//! Currently, there is no support for the magnetometer, the sensor acting like an I2C master, the FIFO, the FSYNC, or the DMP.
//! I am hoping to add support for the FIFO and magnetometer soon. Using the sensor as an I2C master over the auxiliary bus is unlikely to ever happen.
//!
//! Examples can be found [here](https://github.com/Zolkin1/icm20948_driver/tree/main/examples).
//!
//! The github repo can be found [here](https://github.com/Zolkin1/icm20948_driver).

#![deny(missing_docs)]
#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _;

/// Main module that holds the SPI and I2C sub modules.
/// Also holds many enums and constants shared between SPI and I2C
pub mod icm20948;
