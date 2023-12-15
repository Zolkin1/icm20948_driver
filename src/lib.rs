// Copyright (c) 2022, Zachary D. Olkin.
// This code is provided under the MIT license.

//! Device agnostic driver for the icm-20948 IMU (inertial measurement unit).
//! The driver depends on [embedded-hal](https://crates.io/crates/embedded-hal/), so as long as the HAL you use implements those traits, then
//! this driver will be compatible.
//!
//! The driver is split into two parts: an I2C part and an SPI part. All functions are implemented for both bus types.
//!
//! You can instantiate multiple objects if you have multiple IMUs. The driver assumes control of
//! the bus and thus it is recommended to use [shared-bus](https://crates.io/crates/shared-bus) if you have multiple device drivers that need control of the bus.
//!
//! Currently, there is no support for the magnetometer, temperature sensor, the sensor acting like an I2C master, the FIFO, the FSYNC, or the DMP.
//! I am hoping to add support for the FIFO, magnetometer, and temperature sensor soon. See the change log in the repo for recent changes.
//!
//! # Examples
//! Examples can be found [here](https://github.com/Zolkin1/icm20948_driver/tree/main/examples).
//!
//! # Features
//! This crate has implemented [defmt](https://crates.io/crates/defmt) for the IcmError type. This can be enabled as a feature of this crate.
//! This implementation assumes that the SPI/I2C error type also implements [defmt](https://crates.io/crates/defmt).
//! The HAL that you are using may have a feature you need to enable for this.
//!
//! # Data Sheet
//! The data sheet for this device can be found [here](https://invensense.tdk.com/download-pdf/icm-20948-datasheet/).
//!
//! # Repository
//! The github repo can be found [here](https://github.com/Zolkin1/icm20948_driver).

#![deny(missing_docs)]
#![no_std]
#![no_main]

use defmt_rtt as _; // global logger

/// Main module that holds the SPI and I2C sub modules.
/// Also holds many enums and constants shared between SPI and I2C
pub mod icm20948;
