// Copyright (c) 2022, Zachary D. Olkin.
// This code is provided under the MIT license.

#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use icm20948_driver::icm20948;
use icm20948_driver::icm20948::i2c as imu_i2c;
use stm32h7xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let device = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    defmt::info!("Setup PWR...");
    let pwr = device.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Constrain and Freeze clock
    defmt::info!("Setup RCC...");
    let rcc = device.RCC.constrain();
    let ccdr = rcc
        .sys_ck(100.MHz())
        .pll1_q_ck(48.MHz())
        .freeze(pwrcfg, &device.SYSCFG);

    defmt::info!("ICM-20948 example - basic_i2c");

    let gpiob = device.GPIOB.split(ccdr.peripheral.GPIOB);
    let sda = gpiob.pb9.into_alternate_open_drain();
    let scl = gpiob.pb8.into_alternate_open_drain();

    let i2c = device
        .I2C1
        .i2c((scl, sda), 300.kHz(), ccdr.peripheral.I2C1, &ccdr.clocks);


    defmt::info!("Setting up IMU...");
    let mut imu = defmt::unwrap!(imu_i2c::IcmImu::new(i2c, 0x68));
    //defmt::unwrap!(imu.reset());
    cortex_m::asm::delay(200_000_000);
    defmt::unwrap!(imu.set_acc_sen(icm20948::AccSensitivity::Sen2g));
    defmt::unwrap!(imu.set_gyro_sen(icm20948::GyroSensitivity::Sen250dps));
    defmt::unwrap!(imu.enable_mag(icm20948::MagRates::Mag100Hz));

    // Configure heartbeat LED
    let gpioe = device.GPIOE.split(ccdr.peripheral.GPIOE);
    let mut led = gpioe.pe1.into_push_pull_output();

    let mut delay = cp.SYST.delay(ccdr.clocks);

    loop {
        loop {
            let acc = defmt::unwrap!(imu.read_acc());

            defmt::debug!(
                "Accelerometer readings (g): X: {}, Y: {}, Z: {}",
                acc[0],
                acc[1],
                acc[2]
            );

            let gyr = defmt::unwrap!(imu.read_gyro());

            defmt::debug!(
                "Gyro readings (dps): X: {}, Y: {}, Z: {}",
                gyr[0],
                gyr[1],
                gyr[2]
            ); // dps is degrees per second

            let temp = defmt::unwrap!(imu.read_temp());
            defmt::debug!(
                "Temperature reading (C): {}",
                temp,
            );

            let mag = defmt::unwrap!(imu.read_mag());
            defmt::debug!(
                "Mag readings (uT): X: {}, Y: {}, Z: {}",
                mag[0],
                mag[1],
                mag[2]
            );

            led.set_high();
            delay.delay_ms(50_u16);

            led.set_low();
            delay.delay_ms(50_u16);
        }
    }
}
