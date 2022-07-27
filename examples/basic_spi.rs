#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use icm20948_driver::icm20948;
use icm20948_driver::icm20948::spi as imu_spi;
use stm32h7xx_hal::{pac, prelude::*, spi};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let device = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    defmt::info!("Setup PWR...                  ");
    let pwr = device.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Constrain and Freeze clock
    defmt::info!("Setup RCC...                  ");
    let rcc = device.RCC.constrain();
    let ccdr = rcc
        .sys_ck(100.MHz())
        .pll1_q_ck(48.MHz())
        .freeze(pwrcfg, &device.SYSCFG);

    defmt::info!("ICM-20948 example - basic_spi");

    let gpioa = device.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = device.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpiod = device.GPIOD.split(ccdr.peripheral.GPIOD);

    let sck = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpiob.pb5.into_alternate();
    let mut cs = gpiod.pd15.into_push_pull_output();
    cs.set_high();

    let spi1: spi::Spi<_, _, u8> = device.SPI1.spi(
        (sck, miso, mosi),
        spi::MODE_0,
        3.MHz(),
        ccdr.peripheral.SPI1,
        &ccdr.clocks,
    );

    let mut imu = defmt::unwrap!(imu_spi::IcmImu::new(spi1, cs));
    defmt::unwrap!(imu.set_acc_sen(icm20948::AccSensitivity::Sen2g));
    defmt::unwrap!(imu.set_gyro_sen(icm20948::GyroSensitivity::Sen250dps));

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

            led.set_high();
            delay.delay_ms(500_u16);

            led.set_low();
            delay.delay_ms(500_u16);
        }
    }
}
