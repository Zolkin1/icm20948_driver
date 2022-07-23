#![no_std]
#![no_main]

use icm20948_driver as _; // memory layout + panic handler

#[cfg(not(feature = "i2c"))]
#[defmt_test::tests]
mod tests {
    use defmt::{assert_eq};
    use stm32h7xx_hal::spi;
    use stm32h7xx_hal::gpio;
    use stm32h7xx_hal::prelude::*;
    use icm20948_driver::icm20948;

    struct State {
        imu: icm20948::IcmImu<spi::Spi<stm32h7xx_hal::pac::SPI1, spi::Enabled>, gpio::PD15<gpio::Output>>
    }

    const MONO_TICK_RATE: u32 = 100;
    const SYS_TICK_RATE: u32 = 100_000_000;

    #[init]
    fn init() -> State {
        let device: stm32h7xx_hal::stm32::Peripherals = stm32h7xx_hal::pac::Peripherals::take().unwrap();

        defmt::info!("Setting up Power...");
        let pwr = device.PWR.constrain();
        let pwrcfg = pwr.freeze();

        defmt::info!("Setting up RCC...");
        let rcc = device.RCC.constrain();
        let ccdr = rcc
            .sys_ck(SYS_TICK_RATE.Hz())
            .pll1_q_ck(48.MHz())
            .freeze(pwrcfg, &device.SYSCFG);

        // Configure the SPI bus
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

        let mut imu = defmt::unwrap!(icm20948::IcmImu::new(spi1, cs));

        State {
            imu
        }
    }

    #[test]
    fn wai(state: &mut State) {
        let wai = defmt::unwrap!(state.imu.wai());
        defmt::debug!("TEST: wai: {:#01x}", wai);
        assert_eq!(wai, 0xea);
    }
}

#[cfg(feature = "i2c")]
#[defmt_test::tests]
mod tests {
    use defmt::{assert, assert_eq};

    #[test]
    fn assert_true() {
        assert!(true)
    }

    #[test]
    fn assert_eq() {
        assert_eq!(24, 24)
    }
}