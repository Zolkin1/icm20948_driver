#![no_main]
#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};

use defmt_rtt as _; // global logger

use panic_probe as _;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

use icm20948_driver as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
device = stm32h7xx_hal::pac,
dispatchers = [SPI1]
)]
mod app {
    use systick_monotonic::{fugit::Duration, Systick};
    use stm32h7xx_hal::gpio::{self, Output, PushPull};
    use stm32h7xx_hal::pac::SPI1;
    use stm32h7xx_hal::prelude::*;
    use stm32h7xx_hal::spi;
    use icm20948_driver::icm20948;
    use defmt::unwrap;

    pub const MONO_TICK_RATE: u32 = 100;
    pub const SYS_TICK_RATE: u32 = 100_000_000;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<MONO_TICK_RATE>;

    // Shared resources go here
    #[shared]
    struct Shared {
    }

    // Local resources go here
    #[local]
    struct Local {
        led: gpio::PE1<Output<PushPull>>,
        state: bool,
        imu: icm20948::IcmImu<spi::Spi<SPI1, spi::Enabled>, gpio::PD15<Output>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("Init");

        let core: cortex_m::Peripherals = cx.core;
        let device: stm32h7xx_hal::stm32::Peripherals = cx.device;

        let mono: Mono = Systick::new(core.SYST, SYS_TICK_RATE);


        defmt::info!("Setting up Power...");
        let pwr = device.PWR.constrain();
        let pwrcfg = pwr.freeze();

        defmt::info!("Setting up RCC...");
        let rcc = device.RCC.constrain();
        let ccdr = rcc
            .sys_ck(SYS_TICK_RATE.Hz())
            .pll1_q_ck(48.MHz())
            .freeze(pwrcfg, &device.SYSCFG);


        let gpioe = device.GPIOE.split(ccdr.peripheral.GPIOE);

        // Configure PE1 as output.
        let led = gpioe.pe1.into_push_pull_output();

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

        let mut imu = unwrap!(icm20948::IcmImu::new(spi1, cs));

        unwrap!(imu.set_acc_sen(icm20948::AccSensitivity::Sen8g));
        unwrap!(imu.set_gyro_sen(icm20948::GyroSensitivity::Sen1000dps));

        unwrap!(imu.config_acc_lpf(icm20948::AccLPF::BW111));
        unwrap!(imu.config_gyro_lpf(icm20948::GyroLPF::BW119));
        unwrap!(imu.config_acc_rate(100));
        unwrap!(imu.config_gyro_rate(500));

        heartbeat::spawn_after(Duration::<u64, 1, MONO_TICK_RATE>::from_ticks(MONO_TICK_RATE.into())).unwrap();
        imufn::spawn_after(Duration::<u64, 1, MONO_TICK_RATE>::from_ticks(MONO_TICK_RATE.into())).unwrap();
        // Setup the monotonic timer
        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                led,
                state: false,
                imu,
            },
            init::Monotonics(
                mono
            ),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            //rtic::export::wfi()
        }
    }

    #[task(local = [led, state])]
    fn heartbeat(cx: heartbeat::Context) {
        if *cx.local.state {
            cx.local.led.set_high();
            *cx.local.state = false;
            defmt::trace!("Heartbeat: LED set high");
        } else {
            cx.local.led.set_low();
            *cx.local.state = true;
            defmt::trace!("Heartbeat: LED set low");
        }

        heartbeat::spawn_after(Duration::<u64, 1, MONO_TICK_RATE>::from_ticks(MONO_TICK_RATE.into())).unwrap();
    }

    #[task(local = [imu])]
    fn imufn(cx: imufn::Context) {
        let res = cx.local.imu.wai();
        defmt::trace!("WAI: {:#01x}", unwrap!(res));

        let acc = unwrap!(cx.local.imu.read_acc());

        defmt::debug!("Accelerometer readings (g): X: {}, Y: {}, Z: {}", acc[0], acc[1], acc[2]);

        let gyr = unwrap!(cx.local.imu.read_gyro());

        defmt::debug!("Gyro readings (dps): X: {}, Y: {}, Z: {}", gyr[0], gyr[1], gyr[2]);          // dps is degrees per second

        imufn::spawn_after(Duration::<u64, 1, MONO_TICK_RATE>::from_ticks(MONO_TICK_RATE.into())).unwrap();
    }
}
