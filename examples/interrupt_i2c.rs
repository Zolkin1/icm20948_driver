// Copyright (c) 2022, Zachary D. Olkin.
// This code is provided under the MIT license.

#![deny(warnings)]
#![no_main]
#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};

use defmt_rtt as _; // global logger

use panic_probe as _;

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
    use defmt::unwrap;
    use icm20948_driver::icm20948;
    use stm32h7xx_hal::gpio::{self, Output, Input, Edge, PushPull, ExtiPin};
    use stm32h7xx_hal::i2c;
    use stm32h7xx_hal::pac::I2C1;
    use stm32h7xx_hal::prelude::*;
    use systick_monotonic::{fugit::Duration, Systick};

    pub const MONO_TICK_RATE: u32 = 100;
    pub const SYS_TICK_RATE: u32 = 100_000_000;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<MONO_TICK_RATE>;

    // Shared resources go here
    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    struct Local {
        led: gpio::PE1<Output<PushPull>>,
        state: bool,
        imu: icm20948::i2c::IcmImu<i2c::I2c<I2C1>>,
        int_pin: gpio::PE3<Input>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("Init");

        let core: cortex_m::Peripherals = cx.core;
        let device: stm32h7xx_hal::stm32::Peripherals = cx.device;
        let mut syscfg = device.SYSCFG;
        let mut exti = device.EXTI;

        let mono: Mono = Systick::new(core.SYST, SYS_TICK_RATE);

        defmt::info!("Setting up Power...");
        let pwr = device.PWR.constrain();
        let pwrcfg = pwr.freeze();

        defmt::info!("Setting up RCC...");
        let rcc = device.RCC.constrain();
        let ccdr = rcc
            .sys_ck(SYS_TICK_RATE.Hz())
            .pll1_q_ck(48.MHz())
            .freeze(pwrcfg, &syscfg);

        // Configure heartbeat LED and interrupt pin
        let gpioe = device.GPIOE.split(ccdr.peripheral.GPIOE);
        let led = gpioe.pe1.into_push_pull_output();
        let mut int_pin = gpioe.pe3.into_pull_up_input();
        int_pin.make_interrupt_source(&mut syscfg);
        int_pin.trigger_on_edge(&mut exti, Edge::Rising);
        int_pin.enable_interrupt(&mut exti);

        let gpiob = device.GPIOB.split(ccdr.peripheral.GPIOB);
        let sda = gpiob.pb9.into_alternate_open_drain();
        let scl = gpiob.pb8.into_alternate_open_drain();

        let i2c = device
            .I2C1
            .i2c((scl, sda), 300.kHz(), ccdr.peripheral.I2C1, &ccdr.clocks);
        // Configure i2c

        let mut imu = unwrap!(icm20948::i2c::IcmImu::new(i2c, 0x68));

        unwrap!(imu.reset());
        defmt::debug!("IMU reset!");
        cortex_m::asm::delay(SYS_TICK_RATE/10);        // Post reset delay
        unwrap!(imu.config_gyro_rate_div(10));                // While gyro is enabled this determines the ODR (output data rate). See data sheet.
        unwrap!(imu.config_acc_rate_div(10));
        unwrap!(imu.enable_int());                            // Enable the interrupt

        heartbeat::spawn_after(Duration::<u64, 1, MONO_TICK_RATE>::from_ticks(
            MONO_TICK_RATE.into(),
        ))
            .unwrap();

        (
            Shared {},
            Local {
                led,
                state: false,
                imu,
                int_pin,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
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

        heartbeat::spawn_after(Duration::<u64, 1, MONO_TICK_RATE>::from_ticks(
            MONO_TICK_RATE.into(),
        ))
            .unwrap();
    }

    #[task(binds = EXTI3, local = [imu, int_pin])]
    fn imufn(cx: imufn::Context) {
        let acc = unwrap!(cx.local.imu.read_acc());

        defmt::debug!(
            "Accelerometer readings (g): X: {}, Y: {}, Z: {}",
            acc[0],
            acc[1],
            acc[2]
        );

        let gyr = unwrap!(cx.local.imu.read_gyro());

        defmt::debug!(
            "Gyro readings (dps): X: {}, Y: {}, Z: {}",
            gyr[0],
            gyr[1],
            gyr[2]
        ); // dps is degrees per second

        cx.local.int_pin.clear_interrupt_pending_bit();
    }
}
