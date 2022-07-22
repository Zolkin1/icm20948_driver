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
    use stm32h7xx_hal::prelude::*;

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
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("Init");

        let core: cortex_m::Peripherals = cx.core;
        let device: stm32h7xx_hal::stm32::Peripherals = cx.device;

        let mono: Mono = Systick::new(core.SYST, SYS_TICK_RATE);


        defmt::info!("Setting up power...");
        let pwr = device.PWR.constrain();
        let pwrcfg = pwr.freeze();

        defmt::info!("Setting up RCC...");
        let rcc = device.RCC.constrain();
        let ccdr = rcc.sys_ck(SYS_TICK_RATE.Hz()).freeze(pwrcfg, &device.SYSCFG);


        let gpioe = device.GPIOE.split(ccdr.peripheral.GPIOE);

        // Configure PE1 as output.
        let led = gpioe.pe1.into_push_pull_output();

        heartbeat::spawn_after(Duration::<u64, 1, MONO_TICK_RATE>::from_ticks(MONO_TICK_RATE.into())).unwrap();

        // Setup the monotonic timer
        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                led,
                state: false,
            },
            init::Monotonics(
                mono
            ),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            rtic::export::wfi()
        }
    }

    #[task(local = [led, state])]
    fn heartbeat(cx: heartbeat::Context) {
        if *cx.local.state {
            cx.local.led.set_high();
            *cx.local.state = false;
            defmt::info!("Heartbeat: LED set high");
        } else {
            cx.local.led.set_low();
            *cx.local.state = true;
            defmt::info!("Heartbeat: LED set low");
        }

        heartbeat::spawn_after(Duration::<u64, 1, MONO_TICK_RATE>::from_ticks(MONO_TICK_RATE.into())).unwrap();
    }
}
