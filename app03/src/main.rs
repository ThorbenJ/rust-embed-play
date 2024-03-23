#![no_std]
#![no_main]

// pick a panicking behavior
// use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use panic_rtt_target as _;

// use cortex_m::asm;
// use cortex_m_rt::entry;
use rtic::app;
use rtt_target::{debug_rtt_init_print, debug_rprintln};
// use stm32f3xx_hal::gpio::{Output, PushPull, PA5};
use stm32f3xx_hal::prelude::*;
use systick_monotonic::{fugit::Duration, Systick};

#[app(device = stm32f3xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
    
    }

    #[local]
    struct Local {
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Setup clocks
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();

        let mono = Systick::new(cx.core.SYST, 36_000_000);

        debug_rtt_init_print!();
        debug_rprintln!("init");

        let _clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(36.MHz())
            .pclk1(36.MHz())
            .freeze(&mut flash.acr);

        // // Setup LED
        // let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb);
        // let mut led = gpioa
        //     .pa5
        //     .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        // led.set_high().unwrap();

        // Schedule the blinking task
        // blink::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();

        (
            Shared {},
            Local { },
            init::Monotonics(mono),
        )
    }
}

// #[entry]
// fn main() -> ! {
//     asm::nop(); // To not have main optimize to abort in release mode, remove when you add code
// 
//     loop {
//         // your code goes here
//     }
// }
