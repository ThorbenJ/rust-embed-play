#![no_std]
#![no_main]

// pick a panicking behavior
// use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
// use panic_rtt_target as _;

// use cortex_m::asm;
// use cortex_m_rt::entry;
use rtic::app;
// use rtt_target::{rtt_init_print, rprintln};
use cortex_m_semihosting::hprintln;

use stm32f3xx_hal::prelude::*;
use stm32f3xx_hal::gpio as gpio; 
use systick_monotonic::{fugit::Duration, Systick};



/*
 * LD03 = PE09      LD04 = PE08         LD05 = PE10         LD06 = PE15
 * LD07 = PE11      LD08 = PE14         LD09 = PE12         LD10 = PE13
 * B1(USER) = PA0
 */
// Index into _LED matches silk screen writing
static _LED: &'static [usize] = &[usize::MAX,usize::MAX,usize::MAX, 9, 8, 10, 15, 11, 14, 12, 13];

#[app(device = stm32f3xx_hal::pac, peripherals = true, dispatchers = [TIM2, TIM3, TIM4])]
mod app {
    use super::*;
    
    #[shared]
    struct Shared {
        ope: [Option<gpio::PEx<gpio::Output<gpio::PushPull>>>; 16],
    }

    #[local]
    struct Local {
        spin: u8,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // rtt_init_print!();
        
        // Setup clocks
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();

        let mono = Systick::new(cx.core.SYST, 36_000_000);

        hprintln!("init").ok();

        let _clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(36.MHz())
            .pclk1(36.MHz())
            .freeze(&mut flash.acr);

        let mut gpioe = cx.device.GPIOE.split(&mut rcc.ahb);

        let mut ope: [Option<gpio::PEx<gpio::Output<gpio::PushPull>>>; 16] = [
            None, None, None, None, None, None, None, None, 
            Some(gpioe.pe8.downgrade().into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)), 
            Some(gpioe.pe9.downgrade().into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)), 
            Some(gpioe.pe10.downgrade().into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)), 
            Some(gpioe.pe11.downgrade().into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)), 
            Some(gpioe.pe12.downgrade().into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)), 
            Some(gpioe.pe13.downgrade().into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)),
            Some(gpioe.pe14.downgrade().into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)), 
            Some(gpioe.pe15.downgrade().into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper))
        ];
                
            
        for l in 3..11 {
            if usize::MAX == _LED[l] { continue }
            
            match ope[_LED[l]] {
                Some(ref mut led) => led.set_low().unwrap(),
                None => continue
            }
        }
            
        spin::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
        
        (
            Shared {
                ope: ope,
            },
            Local {
                spin : 0,
            },
            init::Monotonics(mono),
        )
    }
    
    #[task(shared = [ope], local = [spin])]
    fn spin(mut cx: spin::Context) {
        let pat: [usize;8 ] = [3, 5, 7, 9, 10, 8, 6, 4];
        
        if *cx.local.spin > 7 { *cx.local.spin = 0 }
        let s: usize = pat[*cx.local.spin as usize];
        hprintln!("spin {}->{}", *cx.local.spin, s).ok(); 
        
        cx.shared.ope.lock(|ope| {
            for l in pat {
                if usize::MAX == _LED[l] { continue }
                
                match ope[_LED[l]] {
                    Some(ref mut led) => if l == s { led.set_high().unwrap() } else { led.set_low().unwrap() },
                    None => continue
                }

            }
        });
        
        *cx.local.spin += 1;

        spin::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
    }
    
}



