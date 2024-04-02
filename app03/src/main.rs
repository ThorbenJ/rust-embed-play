#![no_std]
#![no_main]

use panic_itm as _; // logs messages over ITM; requires ITM support

use rtic::app;

use cortex_m::{/*iprint,*/ iprintln};
use stm32f3xx_hal::prelude::*;
use stm32f3xx_hal::{gpio , i2c, pac} ; 
use rtic_monotonics::Monotonic;

#[app(device = stm32f3xx_hal::pac, peripherals = true, dispatchers = [TIM2, TIM3, TIM4])]
mod app {
    use super::*;
    use rtic_monotonics::systick::*;
    
    // pub needed for #[app... ]
    #[derive(PartialEq, Copy, Clone)]
    pub enum Demo {
        Spin,
        Compass,
        Accelero,
    }
    
    impl Demo {
        fn next(&self) -> Self {
            use Demo::*;
            match *self {
                Spin => Compass,
                Compass => Accelero,
                Accelero => Spin,
            }
        }
    }
    
    #[shared]
    struct Shared {
        itm: cortex_m::peripheral::ITM,
        led: [gpio::PEx<gpio::Output<gpio::PushPull>>; 8],
        i2c1:  &'static pac::i2c1::RegisterBlock,
        demo: Demo,
        submode: usize,
    }

    #[local]
    struct Local {
        button: gpio::PA0<gpio::Input>,
        button_time: fugit::Instant<u32, 1, 1000>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut exti = cx.device.EXTI;
        let mut itm = cx.core.ITM;

        //--------- Clocks ------------
        let montok = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 36_000_000, montok);

        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(36.MHz())
            .pclk1(36.MHz())
            .freeze(&mut flash.acr);
        //Do not use itm stuff (eg iprintln) before clocks are set (not sure what panic_itm does before this point)
        
        iprintln!(&mut itm.stim[0] , "init start");
        
        //------------ User button init ----------------
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb);
        
        let mut button: gpio::PA0<gpio::Input> = gpioa.pa0.into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);
        
        let mut sys_cfg = cx.device.SYSCFG.constrain(&mut rcc.apb2);
        sys_cfg.select_exti_interrupt_source(&button);
        button.trigger_on_edge(&mut exti, gpio::Edge::RisingFalling);
        button.enable_interrupt(&mut exti);
        _ = button.interrupt();
        
        //-------- LED init -----------
        let mut gpioe = cx.device.GPIOE.split(&mut rcc.ahb);

        let mut led: [gpio::PEx<gpio::Output<gpio::PushPull>>; 8] = [
            gpioe.pe9.downgrade().into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper), 
            gpioe.pe10.downgrade().into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper), 
            gpioe.pe11.downgrade().into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper), 
            gpioe.pe12.downgrade().into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper), 
            gpioe.pe13.downgrade().into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper), 
            gpioe.pe14.downgrade().into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper),
            gpioe.pe15.downgrade().into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper), 
            gpioe.pe8.downgrade().into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)
        ];
                
            
        for l in 0..8 {
            led[l].set_low().unwrap();
        }
        
        //------------- Compass -------------
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb);
        let mut scl = gpiob.pb6.into_af_open_drain(
            &mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl
        );
        let mut sda = gpiob.pb7.into_af_open_drain(
            &mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl
        );
        scl.internal_pull_up(&mut gpiob.pupdr, true);
        sda.internal_pull_up(&mut gpiob.pupdr, true);
        _ = i2c::I2c::new(
            cx.device.I2C1, (scl, sda), 400.kHz().try_into().unwrap(), clocks, &mut rcc.apb1,
        );
        
        let i2c1: &'static pac::i2c1::RegisterBlock = unsafe {&*(pac::I2C1::ptr() ) };
        
        compass::init_compass(&i2c1);
        
        // let offset = compass::get_mag_offset(&i2c1);
        // iprintln!(&mut itm.stim[0], "Compass (magnetic) offsets {}, {}, {}", offset.0, offset.1, offset.2);
        
        
        //------------- Accelo -------------
        
        compass::init_accelo(i2c1);
        
        //------------ Complete -------------
        demo_select::spawn().ok();
        
        iprintln!(&mut itm.stim[0] , "init end");
        (
            Shared {
                itm,
                led,
                i2c1,
                demo: Demo::Spin,
                submode: 0,
            },
            Local {
                button_time: Systick::now(),
                button,
            }
        )
    }
    
    #[task(shared = [demo, led])]
    async fn demo_select(mut cx: demo_select::Context) {
        cx.shared.led.lock(|led| {
            for l in 0..8 {
                led[l].set_low().unwrap()
            }
        });
        
        cx.shared.demo.lock(|dm| {
            use Demo::*;
            match dm {
                Spin => spin_demo::spawn().ok(),
                Compass => compass_demo::spawn().ok(),
                Accelero => accelo_demo::spawn().ok(),
            };
        });
    }

    #[task(binds = EXTI0, shared = [itm, submode, demo], local = [button, button_time])]
    fn on_exti(mut cx: on_exti::Context) {
        //Can't use cx inside the closure
        let mut bt: Option<fugit::Instant<u32, 1, 1000>> = Some(*cx.local.button_time);
        let button = cx.local.button;
        
         
        button.clear_interrupt();
        let now = Systick::now();
        
        if button.is_high().unwrap() {
            bt = Some(now);
        } else {

            if (now - bt.unwrap()).to_millis() > 500 {
                cx.shared.itm.lock(|itm| { iprintln!(&mut itm.stim[0] ,"Long button press") });
                cx.shared.demo.lock(|dm| { *dm = dm.next(); });
                
                demo_select::spawn().ok();
                
            } else {
                cx.shared.itm.lock(|itm| { iprintln!(&mut itm.stim[0], "Short button press") });
                cx.shared.submode.lock(|sm| { *sm += 1; })
            }
        }
    
        *cx.local.button_time = bt.unwrap();
 
    }
        
    #[task(shared = [itm, led, i2c1, demo])]
    async fn compass_demo(mut cx: compass_demo::Context) {
        let mut res = (cx.shared.itm, cx.shared.led);
        
        let mut l:usize = 0;
        while Demo::Compass == cx.shared.demo.lock(|d| *d) {
            
            let direction = cx.shared.i2c1.lock(|i2c1| {
                compass::get_compass_xy_deg(i2c1)
            });
            
            // let (direction, rad, xyz) = cx.shared.i2c1.lock(|i2c1| {
            //     (compass::get_compass_xy_deg(i2c1), compass::get_compass_xy_rad(i2c1), compass::get_compass_coord(i2c1))
            // });
            
            res.lock(|itm, led| {
                led[l].set_low().unwrap();
                // +23 (45/2) segment boarder between led, not on led
                // /45 to get 8 segments for 8 leds
                l = (((direction+23)%360)/45) as usize;
                iprintln!(&mut itm.stim[0], "Compass {} -> {}", direction, l);
                // iprintln!(&mut itm.stim[0], "Compass {} -> {} ({},{},{} -> {})", direction, l, xyz.0, xyz.1, xyz.2, rad);
                led[l].set_high().unwrap();
            });

            Systick::delay(333.millis()).await;
        }
    }
    
    #[task(shared = [itm, led, i2c1, demo])]
    async fn accelo_demo(mut cx: accelo_demo::Context) {
        let mut res = (cx.shared.itm, cx.shared.led);
        let mut led_off: [Option<fugit::Instant<u32, 1, 1000>>;8] = [None; 8];
        
        let mut l:usize = 0;
        while Demo::Accelero == cx.shared.demo.lock(|d| *d) {
            
            let now = Systick::now();
            for i in 0..8 {
                if None != led_off[i] && led_off[i].unwrap() < now {
                    res.lock(|_, led| {
                        led[i].set_low().unwrap();
                    });
                }
            }
            
            let (x, y, d) = cx.shared.i2c1.lock(|i2c1| { compass::get_accelo_xy_and_deg(i2c1) });
            
            if x.abs() > 10000 || y.abs() > 10000 {
                
                res.lock(|itm, led| {
                    // +23 (45/2) segment boarder between led, not on led
                    // /45 to get 8 segments for 8 leds
                    l = (((d+23)%360)/45) as usize;
                    iprintln!(&mut itm.stim[0], "Accelerometer ({}, {}) -> {}", x,y,d);
                    led[l].set_high().unwrap();
                    led_off[l] = Some(now + 1111.millis());
                });
            }

            Systick::delay(222.millis()).await;
        }
    }
    
    #[task(shared = [itm, led, demo, submode])]
    async fn spin_demo(mut cx: spin_demo::Context) {

        let mut s:usize = 0;
        while Demo::Spin == cx.shared.demo.lock(|d| *d) {

            let m: usize = cx.shared.submode.lock(|submode| { *submode %= 16; return *submode });
            cx.shared.itm.lock(|itm| iprintln!(&mut itm.stim[0] ,"Spin mode {}", m));
            
            // let n: usize = (s+m)%8;
            let b: usize = (s+16-m)%8;
            
            cx.shared.led.lock(|led| {
                if m/8 > 0 {
                    //anti-clockwise
                    led[7-b].set_low().unwrap();
                    led[7-s].set_high().unwrap();
                } else {
                    //clockwise
                    led[b].set_low().unwrap();
                    led[s].set_high().unwrap();
                }
            });
        
            s = (s+1)%8;
            Systick::delay(333.millis()).await;
        }
    }
    
}

mod compass {
    use super::*;
    
    use core::f32::consts::PI;
    use fast_math;
    
    const MAGNETOMETER: u16 = 0x3C;
    const ACCELEROMETER: u16 = 0x32;
    
    const M_WHO_AM_I_BANK: u8 = 0x4F;
    const M_CFG_REG_A_BANK: u8 = 0x60;
    const M_OUTX_L_REG_BANK: u8 = 0x68;
    const M_OFFSET_X_REG_L_BANK: u8 = 0x45;
    
    const A_WHO_AM_I_BANK: u8 = 0x0F;
    const A_CTRL_REG1_BANK: u8 = 0x20;
    const A_CTRL_REG2_BANK: u8 = 0x21;
    const A_OUT_X_L_BANK: u8 = 0x28;
    
    
    const M_WHO_AM_I_TOKEN: u8 = 0x40; //0b0100_0000;
    const A_WHO_AM_I_TOKEN: u8 = 0x33; //0b0011_0011
    
    //============= Compass =======================
    
    pub fn init_compass(i2c1: &pac::i2c1::RegisterBlock) {
        let mut _bytes: [u8;3] = [0;3];
        
        _bytes[0] = read_byte(i2c1, MAGNETOMETER, M_WHO_AM_I_BANK );
        assert_eq!(M_WHO_AM_I_TOKEN, _bytes[0]);
        
        // Inistialised using hints from sect. 4.1.5 (Self-test proceedure)
        write_bytes(i2c1, MAGNETOMETER, M_CFG_REG_A_BANK, &[0x80, 0x00, 0x10]);
        read_bytes(i2c1, MAGNETOMETER, M_CFG_REG_A_BANK, &mut _bytes);
        assert_eq!(0x80, _bytes[0]);
        assert_eq!(0x00, _bytes[1]);
        assert_eq!(0x10, _bytes[2]);
    }
    
    pub fn get_mag_offset(i2c1: &pac::i2c1::RegisterBlock) -> (i16, i16, i16) {
        let mut data: [u8;6] = [0;6];
        
        read_bytes(i2c1, MAGNETOMETER, M_OFFSET_X_REG_L_BANK, &mut data);

        bytes_to_coord(&data)
    }
    
    // In Rust 2024 you can't &mut a static. The current warning recommends using addr_of_mut!()
    // However that is in std, and this is a no_std application =/
    #[allow(static_mut_refs)]
    pub fn get_compass_coord(i2c1: &pac::i2c1::RegisterBlock) -> (i16, i16, i16) {
        let mut data: [u8;6] = [0;6];
        
        // Expect to be on a single core cortex-m4
        static mut MINMAX: [i16; 6] = [i16::MAX, i16::MIN, i16::MAX, i16::MIN, i16::MAX, i16::MIN];
        
        read_bytes(i2c1, MAGNETOMETER, M_OUTX_L_REG_BANK, &mut data);

        let mut xyz = bytes_to_coord(&data);
        
        unsafe {
            xyz = auto_offset_coord(xyz, &mut MINMAX);
        }
        
        xyz
    }
    
    pub fn get_compass_xy(i2c1: &pac::i2c1::RegisterBlock) -> (i16,i16) {
        xyz_to_xy( get_compass_coord(i2c1) )
    }
    
    pub fn get_compass_xy_rad(i2c1: &pac::i2c1::RegisterBlock) -> f32 {
        xy_to_rad( get_compass_xy(i2c1) )
    }
    
    pub fn get_compass_xy_deg(i2c1: &pac::i2c1::RegisterBlock) -> u16 {
        rad_to_deg( get_compass_xy_rad(i2c1) )
    }
    
    pub fn get_compass_xy_and_deg(i2c1: &pac::i2c1::RegisterBlock) -> (i16, i16, u16) {
        let xy = get_compass_xy(i2c1);
        ( xy.0, xy.1, rad_to_deg(xy_to_rad(xy) ) )
    }
    
    //================== Accelo' ========================
    
    pub fn init_accelo(i2c1: &pac::i2c1::RegisterBlock) {
        let mut _bytes: [u8;3] = [0;3];
        
        _bytes[0] = read_byte(i2c1, ACCELEROMETER, A_WHO_AM_I_BANK );
        assert_eq!(A_WHO_AM_I_TOKEN, _bytes[0]);
        
        write_bytes(i2c1, ACCELEROMETER, A_CTRL_REG2_BANK, &[0x00, 0x00, 0x80]);
        read_bytes(i2c1, ACCELEROMETER, A_CTRL_REG2_BANK, &mut _bytes);
        assert_eq!(0x00, _bytes[0]);
        assert_eq!(0x00, _bytes[1]);
        assert_eq!(0x80, _bytes[2]);
        
        //Self-test plan Sect. 4.2.4 uses this bank sequence to initialise
        write_byte(i2c1, ACCELEROMETER, A_CTRL_REG1_BANK, 0x57);
        _bytes[0] = read_byte(i2c1, ACCELEROMETER, A_CTRL_REG1_BANK);
        assert_eq!(0x57, _bytes[0]);
    }

    #[allow(static_mut_refs)]
    pub fn get_accelo_coord(i2c1: &pac::i2c1::RegisterBlock) -> (i16, i16, i16) {
        let mut data: [u8;6] = [0;6];
        
        read_bytes(i2c1, ACCELEROMETER, A_OUT_X_L_BANK, &mut data);

        bytes_to_coord(&data)
    }
    
    pub fn get_accelo_xy(i2c1: &pac::i2c1::RegisterBlock) -> (i16,i16) {
        xyz_to_xy( get_accelo_coord(i2c1) )
    }
    
    pub fn get_accelo_xy_rad(i2c1: &pac::i2c1::RegisterBlock) -> f32 {
        xy_to_rad( get_accelo_xy(i2c1) )
    }
    
    pub fn get_accelo_xy_deg(i2c1: &pac::i2c1::RegisterBlock) -> u16 {
        rad_to_deg( get_accelo_xy_rad(i2c1) )
    }
    
    pub fn get_accelo_xy_and_deg(i2c1: &pac::i2c1::RegisterBlock) -> (i16, i16, u16) {
        let xy = get_accelo_xy(i2c1);
        ( xy.0, xy.1, rad_to_deg(xy_to_rad(xy) ) )
    }
    
    //================== Util ==========================

    pub fn xyz_to_xy(xyz: (i16, i16, i16)) -> (i16,i16) {
        (xyz.0,xyz.1)
    }
    
    pub fn xy_to_rad(xy: (i16, i16)) -> f32 {
        fast_math::atan2(xy.1 as f32, xy.0 as f32)
    }
    
    pub fn rad_to_deg(rad: f32) -> u16 {
        // Rad from horizontal axis, -90 to make deg from verital axis (i.e. 0=north)
        ((rad * 180_f32 / PI) + 270_f32) as u16 % 360_u16
    }
    
    // Track min/max history and apply a center offset
    #[allow(dead_code)]
    fn auto_offset_coord(xyz: (i16,i16,i16), minmax: &mut [i16;6]) -> (i16,i16,i16) {
        if xyz.0 < minmax[0] { minmax[0] = xyz.0 };
        if xyz.0 > minmax[1] { minmax[1] = xyz.0 };
        if xyz.1 < minmax[2] { minmax[2] = xyz.1 };
        if xyz.1 > minmax[3] { minmax[3] = xyz.1 };
        if xyz.2 < minmax[4] { minmax[4] = xyz.2 };
        if xyz.2 > minmax[5] { minmax[5] = xyz.2 };
        
        let range: (i16,i16,i16) = (
            if minmax[1] - minmax[0] > 0 { minmax[1] - minmax[0] } else { 2 },
            if minmax[3] - minmax[2] > 0 { minmax[3] - minmax[2] } else { 2 },
            if minmax[5] - minmax[4] > 0 { minmax[5] - minmax[4] } else { 2 }
        );
        
        (
            (xyz.0-(minmax[0]+range.0/2)) ,
            (xyz.1-(minmax[2]+range.1/2)) ,
            (xyz.2-(minmax[4]+range.2/2)) 
        )
    }
    
    // Track min/max history and normalise that range to -256/+256
    #[allow(dead_code)]
    fn auto_scale_coord(xyz: (i16,i16,i16), minmax: &mut [i16;6]) -> (i16,i16,i16) {
        if xyz.0 < minmax[0] { minmax[0] = xyz.0 };
        if xyz.0 > minmax[1] { minmax[1] = xyz.0 };
        if xyz.1 < minmax[2] { minmax[2] = xyz.1 };
        if xyz.1 > minmax[3] { minmax[3] = xyz.1 };
        if xyz.2 < minmax[4] { minmax[4] = xyz.2 };
        if xyz.2 > minmax[5] { minmax[5] = xyz.2 };
        
        let range: (i16,i16,i16) = (
            if minmax[1] - minmax[0] > 0 { minmax[1] - minmax[0] } else { 2 },
            if minmax[3] - minmax[2] > 0 { minmax[3] - minmax[2] } else { 2 },
            if minmax[5] - minmax[4] > 0 { minmax[5] - minmax[4] } else { 2 }
        );
        
        (
            ( ((xyz.0-(minmax[0]+range.0/2)) as f32 / range.0 as f32) * 512_f32) as i16,
            ( ((xyz.1-(minmax[2]+range.0/2)) as f32 / range.1 as f32) * 512_f32) as i16,
            ( ((xyz.2-(minmax[4]+range.0/2)) as f32 / range.2 as f32) * 512_f32) as i16
        )
    }
    
    // Return Result?
    fn read_byte(i2c1: &pac::i2c1::RegisterBlock, addr: u16, bank: u8) -> u8 {
        let mut data: [u8;1] = [0];
        read_bytes(i2c1, addr, bank, &mut data);
        data[0]
    }
    
    //Return Result?
    fn read_bytes(i2c1: &pac::i2c1::RegisterBlock, addr: u16, bank: u8, data: &mut [u8]) {
       
        i2c1.cr2.write(|w| {
            w.start().set_bit();
            w.sadd().bits(addr);
            w.rd_wrn().clear_bit();
            w.nbytes().bits(1);
            w.autoend().clear_bit()
        });
        while i2c1.isr.read().txis().bit_is_clear() {};

        // MSB = 1 for bank auto increment
        i2c1.txdr.write(|w| w.txdata().bits(bank|0x80));
        while i2c1.isr.read().tc().bit_is_clear() {};
    
        i2c1.cr2.modify(|_, w| {
            w.start().set_bit();
            w.sadd().bits(addr);
            w.nbytes().bits(data.len() as u8);
            w.rd_wrn().set_bit();
            w.autoend().clear_bit()
        });

        for byte in data {
            while i2c1.isr.read().rxne().bit_is_clear() {}
            *byte = i2c1.rxdr.read().rxdata().bits()
        };
        
        i2c1.cr2.modify(|_, w| {
            w.stop().set_bit() 
        });
        while i2c1.isr.read().stopf().bit_is_clear() {}
        
        i2c1.icr.write(|w| w.stopcf().set_bit());
        
    }
    
    fn write_byte(i2c1: &pac::i2c1::RegisterBlock, addr: u16, bank: u8, data: u8) {
        write_bytes(i2c1, addr, bank, &[data]);
    }
    
    fn write_bytes(i2c1: &pac::i2c1::RegisterBlock, addr: u16, bank: u8, data: &[u8]) {
        
        i2c1.cr2.write(|w| {
            w.start().set_bit();
            w.sadd().bits(addr);
            w.rd_wrn().clear_bit();
            w.nbytes().bits(1_u8 + (data.len() as u8));
            w.autoend().clear_bit()
        });
        while i2c1.isr.read().txis().bit_is_clear() {}

        // MSB = 1 for bank auto increment
        i2c1.txdr.write(|w| w.txdata().bits(bank|0x80) );
        
        for byte in data {
            while i2c1.isr.read().txe().bit_is_clear() {}
            i2c1.txdr.write(|w| w.txdata().bits(*byte));
        };
        while i2c1.isr.read().tc().bit_is_clear() {};
        
        i2c1.cr2.modify(|_, w| {
            w.stop().set_bit() 
        });
        while i2c1.isr.read().stopf().bit_is_clear() {}
        
        i2c1.icr.write(|w| w.stopcf().set_bit());
    }
    
    fn bytes_to_coord(bytes: &[u8;6]) -> (i16, i16, i16) {
        let x:i16 = (((bytes[1] as u16) << 8) | bytes[0] as u16) as i16;
        let y:i16 = (((bytes[3] as u16) << 8) | bytes[2] as u16) as i16;
        let z:i16 = (((bytes[5] as u16) << 8) | bytes[4] as u16) as i16;
        
        (x,y,z)
    }
} 


