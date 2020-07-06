#![deny(unsafe_code)]
#![no_main]
#![no_std]

use rtic::app;


use panic_semihosting as _;
use cortex_m_semihosting::hprintln;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::*;

use embedded_ccs811::{prelude::*, Ccs811, MeasurementMode, SlaveAddr, AlgorithmResult, InterruptMode};

use stm32f1xx_hal::{
    prelude::*,
    delay::Delay,
    i2c::{BlockingI2c, DutyCycle, Mode},
    gpio::*,
    serial::{Config, Serial, Event as SerialEvent},
    timer::{Event as TimerEvent, Timer},
};
use asm_delay::{AsmDelay, bitrate};
use core::fmt::Write;

use heapless::Vec;
use heapless::consts::*;
use core::convert::TryInto;

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        serial_tx: stm32f1xx_hal::serial::Tx<stm32f1::stm32f100::USART1>,
        serial_rx: stm32f1xx_hal::serial::Rx<stm32f1::stm32f100::USART1>,
        serial_rx_buf: Vec<u8, U128>,
        baseline_timer: stm32f1xx_hal::timer::CountDownTimer<stm32f1::stm32f100::TIM1>,
        ccs811: Ccs811<stm32f1xx_hal::i2c::BlockingI2c<stm32f1::stm32f100::I2C1, (stm32f1xx_hal::gpio::gpiob::PB8<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>>, stm32f1xx_hal::gpio::gpiob::PB9<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>>)>, stm32f1xx_hal::gpio::gpiob::PB7<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>>, asm_delay::AsmDelay, embedded_ccs811::mode::App>,
        int_pin: gpiob::PB6<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::PullUp>>,
        algo_res: AlgorithmResult,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();

        // Freeze the configuration of all the clocks in the system and store the frozen frequencies
        // in `clocks`
        // let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(24.mhz())
            .pclk1(24.mhz())
            .freeze(&mut flash.acr);

        let mut afio = cx.device.AFIO.constrain(&mut rcc.apb2);

        let mut gpioa = cx.device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.apb2);

        // USART1
        let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let rx = gpioa.pa10;

        let mut serial = Serial::usart1(
            cx.device.USART1,
            (tx, rx),
            &mut afio.mapr,
            Config::default(),
            clocks,
            &mut rcc.apb2,
        );

        serial.listen(SerialEvent::Rxne);

        let (tx, rx) = serial.split();

        let mut int_pin = gpiob.pb6.into_pull_up_input(&mut gpiob.crl);

        int_pin.make_interrupt_source(&mut afio);
        int_pin.trigger_on_edge(&cx.device.EXTI, Edge::FALLING);
        int_pin.enable_interrupt(&cx.device.EXTI);

        let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
        let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

        let mut nwake = gpiob.pb7.into_push_pull_output(&mut gpiob.crl);
        nwake.set_high().unwrap();

        let i2c = BlockingI2c::i2c1(
            cx.device.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 100_000.hz(),
                duty_cycle: DutyCycle::Ratio2to1,
            },
            clocks,
            &mut rcc.apb1,
            1000,
            10,
            1000,
            1000,
        );

        // Configure the syst timer to trigger an update every second and enables interrupt
        let mut baseline_timer =
            Timer::tim1(cx.device.TIM1, &clocks, &mut rcc.apb2).start_count_down(1.hz());
        baseline_timer.listen(TimerEvent::Update);

        let mut delay = Delay::new(cx.core.SYST, clocks);
        let asm_delay = AsmDelay::new(bitrate::U32BitrateExt::mhz(24));

        let address = SlaveAddr::Alternative(true);
        let mut ccs811= Ccs811::new(i2c, address, nwake, asm_delay);

        ccs811.software_reset();
        delay.delay_ms(3_u16);
        let mut ccs811 = ccs811.start_application().ok().unwrap();
        delay.delay_ms(2_u8);
        let temperature_c = 25.0;
        let relative_humidity_perc = 60.0;
        ccs811
            .set_environment(temperature_c, relative_humidity_perc)
            .unwrap();
        ccs811.set_interrupt_mode(InterruptMode::OnDataReady).unwrap();
        ccs811.set_mode(MeasurementMode::LowPowerPulseHeating60s).unwrap();

        // Get hw/sw versions
        /*
        let hw_version = ccs811.hardware_version().unwrap();
        let bl_version = ccs811.firmware_bootloader_version().unwrap();
        let app_version = ccs811.firmware_application_version().unwrap();
        */

        init::LateResources {
            serial_tx: tx,
            serial_rx: rx,
            serial_rx_buf: Vec::<u8, U128>::new(),
            baseline_timer,
            ccs811,
            int_pin,
            algo_res: AlgorithmResult {
                eco2: 9999,
                etvoc: 9999,
                raw_current: 255,
                raw_voltage: 9999,
            },
        }
    }

    #[task(binds = EXTI9_5, priority = 2, resources = [int_pin, ccs811, algo_res, serial_tx])]
    fn ext_int(cx: ext_int::Context) {
        if cx.resources.int_pin.check_interrupt() {
            let default = AlgorithmResult {
                eco2: 9999,
                etvoc: 9999,
                raw_current: 255,
                raw_voltage: 9999,
            };

            *cx.resources.algo_res = cx.resources.ccs811.data().unwrap_or(default);
            writeln!(cx.resources.serial_tx, "eCO2: {}, eTVOC: {}, raw_current: {}, raw_voltage: {}\r",
                     cx.resources.algo_res.eco2,
                     cx.resources.algo_res.etvoc,
                     cx.resources.algo_res.raw_current,
                     cx.resources.algo_res.raw_voltage)
                .unwrap();
            // hprintln!("eCO2: {}, eTVOC: {}", cx.resources.algo_res.eco2, cx.resources.algo_res.etvoc).unwrap();

            // if we don't clear this bit, the ISR would trigger indefinitely
            cx.resources.int_pin.clear_interrupt_pending_bit();
        }
    }

    #[task(binds = TIM1_UP_TIM16, resources = [ccs811, serial_tx, baseline_timer])]
    fn tx_baseline(mut cx: tx_baseline::Context) {
        static mut COUNT: u16 = 0;

        if *COUNT == 600 {
            let baseline = cx.resources.ccs811.lock(|ccs811| {
                ccs811.baseline().unwrap_or([0; 2])
            });

            cx.resources.serial_tx.lock(|serial_tx| {
                writeln!(serial_tx, "baseline: [{},{}]\r", baseline[0], baseline[1]).unwrap();
            });
            *COUNT = 0;
        } else {
            *COUNT += 1;
        }

        cx.resources.baseline_timer.clear_update_interrupt_flag();
    }

    #[task(binds = USART1, resources = [ccs811, serial_tx, serial_rx, serial_rx_buf])]
    fn rx_usart(mut cx: rx_usart::Context) {
        let buf: &mut Vec<u8, U128> = cx.resources.serial_rx_buf;
        match cx.resources.serial_rx.read() {
            Ok(b) if b == 13 => {
                if !buf.is_empty() {
                    let res= match buf[0] {
                        0x62 => {
                            match buf.len() {
                                3 => cx.resources.ccs811.lock(|ccs811| {
                                    match ccs811.set_baseline(buf[1..3].try_into().unwrap()) {
                                        Ok(_) => Ok(()),
                                        Err(_) => Err("Could not set baseline!")
                                    }
                                }),
                                _ => Err("Command too short!")
                            }
                        }
                        0x65 => {
                            match buf.len() {
                                9 => {
                                    let hum = f32::from_be_bytes(buf[1..5].try_into().unwrap());
                                    let temp = f32::from_be_bytes(buf[5..9].try_into().unwrap());
                                    cx.resources.ccs811.lock(|ccs811| {
                                        match ccs811.set_environment(hum, temp) {
                                            Ok(_) => Ok(()),
                                            Err(_) => Err("Could not set environment!")
                                        }
                                    })
                                },
                                _ => Err("Command too short!")
                            }
                        }
                        0x6D => {
                            match buf.len() {
                                2 => {
                                    let mode = match buf[1] {
                                        0x30 => Ok(MeasurementMode::Idle),
                                        0x31 => Ok(MeasurementMode::ConstantPower1s),
                                        0x32 => Ok(MeasurementMode::PulseHeating10s),
                                        0x33 => Ok(MeasurementMode::LowPowerPulseHeating60s),
                                        0x34 => Ok(MeasurementMode::ConstantPower250ms),
                                        _ => Err("Invalid measurement mode!")
                                    };

                                    match mode {
                                        Ok(m) => {
                                            cx.resources.ccs811.lock(|ccs811| {
                                                match ccs811.set_mode(m) {
                                                    Ok(_) => Ok(()),
                                                    Err(_) => Err("Could not set mode!")
                                                }
                                            })
                                        }
                                        Err(e) => Err(e)
                                    }
                                },
                                _ => Err("Command too short!")
                            }
                        }
                        _ => Err("Invalid command!")
                    };
                    cx.resources.serial_tx.lock(|serial_tx| {
                        match res {
                            Ok(_) => writeln!(serial_tx, "OK\r").unwrap(),
                            Err(e) => writeln!(serial_tx, "ERR: {}\r", e).unwrap(),
                        };
                    });
                    buf.clear();
                };
            }
            Ok(b) => if let Err(_) = buf.push(b) {
                buf.clear()
            }
            _ => {}
        };
    }

    extern "C" {
        fn EXTI1();
        fn EXTI2();
        fn EXTI3();
    }
};
