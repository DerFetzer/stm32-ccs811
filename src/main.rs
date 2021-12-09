#![deny(unsafe_code)]
#![no_main]
#![no_std]

mod types;

use rtic::app;

use panic_semihosting as _;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::*;

use embedded_ccs811::{
    prelude::*, AlgorithmResult, Ccs811, InterruptMode, MeasurementMode, SlaveAddr,
};

use asm_delay::{bitrate, AsmDelay};
use stm32f1xx_hal::{
    delay::Delay,
    gpio::*,
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac::TIM1,
    prelude::*,
    timer::{Event as TimerEvent, Timer},
    usb::{Peripheral, UsbBus, UsbBusType},
};

use core::convert::TryInto;
use heapless::Vec;

use heapless::String;
use ufmt::uwriteln;
use usb_device::bus;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use crate::types::{Ccs811Type, IntPinType};

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: usb_device::device::UsbDevice<'static, UsbBusType>,
        serial: SerialPort<'static, UsbBusType>,
        serial_rx_buf: Vec<u8, 128>,
        baseline_timer: stm32f1xx_hal::timer::CountDownTimer<TIM1>,
        ccs811: Ccs811Type,
        int_pin: IntPinType,
        algo_res: AlgorithmResult,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;

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
            .sysclk(48.mhz())
            .pclk1(24.mhz())
            .freeze(&mut flash.acr);

        assert!(clocks.usbclk_valid());

        let delay = Delay::new(cx.core.SYST, clocks);

        // Tuned delay frequency with oscilloscope
        let mut asm_delay = AsmDelay::new(bitrate::U32BitrateExt::mhz(77));

        let mut afio = cx.device.AFIO.constrain(&mut rcc.apb2);

        let mut gpioa = cx.device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.apb2);

        // USB Serial

        // BluePill board has a pull-up resistor on the D+ line.
        // Pull the D+ pin down to send a RESET condition to the USB bus.
        // This forced reset is needed only for development, without it host
        // will not reset your device when you upload new firmware.
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low().unwrap();
        asm_delay.delay_ms(100_u16);
        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usb = Peripheral {
            usb: cx.device.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        *USB_BUS = Some(UsbBus::new(usb));

        let serial = SerialPort::new(USB_BUS.as_ref().unwrap());

        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .build();

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

        let address = SlaveAddr::Alternative(true);
        let mut ccs811 = Ccs811::new(i2c, address, nwake, delay);

        ccs811.software_reset().unwrap_or(());
        asm_delay.delay_ms(4_u16);
        let mut ccs811 = ccs811.start_application().ok().unwrap();
        asm_delay.delay_ms(3_u8);
        ccs811
            .set_interrupt_mode(InterruptMode::OnDataReady)
            .unwrap();
        ccs811
            .set_mode(MeasurementMode::LowPowerPulseHeating60s)
            .unwrap();

        init::LateResources {
            usb_dev,
            serial,
            serial_rx_buf: Vec::<u8, 128>::new(),
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

    #[task(binds = EXTI9_5, priority = 2, resources = [int_pin, ccs811, algo_res, serial])]
    fn ext_int(mut cx: ext_int::Context) {
        if cx.resources.int_pin.check_interrupt() {
            let default = AlgorithmResult {
                eco2: 9999,
                etvoc: 9999,
                raw_current: 255,
                raw_voltage: 9999,
            };

            *cx.resources.algo_res = cx
                .resources
                .ccs811
                .lock(|ccs811| ccs811.data().unwrap_or(default));
            let mut s: String<128> = String::new();
            uwriteln!(
                s,
                "eCO2: {}, eTVOC: {}, raw_current: {}, raw_voltage: {}\r",
                cx.resources.algo_res.eco2,
                cx.resources.algo_res.etvoc,
                cx.resources.algo_res.raw_current,
                cx.resources.algo_res.raw_voltage
            )
            .unwrap();

            cx.resources.serial.lock(|serial| {
                if serial.flush().is_ok() {
                    serial.write(s.as_bytes()).unwrap();
                }
            });

            // if we don't clear this bit, the ISR would trigger indefinitely
            cx.resources.int_pin.clear_interrupt_pending_bit();
        }
    }

    #[task(binds = TIM1_UP, resources = [ccs811, serial, baseline_timer])]
    fn tx_baseline(mut cx: tx_baseline::Context) {
        static mut COUNT: u16 = 0;

        if *COUNT == 600 {
            let baseline = cx
                .resources
                .ccs811
                .lock(|ccs811| ccs811.baseline().unwrap_or([0; 2]));

            let mut s: String<64> = String::new();
            uwriteln!(s, "baseline: [{},{}]\r", baseline[0], baseline[1]).unwrap();

            cx.resources.serial.lock(|serial| {
                if serial.flush().is_ok() {
                    serial.write(s.as_bytes()).unwrap();
                }
            });
            *COUNT = 0;
        } else {
            *COUNT += 1;
        }

        cx.resources.baseline_timer.clear_update_interrupt_flag();
    }

    #[task(binds = USB_HP_CAN_TX, priority = 3, resources = [usb_dev, serial, serial_rx_buf, ccs811])]
    fn usb_tx(mut cx: usb_tx::Context) {
        usb_poll(
            &mut cx.resources.usb_dev,
            &mut cx.resources.serial,
            &mut cx.resources.serial_rx_buf,
            cx.resources.ccs811,
        );
    }

    #[task(binds = USB_LP_CAN_RX0, priority = 3, resources = [usb_dev, serial, serial_rx_buf, ccs811])]
    fn usb_rx0(mut cx: usb_rx0::Context) {
        usb_poll(
            &mut cx.resources.usb_dev,
            &mut cx.resources.serial,
            &mut cx.resources.serial_rx_buf,
            cx.resources.ccs811,
        );
    }

    extern "C" {
        fn EXTI1();
        fn EXTI2();
        fn EXTI3();
    }
};

fn usb_poll<B: bus::UsbBus>(
    usb_dev: &mut UsbDevice<'static, B>,
    serial: &mut SerialPort<'static, B>,
    buf: &mut Vec<u8, 128>,
    ccs811: &mut Ccs811Type,
) {
    if !usb_dev.poll(&mut [serial]) {
        return;
    }

    let mut read_buf = [0u8; 64];

    match serial.read(&mut read_buf) {
        Ok(count) if count > 0 => {
            for c in read_buf[0..count].iter() {
                match c {
                    13 => {
                        if !buf.is_empty() {
                            let res = match buf[0] {
                                0x62 => match buf.len() {
                                    3 => match ccs811.set_baseline(buf[1..3].try_into().unwrap()) {
                                        Ok(_) => Ok(()),
                                        Err(_) => Err("Could not set baseline!"),
                                    },
                                    _ => Err("Command too short!"),
                                },
                                0x65 => match buf.len() {
                                    9 => {
                                        let hum = f32::from_be_bytes(buf[1..5].try_into().unwrap());
                                        let temp =
                                            f32::from_be_bytes(buf[5..9].try_into().unwrap());
                                        match ccs811.set_environment(hum, temp) {
                                            Ok(_) => Ok(()),
                                            Err(_) => Err("Could not set environment!"),
                                        }
                                    }
                                    _ => Err("Command too short!"),
                                },
                                0x6D => match buf.len() {
                                    2 => {
                                        let mode = match buf[1] {
                                            0x30 => Ok(MeasurementMode::Idle),
                                            0x31 => Ok(MeasurementMode::ConstantPower1s),
                                            0x32 => Ok(MeasurementMode::PulseHeating10s),
                                            0x33 => Ok(MeasurementMode::LowPowerPulseHeating60s),
                                            0x34 => Ok(MeasurementMode::ConstantPower250ms),
                                            _ => Err("Invalid measurement mode!"),
                                        };

                                        match mode {
                                            Ok(m) => match ccs811.set_mode(m) {
                                                Ok(_) => Ok(()),
                                                Err(_) => Err("Could not set mode!"),
                                            },
                                            Err(e) => Err(e),
                                        }
                                    }
                                    _ => Err("Command too short!"),
                                },
                                _ => Err("Invalid command!"),
                            };
                            match res {
                                Ok(_) => serial.write("OK\r\n".as_bytes()).unwrap(),
                                Err(e) => {
                                    let mut s: String<64> = String::new();
                                    uwriteln!(s, "ERR: {}\r", e).unwrap();
                                    serial.write(s.as_bytes()).unwrap()
                                }
                            };
                            buf.clear();
                        };
                    }
                    b => {
                        if buf.push(*b).is_err() {
                            buf.clear()
                        }
                    }
                }
            }
        }
        _ => (),
    }
}
