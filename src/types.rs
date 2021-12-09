use embedded_ccs811::Ccs811;
use stm32f1xx_hal::gpio::gpiob;
use stm32f1xx_hal::pac::I2C1;

pub type Ccs811Type = Ccs811<
    stm32f1xx_hal::i2c::BlockingI2c<
        I2C1,
        (
            stm32f1xx_hal::gpio::gpiob::PB8<
                stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>,
            >,
            stm32f1xx_hal::gpio::gpiob::PB9<
                stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>,
            >,
        ),
    >,
    stm32f1xx_hal::gpio::gpiob::PB7<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>>,
    stm32f1xx_hal::delay::Delay,
    embedded_ccs811::mode::App,
>;

pub type IntPinType = gpiob::PB6<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::PullUp>>;
