//#![deny(unsafe_code)]
//#![deny(warnings)]
#![feature(unsize)]
#![feature(const_unsafe_cell_new)]
#![feature(const_fn)]
#![feature(proc_macro)]
#![no_std]

extern crate bsp;
#[macro_use(iprint, iprintln)]
extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
extern crate cortex_m_semihosting as semihosting;
extern crate embedded_hal as hal;
extern crate logger;
#[macro_use]
extern crate nb;
extern crate static_ref;
extern crate tlc5955;

use core::marker::Unsize;

use bsp::stm32f411::{GPIOA, GPIOB};
use rtfm::{app, Threshold};

use bsp::usart::{Tx, Usart};
use bsp::pwm::PwmExt;
use bsp::rcc::{ClockSource, RccExt};
use bsp::gpio::{Io, Mode, Pupd, Speed};
use bsp::spi::{DataSize, Role, Spi, NSS};
use bsp::time::TimeExt;
use bsp::gpio::{Gpio, GpioExt};
use bsp::dma::{D2S1, D2S4};

use hal::spi::{Mode as SpiMode, Phase, Polarity};
use hal::serial::Write;
use hal::digital::OutputPin;

use tlc5955::{SpiWriteBit, Tlc5955};
use logger::Logger;

pub struct SerialLogger(pub Tx);

impl Logger for SerialLogger {
    type Error = u8;

    fn log(&mut self, byte: u8) {
        let SerialLogger(ref mut tx) = *self;
        block!(tx.write(byte)).ok().unwrap();
    }
}

struct BitWrite;

// No clean way of doing this.
// Choose a better chip next time :(
impl SpiWriteBit for BitWrite {
    fn write_bit(&self, bit: u8) {
        unsafe {
            Gpio::set_mode(&(*GPIOB::ptr()), 13, Mode::Output);
            Gpio::set_mode(&(*GPIOA::ptr()), 1, Mode::Output);

            if bit > 0 {
                Gpio::set(&(*GPIOA::ptr()), 1, Io::High);
            } else {
                Gpio::set(&(*GPIOA::ptr()), 1, Io::Low);
            }

            Gpio::set(&(*GPIOB::ptr()), 13, Io::Low);
            Gpio::set(&(*GPIOB::ptr()), 13, Io::High);
            Gpio::set(&(*GPIOB::ptr()), 13, Io::Low);

            Gpio::set_mode(&(*GPIOB::ptr()), 13, Mode::AlternateFunction);
            Gpio::set_mode(&(*GPIOA::ptr()), 1, Mode::AlternateFunction);

            Gpio::alternate_function(&(*GPIOB::ptr()), 13, 6);
            Gpio::alternate_function(&(*GPIOA::ptr()), 1, 5);
        }
    }
}

app! {
    device: bsp::stm32f411,

    // resources: {
    //     static RX: Tlc5955;
    // },
}

fn init(p: init::Peripherals)  {
    let mut rcc = p.device.RCC.split();

    //rcc.cfgr.sysclk(64.mhz()).pclk1(30.mhz());
    let clocks = rcc.cfgr.freeze(ClockSource::Hsi);

    let gpiob = p.device.GPIOB.split(&mut rcc.enr);
    let gpioc = p.device.GPIOC.split(&mut rcc.enr);
    let gpioa = p.device.GPIOA.split(&mut rcc.enr);

    // Serial Tx, Rx
    let pa2 = gpioa.pa2.as_alt_function();
    let pa3 = gpioa.pa3.as_alt_function();

    // Leds
    let mut pc1 = gpioc.pc1.as_output();
    let mut pc2 = gpioc.pc2.as_output();
    pc1.set_high();
    pc2.set_high();

    let (tx, _rx) = Usart::new(
        p.device.USART2,
        (pa2, pa3),
        115_200.bps(),
        clocks,
        &mut rcc.enr,
    ).split();

    let mut logger = SerialLogger(tx);
    logger.log_string("TLC5955 BikePov v0.1\n");

    // Spi Clk, Miso, Mosi
    let pb13 = gpiob.pb13.as_alt_function();
    let pa11 = gpioa.pa11.as_alt_function();
    let pa1 = gpioa.pa1.as_alt_function();
    pa1.set_pupd(Pupd::PullUp);
    pb13.set_pupd(Pupd::PullUp);

    let spi = Spi::new(p.device.SPI4, (pb13, pa1, pa11), &mut rcc.enr);
    spi.set_frequency(clocks, 250_000.hz());
    let spi_mode = SpiMode {
        phase: Phase::CaptureOnFirstTransition,
        polarity: Polarity::IdleLow,
    };
    spi.set_role(Role::MASTER);
    spi.set_mode(spi_mode);
    spi.data_size(DataSize::_8BIT);
    spi.msb_first(true);
    spi.crc_calculation(false);
    spi.nss(NSS::Soft);
    spi.enable();

    // Latch
    let pa0 = gpioa.pa0.as_output();

    // Pwm
    let pa8 = gpioa.pa8.as_alt_function();
    pa8.set_speed(Speed::Fast);

    let pwm = PwmExt::pwm(p.device.TIM1, pa8, 50_000.hz(), clocks, &mut rcc.enr);

    let bit_writer = BitWrite {};
    // let tlc: Tlc5955<[u8; 1], _, _, _, _> =
    let tlc = Tlc5955::new(spi, bit_writer, pa0, pwm, &mut logger, 4).unwrap();

    //init::LateResources { RX: tlc }
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}
