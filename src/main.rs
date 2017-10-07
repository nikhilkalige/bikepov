//! Prints "Hello, World" in the OpenOCD console
//#![deny(unsafe_code)]
//#![deny(warnings)]
#![feature(unsize)]
#![feature(const_unsafe_cell_new)]
#![feature(const_fn)]
#![feature(proc_macro)]
#![no_std]

extern crate bsp;
extern crate cortex_m_rtfm as rtfm;
extern crate cortex_m_semihosting as semihosting;
extern crate numtoa;
extern crate embedded_hal as hal;
#[macro_use(iprint, iprintln)]
extern crate cortex_m;
#[macro_use]
extern crate nb;
extern crate static_ref;
use core::fmt::Write;

use rtfm::{app, Threshold};
use semihosting::hio;
use bsp::spi2;
use bsp::serial::Serial;
use bsp::dma2 as dma;
use bsp::timer::Channel;
use bsp::pwm2::Pwm;
use bsp::tlc5955::TLC5955;
use bsp::time::Hertz;
use bsp::prelude::*;
// use hal::prelude::*;

pub mod setup;
pub mod tlc;

app! {
    device: bsp::stm32f411,

    resources: {
        static BUFFER: dma::Buffer<[u8; 96]> = dma::Buffer::new([0; 96], dma::DMAStream::Stream1);
        static RX_BUFFER: dma::Buffer<[u8; 96]> = dma::Buffer::new([0; 96], dma::DMAStream::Stream4);
    },

    tasks: {
        DMA2_STREAM1: {
            path: transfer_done,
            resources: [
                BUFFER,
                RX_BUFFER,
                DMA2],
        },
    },

    idle: {
        resources: [ITM, SPI4]
    }
}

// app! {
//     device: bsp::stm32f411,

//     tasks: {
//         DMA1_STREAM3: {
//             path: transfer_done,
//             resources: [DMA1],
//         },
//     },

//     idle: {
//         resources: [ITM, SPI1]
//     }
// }
const FREQUENCY: Hertz = Hertz(1_000);

fn init(p: init::Peripherals, r: init::Resources) {
    // writeln!(hio::hstdout().unwrap(), "Setting up peripherals").unwrap();
    // iprintln!(&p.ITM.stim[0], "Hello");
    setup::spi_setup(&p);
    setup::dma_setup(&p);
    setup::pwm_setup(&p);
    setup::serial_setup(&p);

    let serial = Serial(p.USART2);
    serial.init(Hertz(115200).invert());

    serial.write("Setup in progress\n").is_ok();

    let dma_tx = dma::Dma::new(p.DMA2, dma::DMAStream::Stream1);
    let dma_rx = dma::Dma::new(p.DMA2, dma::DMAStream::Stream4);

    let spi = spi2::Spi::new(p.SPI4, spi2::Role::MASTER, Some(&dma_rx), Some(&dma_tx));
    spi.enable();
    // dma_tx.enable();

    //let timer = timer::Timer::new(p.TIM4);
    let pwm = Pwm(p.TIM1);
    pwm.init(FREQUENCY.invert());
    let duty = pwm.get_max_duty() / 2;

    const CHANNELS: [Channel; 1] = [Channel::_1];
    for c in &CHANNELS {
        pwm.set_duty(*c, duty);
        // pwm.enable(*c);
    }

//    r.BUFFER.borrow_mut().clone_from_slice(b"Hello, world!\n");
//    spi.send_dma(r.BUFFER).unwrap();
    // *r.BUFFER.borrow_mut().last_mut().unwrap() = 0;
    writeln!(hio::hstdout().unwrap(), "Filling").unwrap();
    // fill_control_data(&mut *r.BUFFER.borrow_mut());
    writeln!(hio::hstdout().unwrap(), "Filling Done").unwrap();
    let mut tlc = TLC5955::new(1);
    let driver = tlc::TLCHardwareInterface::new(p.SYST, p.GPIOA, p.GPIOB, &spi,
        p.DMA2, p.ITM, &serial);
    tlc.setup(r.BUFFER, r.RX_BUFFER, &driver);
}

fn idle(_t: &mut Threshold, _r: idle::Resources) -> ! {
    // let spi = spi2::Spi::<_, bsp::stm32f411::DMA1>::new(&*r.SPI1, spi2::Role::MASTER, None, None);
    // spi.enable();

    // loop {
        // for i in 0..255 {
            // while spi.send(i).is_err() {}
            // while spi.read().is_err() {}
            // writeln!(hio::hstdout().unwrap(), "Finished sending {}", i).unwrap();
        // }
    // }

    writeln!(hio::hstdout().unwrap(), "Finished sending").unwrap();
    loop {
        rtfm::wfi();
    }
}

fn transfer_done(_t: &mut Threshold, r: DMA2_STREAM1::Resources) {
    // let dma = dma::Dma::new(&**r.DMA2, dma::DMAStream::Stream0);
    r.BUFFER.release(&**r.DMA2).unwrap();
    r.RX_BUFFER.release(&**r.DMA2).unwrap();
    writeln!(hio::hstdout().unwrap(), "Buffer released").unwrap();
    // rtfm::bkpt();
}

fn fill_control_data(buffer: &mut[u8]) {
    let mut pos = 0;
    for index in 0..48 {
        pos = add_bits(buffer, 7 as u32, 7, pos);
    }

    for index in 0..3 {
        pos = add_bits(buffer, 3 as u32, 3, pos);
    }

    for index in 0..3 {
        pos = add_bits(buffer, 3 as u32, 3, pos);
    }
    pos = add_bits(buffer, 0x0A as u32, 5, pos);
    pos = 760;
    add_bits(buffer, 0x96 as u32, 5, pos);
}

fn add_bits(buffer: &mut[u8], mut val: u32, mut size: u32, mut pos: usize) -> usize{
    while (size > 0) {
        if (val & 1) > 0 {
            buffer[pos >> 3] |= 1 << (pos & 7);
        }
        val >>= 1;
        pos += 1;
        size -= 1;
    }
    return pos;
}
