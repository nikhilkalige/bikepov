//! Prints "Hello, World" in the OpenOCD console
//#![deny(unsafe_code)]
#![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

extern crate bsp;
extern crate cortex_m_rtfm as rtfm;
extern crate cortex_m_semihosting as semihosting;

use core::fmt::Write;

use rtfm::{app, Threshold};
use semihosting::hio;
use bsp::spi2;
use bsp::prelude::*;

pub mod setup;

app! {
    device: bsp::stm32f411,
    idle: {
        resources: [ITM, SPI1]
    }
}

fn init(p: init::Peripherals) {
    setup::spi_setup(p);
}

fn idle(_t: &mut Threshold, r: idle::Resources) -> ! {
    writeln!(hio::hstdout().unwrap(), "Hello, world!").unwrap();
    let spi = spi2::Spi::<_, bsp::stm32f411::DMA1>::new(&*r.SPI1, spi2::Role::MASTER, None, None);
    spi.enable();

    loop {
        for i in 0..255 {
            while spi.send(i).is_err() {}
            while spi.read().is_err() {}
            // writeln!(hio::hstdout().unwrap(), "Finished sending {}", i).unwrap();
        }
    }

//    writeln!(hio::hstdout().unwrap(), "Finished sending").unwrap();
    // loop {
    //     rtfm::wfi();
    // }
}
