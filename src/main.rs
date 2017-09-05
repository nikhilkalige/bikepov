//! Prints "Hello, World" in the OpenOCD console
//#![deny(unsafe_code)]
//#![deny(warnings)]
#![feature(const_fn)]
#![feature(proc_macro)]
#![no_std]

extern crate bsp;
extern crate cortex_m_rtfm as rtfm;
extern crate cortex_m_semihosting as semihosting;

use core::fmt::Write;

use rtfm::{app, Threshold};
use semihosting::hio;
use bsp::spi2;
use bsp::dma2 as dma;
use bsp::prelude::*;

pub mod setup;


app! {
    device: bsp::stm32f411,

    resources: {
        static BUFFER: dma::Buffer<[u8; 14], dma::DMA1Stream0> = dma::Buffer::new([0; 14]);
    },

    tasks: {
        DMA2_STREAM2: {
            path: transfer_done,
            resources: [
                BUFFER, 
                DMA2],
        },
    },

    idle: {
        resources: [ITM, SPI1]
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

fn init(p: init::Peripherals, r: init::Resources) {
    writeln!(hio::hstdout().unwrap(), "Setting up peripherals").unwrap();
    setup::spi_setup(&p);
    setup::dma_setup(&p);
    
    let dma_tx = dma::Dma::new(p.DMA2, dma::DMAStream::Stream2);
    
    let spi = spi2::Spi::new(p.SPI1, spi2::Role::MASTER, None, Some(&dma_tx));
    spi.enable();

    r.BUFFER.borrow_mut().clone_from_slice(b"Hello, world!\n");
    spi.send_dma(r.BUFFER).unwrap();
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

fn transfer_done(_t: &mut Threshold, r: DMA2_STREAM2::Resources) {
    r.BUFFER.release(r.DMA2, dma::DMAStream::Stream2).unwrap();

    // rtfm::bkpt();
}