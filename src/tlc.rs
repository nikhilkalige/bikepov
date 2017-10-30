use core::marker::Unsize;
use cortex_m::itm as Itm;
use numtoa::NumToA;

use bsp;
use bsp::stm32f411::{GPIOA, GPIOB, SYST, SPI4, DMA2, ITM};
use bsp::spi2;
use bsp::dma2;
use bsp::gpio;
use bsp::time::{Microseconds, Milliseconds};
use bsp::delay::{delay_us, delay_ms};
use bsp::dma2::Buffer;
use bsp::prelude::*;
// use hal::serial::{Write};

use static_ref::Static;

use bsp::tlc5955::TLCHardwareLayer;

// SPI
pub const LATCH: gpio::Pin<GPIOA> = gpio::Pin::new(0);
// pub const CLK: gpio::Pin<GPIOA> = gpio::Pin::new(5);
// pub const MISO: gpio::Pin<GPIOA> = gpio::Pin::new(6);
// pub const MOSI: gpio::Pin<GPIOA> = gpio::Pin::new(7);
pub const CLK: gpio::Pin<GPIOB> = gpio::Pin::new(13);
pub const MISO: gpio::Pin<GPIOA> = gpio::Pin::new(11);
pub const MOSI: gpio::Pin<GPIOA> = gpio::Pin::new(1);


pub fn log_buffer<S>(serial: &S, buffer: &[u8]) -> Result<(), S::Error>
where
    S: _embedded_hal_serial_Write<u8>
{
    for &byte in buffer {
        block!(serial.write(byte))?;
    }

    Ok(())
}

pub fn log_number<S>(serial: &S, num: u8) -> Result<(), S::Error>
where
    S: _embedded_hal_serial_Write<u8>
{
    let mut num_string = [0u8; 20];
    let start = num.numtoa(16, &mut num_string);
    log_buffer(serial, &num_string[start..])
}


pub struct TLCHardwareInterface<'a, S>
    where S: 'a +  _embedded_hal_serial_Write<u8>
{
    syst: &'a SYST,
    gpioa: &'a GPIOA,
    gpiob: &'a GPIOB,
    spi: &'a spi2::Spi<'a, SPI4, DMA2>,
    dma: &'a DMA2,
    itm: &'a ITM,
    log: &'a S,
}

impl<'a, S> TLCHardwareInterface<'a, S>
    where S: 'a +  _embedded_hal_serial_Write<u8>
{
    pub const fn new(syst: &'a SYST, gpioa: &'a GPIOA, gpiob: &'a GPIOB,
        spi: &'a spi2::Spi<'a, SPI4, DMA2>, dma: &'a DMA2, itm: &'a ITM,
        log: &'a S)-> Self {
        TLCHardwareInterface { syst, gpioa, gpiob, spi, dma, itm, log}
    }

    // fn print_number(&self, num: u8) {
    //     let mut num_string = [0u8; 20];
    //     if num < 16 {
    //         Itm::write_all(&self.itm.stim[0], b"0");
    //     }
    //     let start = num.numtoa(16, &mut num_string);
    //     Itm::write_all(&self.itm.stim[0], &num_string[start..]);
    //     Itm::write_all(&self.itm.stim[0], b" ");
    // }

    fn print_number(&self, num: u8) {
        let mut num_string = [0u8; 20];
        if num < 16 {
            self.debug("0");
        }
        let start = num.numtoa(16, &mut num_string);
        log_buffer(self.log, &num_string[start..]).is_ok();
    }
}

impl<'a, S> TLCHardwareLayer for TLCHardwareInterface<'a, S>
    where S: 'a +  _embedded_hal_serial_Write<u8>
{
    fn as_gpio(&self) {
        CLK.set_mode(self.gpiob, gpio::Mode::Output);
        MOSI.set_mode(self.gpioa, gpio::Mode::Output);
    }

    fn as_spi(&self) {
        CLK.set_mode(self.gpiob, gpio::Mode::AlternateFunction);
        MOSI.set_mode(self.gpioa, gpio::Mode::AlternateFunction);
    }

    fn latch(&self, delay: u32) {
        LATCH.set(self.gpioa, gpio::Io::High);
        LATCH.set(self.gpioa, gpio::Io::Low);
    }

    fn write_bit(&self, bit: u8) {
        if bit > 0 {
            MOSI.set(self.gpioa, gpio::Io::High);
        } else {
            MOSI.set(self.gpioa, gpio::Io::Low);
        }
        CLK.set(self.gpiob, gpio::Io::Low);
        CLK.set(self.gpiob, gpio::Io::High);
        //MOSI.set(self.gpioa, gpio::Io::Low);
        CLK.set(self.gpiob, gpio::Io::Low);
    }

    fn delay(&self, count: u16) {
        // delay_us(self.syst, Microseconds(count as u32));
        delay_ms(self.syst, Milliseconds(count as u32));
    }


    fn write<B>(&self, tx_buffer: &Buffer<B>,
        rx_buffer: &Buffer<B>)
        where B: Unsize<[u8]>
    {
        // self.spi.send_dma(buffer).unwrap();
        self.spi.rxtx_dma(tx_buffer, rx_buffer).unwrap();
    }

    fn read_write_byte(&self, byte: u8) -> u8 {
        while self.spi.send(byte).is_err() {}
        loop {
            if let Ok(byte) = self.spi.read() {
                break byte;
            }
        }
    }


    fn wait<B>(&self, buffer: &Buffer<B>)
        where B: Unsize<[u8]>
    {
//        block!(buffer.release(self.dma, dma2::DMAStream::Stream1)).unwrap();
        block!(buffer.release(self.dma)).unwrap();
    }

    fn dump_buffer(&self, buffer: &[u8]) {
        let mut counter = 0;
        for chunk in buffer.chunks(16) {
            self.print_number(counter);
            self.debug("  ");

            for small_chunk in chunk.chunks(4) {
                for byte in small_chunk {
                    self.print_number(*byte);
                }
                self.debug(" ");
            }

            self.debug("\n");
            counter += 16;
        }
    }

    fn debug<'b>(&self, data: &'b str) {
        // Itm::write_str(&self.itm.stim[0], data);
        log_buffer(self.log, data.as_bytes()).is_ok();
    }


//     void serial_dump_buf(uint8_t *buf, uint32_t len)
// {
//     uint32_t i, j;
//     char buffer[10];
//     for (i = 0; i < len; i += 16)
//     {
//         sprintf(buffer, "%02X", i);
//         Serial.print(buffer);
//         Serial.print(" ");
//         for (j = 0; j < 16 && (i + j) < len; ++j)
//         {
//             if (!(j % 4))
//                 Serial.print(" ");
//             // Serial.print(buf[i + j], HEX);
//             sprintf(buffer, "%02X", buf[i + j]);
//             Serial.print(buffer);
//         }
//         Serial.println("");
//     }
// }
}
