use bsp::stm32f411;
use bsp::spi2;
use bsp::serial;
use bsp::dma2;
use bsp::gpio::{Io, Mode, Pupd, Pin};

pub fn dma_setup(p: &stm32f411::Peripherals) {
    p.RCC.ahb1enr.modify(|_, w| w.dma2en().set_bit());

    let dma_rx = dma2::Dma::new(p.DMA2, dma2::DMAStream::Stream4);
    let dma_tx = dma2::Dma::new(p.DMA2, dma2::DMAStream::Stream1);

    dma_rx.channel(dma2::Channel::CHANNEL_4);
    dma_rx.priority(dma2::Priority::MEDIUM);
    dma_rx.memdata_alignment(dma2::DataSize::BYTE);
    dma_rx.periphdata_alignment(dma2::DataSize::BYTE);
    dma_rx.memory_increment(true);
    dma_rx.direction(dma2::Direction::PERIPH_TO_MEMORY);
    dma_rx.reg.s0cr.modify(|_, w| w.tcie().set_bit());

    dma_tx.channel(dma2::Channel::CHANNEL_4);
    dma_tx.priority(dma2::Priority::MEDIUM);
    dma_tx.memdata_alignment(dma2::DataSize::BYTE);
    dma_tx.periphdata_alignment(dma2::DataSize::BYTE);
    dma_tx.memory_increment(true);
    dma_tx.direction(dma2::Direction::MEMORY_TO_PERIPH);
    dma_tx.reg.s2cr.modify(|_, w| w.tcie().set_bit());

    dma_rx.disable();
    dma_tx.disable();

}

pub fn spi_setup(p: &stm32f411::Peripherals) {
    p.RCC.ahb1enr.modify(|_, w| w.gpioaen().set_bit());
    p.RCC.ahb1enr.modify(|_, w| w.gpioben().set_bit());
    p.RCC.apb2enr.modify(|_, w| w.spi4en().set_bit());

    ::tlc::CLK.set_mode(p.GPIOB, Mode::AlternateFunction);
    ::tlc::MOSI.set_mode(p.GPIOA, Mode::AlternateFunction);
    ::tlc::MISO.set_mode(p.GPIOA, Mode::AlternateFunction);
    ::tlc::LATCH.set_mode(p.GPIOA, Mode::Output);

    ::tlc::CLK.alternate_function(p.GPIOB, 6);
    ::tlc::MOSI.alternate_function(p.GPIOA, 5);
    ::tlc::MISO.alternate_function(p.GPIOA, 6);

    ::tlc::MOSI.set_pupd(p.GPIOA, Pupd::PullUp);
    ::tlc::CLK.set_pupd(p.GPIOB, Pupd::PullUp);

    ::tlc::LATCH.set(p.GPIOA, Io::Low);

    // let dma = dma2::Dma::new(p.DMA1, dma2::DMAStream::Stream0);
    // let spi = spi2::Spi::new(p.SPI1, spi2::Role::MASTER, None::<stm32f411::DMA1>, None);
    let spi = spi2::Spi::<_, stm32f411::DMA1>::new(p.SPI4, spi2::Role::MASTER, None, None);

    spi.init(spi2::Role::MASTER);
    spi.baud_rate_prescaler(spi2::BaudRatePreScale::SCALE_64);
    spi.clk_phase(spi2::Phase::_1EDGE);
    spi.clk_polarity(spi2::Polarity::LOW);
    spi.data_size(spi2::DataSize::_8BIT);
    spi.msb_first(true);
    spi.crc_calculation(false);
    // spi.nss(spi2::NSS::HardOutput);
    spi.nss(spi2::NSS::Soft);

    // spi.reg.cr2.modify(|_, w| w.txdmaen().set_bit());
    // spi.reg.cr2.modify(|_, w| w.rxdmaen().set_bit());
}


pub fn pwm_setup(p: &stm32f411::Peripherals) {
    p.RCC.ahb1enr.modify(|_, w| w.gpioaen().set_bit());
    p.RCC.apb2enr.modify(|_, w| w.tim1en().set_bit());

    unsafe {
        p.GPIOA.moder.modify(|_, w| {
            w.moder8().bits(0b10)
        });

        p.GPIOA.afrh.modify(|_, w| {
            w.afrh8().bits(0b1)
        });

        p.GPIOA.ospeedr.modify(|_, w| {
            w.ospeedr8().bits(0b10)
        })
    }

}

pub const DEBUG_TX: Pin<stm32f411::GPIOA> = Pin::new(2);
pub const DEBUG_RX: Pin<stm32f411::GPIOA> = Pin::new(3);

pub fn serial_setup(p: &stm32f411::Peripherals) {
    p.RCC.apb1enr.modify(|_, w| w.usart2en().set_bit());

    DEBUG_RX.set_mode(p.GPIOA, Mode::AlternateFunction);
    DEBUG_TX.set_mode(p.GPIOA, Mode::AlternateFunction);

    DEBUG_RX.alternate_function(p.GPIOA, 7);
    DEBUG_TX.alternate_function(p.GPIOA, 7);
}