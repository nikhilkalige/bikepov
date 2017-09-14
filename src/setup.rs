use bsp::stm32f411;
use bsp::spi2;
use bsp::dma2;

pub fn dma_setup(p: &stm32f411::Peripherals) {
    p.RCC.ahb1enr.modify(|_, w| w.dma2en().set_bit());

    let dma_rx = dma2::Dma::new(p.DMA2, dma2::DMAStream::Stream0);
    let dma_tx = dma2::Dma::new(p.DMA2, dma2::DMAStream::Stream2);

    dma_rx.channel(dma2::Channel::CHANNEL_3);
    dma_rx.priority(dma2::Priority::MEDIUM);
    dma_rx.memdata_alignment(dma2::DataSize::BYTE);
    dma_rx.periphdata_alignment(dma2::DataSize::BYTE);
    dma_rx.memory_increment(true);
    dma_rx.direction(dma2::Direction::PERIPH_TO_MEMORY);
    dma_rx.reg.s0cr.modify(|_, w| w.tcie().set_bit());

    dma_tx.channel(dma2::Channel::CHANNEL_2);
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
    p.RCC.apb2enr.modify(|_, w| w.spi1en().set_bit());

    unsafe {
        p.GPIOA.moder.modify(|_, w| {
            w.moder4().bits(0b10)
                .moder5().bits(0b10)
                .moder6().bits(0b10)
                .moder7().bits(0b10)
        });

        p.GPIOA.afrl.modify(|_, w| {
            w.afrl4().bits(0b101)
                .afrl5().bits(0b101)
                .afrl6().bits(0b101)
                .afrl7().bits(0b101)
        });
    }

    // let dma = dma2::Dma::new(p.DMA1, dma2::DMAStream::Stream0);
    // let spi = spi2::Spi::new(p.SPI1, spi2::Role::MASTER, None::<stm32f411::DMA1>, None);
    let spi = spi2::Spi::<_, stm32f411::DMA1>::new(p.SPI1, spi2::Role::MASTER, None, None);

    spi.init(spi2::Role::MASTER);
    spi.baud_rate_prescaler(spi2::BaudRatePreScale::SCALE_64);
    spi.clk_phase(spi2::Phase::_1EDGE);
    spi.clk_polarity(spi2::Polarity::LOW);
    spi.data_size(spi2::DataSize::_8BIT);
    spi.msb_first(true);
    spi.crc_calculation(false);
    spi.nss(spi2::NSS::HardOutput);

    spi.reg.cr2.modify(|_, w| w.txdmaen().set_bit());
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
