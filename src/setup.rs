use bsp::stm32f411;
extern crate bsp;

use bsp::spi2;
// use bsp::dma2;

pub fn spi_setup(p: stm32f411::Peripherals) {
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
    spi.nss(spi2::NSS::HARD_OUTPUT);
}