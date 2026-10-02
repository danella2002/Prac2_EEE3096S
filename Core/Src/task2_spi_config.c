/**
  ******************************************************************************
  * @file    task2_spi_config.c
  * @brief   TASK 2 : CONFIGURE THE HARDWARE SPI PERIPHERAL
  *
  * Configure the SPI peripheral entirely through its registers. Not allowed:
  * HAL_SPI_*, LL SPI transfer functions, CubeMX-generated SPI configuration,
  * GPIO bit-banging. Every register write you add should cite the RM0091
  * register it comes from.
  ******************************************************************************
  */

#include "prac2a.h"

volatile uint32_t dbg_gpiob_moder      = 0u;
volatile uint32_t dbg_gpiob_afrh       = 0u;
volatile uint32_t dbg_spi_cr1          = 0u;
volatile uint32_t dbg_spi_cr2          = 0u;
volatile uint32_t dbg_spi_sr           = 0u;
volatile uint32_t dbg_sck_hz_predicted = 0u;

void eeprom_spi_init(void)
{
    /* TODO 2.4  Enable the clocks this needs: the GPIO port that carries the
     *           pins AND the SPI peripheral itself. They are in different RCC
     *           enable registers - find both in RM0091. */

    /* RCC_AHBENR bit 18 IOPBEN   (RM0091 Sec. 6.4.6)
     * RCC_APB1ENR bit 14 SPI2EN  (RM0091 Sec. 6.4.8) */
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    (void)RCC->APB1ENR;             /* read back: clock is running before use */

    /* TODO 2.5  Chip select: make EE_PIN_CS a general purpose output driven
     *           HIGH. Think about the ORDER of those two steps. Be ready to
     *           explain why CS must start high. */

    /* Level FIRST (BSRR set bit), THEN mode. ODR already holds 1 when MODER
         * switches the pin to output, so the pin goes straight from floating input
         * to HIGH and never glitches low (a low glitch would select the EEPROM). */

     EE_SPI_GPIO->BSRR = EE_CS_MASK;
     EE_SPI_GPIO->MODER = (EE_SPI_GPIO->MODER & ~MODER2_MASK(EE_PIN_CS))
                           | MODER2(EE_PIN_CS, 1u);            /* 01 = output */

    /* TODO 2.6  SCK, MISO and MOSI: put them in alternate-function mode
     *           (MODER), and select the alternate function number EE_SPI_AF
     *           (the AF register for pins 8..15). Both registers are needed. */

     EE_SPI_GPIO->AFR[1] = (EE_SPI_GPIO->AFR[1]
                                  & ~(AFRH4_MASK(EE_PIN_SCK) | AFRH4_MASK(EE_PIN_MISO)
                                      | AFRH4_MASK(EE_PIN_MOSI)))
                                | AFRH4(EE_PIN_SCK,  EE_SPI_AF)
                                | AFRH4(EE_PIN_MISO, EE_SPI_AF)
                                | AFRH4(EE_PIN_MOSI, EE_SPI_AF);
     EE_SPI_GPIO->MODER = (EE_SPI_GPIO->MODER
                                 & ~(MODER2_MASK(EE_PIN_SCK) | MODER2_MASK(EE_PIN_MISO)
                                     | MODER2_MASK(EE_PIN_MOSI)))
                               | MODER2(EE_PIN_SCK,  2u)            /* 10 = AF */
                               | MODER2(EE_PIN_MISO, 2u)
                               | MODER2(EE_PIN_MOSI, 2u);

    /* TODO 2.7  Recommended: high output speed on SCK and MOSI, and a pull-up
     *           on MISO. In your report, explain what the EEPROM does with its
     *           output pin while CS is high, and why a pull-up helps. */

    /* OSPEEDR 11 = high speed (Sec. 8.4.3); PUPDR 01 = pull-up (Sec. 8.4.4).
         * With CS high the EEPROM's SO pin is high-impedance, so MISO would float;
         * the pull-up holds it at a defined level. */
        EE_SPI_GPIO->OSPEEDR |= MODER2(EE_PIN_SCK, 3u) | MODER2(EE_PIN_MOSI, 3u);
        EE_SPI_GPIO->PUPDR = (EE_SPI_GPIO->PUPDR & ~MODER2_MASK(EE_PIN_MISO))
                           | MODER2(EE_PIN_MISO, 1u);


    /* TODO 2.8  SPI_CR2: 8-bit data frames, and a receive FIFO threshold that
     *           reports a received byte after 8 bits. Read the RM0091
     *           description of the FIFO threshold carefully - on the STM32F0
     *           the reset value does not suit single-byte transfers.
     *           Configure CR2 BEFORE enabling the peripheral. */

   /* SPI_CR2 (RM0091 Sec. 28.9.2):
    *   DS[3:0] = 0111  -> 8-bit data size
    *   FRXTH   = 1     -> RXNE set when the RX FIFO holds 8 bits (reset value
    *                      0 waits for 16 bits, so a single byte would never
    *                      raise RXNE). */
        EE_SPI->CR2 = (7UL << SPI_CR2_DS_Pos)| SPI_CR2_FRXTH;

    /* TODO 2.9  SPI_CR1: master mode, your baud-rate divider, the clock
     *           polarity and phase the EEPROM supports (EEPROM datasheet), and
     *           the bit order.
     *           You are driving CS yourself on a GPIO. Read RM0091 on
     *           slave-select (NSS) management in master mode: if the
     *           peripheral believes its NSS input is low it will leave master
     *           mode on its own, and you will see no clock at all. */

    /* SPI_CR1 (RM0091 Sec. 28.9.1). SPE is still 0 here.
     *   MSTR     = 1        master
     *   BR[2:0]  = EE_SPI_BR  f_PCLK / 2^(BR+1) = 8 MHz / 32 = 250 kHz
     *   CPOL = 0, CPHA = 0  SPI mode 0: SCK idles low, data sampled on the
     *                       rising edge (EEPROM supports modes 0 and 3)
     *   LSBFIRST = 0        MSB first (EEPROM protocol)
     *   SSM = 1, SSI = 1    software NSS management, internal NSS held high,
     *                       so no mode fault (MODF) drops us out of master */
        EE_SPI->CR1 = SPI_CR1_MSTR
                    | (EE_SPI_BR << SPI_CR1_BR_Pos)
                    | SPI_CR1_SSM
                    | SPI_CR1_SSI;

    /* Task 5 fault case. Leave this call exactly here: after your CR1 and CR2
     * configuration, before the peripheral is enabled. It does nothing unless
     * RUN_TASK is 5. */
    task5_fault_hook();

    /* TODO 2.10  Enable the peripheral. */
    EE_SPI->CR1 |= SPI_CR1_SPE;

    dbg_gpiob_moder      = EE_SPI_GPIO->MODER;
    dbg_gpiob_afrh       = EE_SPI_GPIO->AFR[1];
    dbg_spi_cr1          = EE_SPI->CR1;
    dbg_spi_cr2          = EE_SPI->CR2;
    dbg_spi_sr           = EE_SPI->SR;
    dbg_sck_hz_predicted = EE_SCK_HZ_PREDICTED;
}

/* ==========================================================================
 * RUN_TASK 2 - given
 * Configures SPI and then does nothing else, so you can inspect the selected
 * SPI peripheral and GPIOB in the SFR view (or the dbg_* variables) while the
 * program runs. PC13 keeps toggling so you can see the program has not hung.
 * ========================================================================== */

void task2_setup(void)
{
    task1_gpio_init();
    eeprom_spi_init();
}

void task2_loop(uint32_t now)
{
    task1_gpio_update(now);
}
