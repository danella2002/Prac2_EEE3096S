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
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;     /* RM0091 RCC_AHBENR  bit 18 IOPBEN */
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;     /* RM0091 RCC_APB1ENR bit 14 SPI2EN */
    (void)RCC->APB1ENR;                     /* read back: clocks are running    */

    /* TODO 2.5  Chip select: make EE_PIN_CS a general purpose output driven
     *           HIGH. Think about the ORDER of those two steps. Be ready to
     *           explain why CS must start high. */
    EE_SPI_GPIO->BSRR = EE_CS_MASK;         /* 1st: output latch HIGH (BSRR)    */
    EE_SPI_GPIO->MODER = (EE_SPI_GPIO->MODER & ~MODER2_MASK(EE_PIN_CS))
                       | MODER2(EE_PIN_CS, 1u);  /* 2nd: MODER12 = 01 output   */

    /* TODO 2.6  SCK, MISO and MOSI: put them in alternate-function mode
     *           (MODER), and select the alternate function number EE_SPI_AF
     *           (the AF register for pins 8..15). Both registers are needed. */
    EE_SPI_GPIO->AFR[1] = (EE_SPI_GPIO->AFR[1]              /* RM0091 GPIOx_AFRH */
                          & ~(AFRH4_MASK(EE_PIN_SCK) | AFRH4_MASK(EE_PIN_MISO)
                            | AFRH4_MASK(EE_PIN_MOSI)))
                        | AFRH4(EE_PIN_SCK,  EE_SPI_AF)
                        | AFRH4(EE_PIN_MISO, EE_SPI_AF)
                        | AFRH4(EE_PIN_MOSI, EE_SPI_AF);

    EE_SPI_GPIO->MODER = (EE_SPI_GPIO->MODER                /* RM0091 GPIOx_MODER */
                         & ~(MODER2_MASK(EE_PIN_SCK) | MODER2_MASK(EE_PIN_MISO)
                           | MODER2_MASK(EE_PIN_MOSI)))
                       | MODER2(EE_PIN_SCK,  2u)            /* 10 = alternate fn */
                       | MODER2(EE_PIN_MISO, 2u)
                       | MODER2(EE_PIN_MOSI, 2u);

    /* TODO 2.7  Recommended: high output speed on SCK and MOSI, and a pull-up
     *           on MISO. In your report, explain what the EEPROM does with its
     *           output pin while CS is high, and why a pull-up helps. */
    EE_SPI_GPIO->OSPEEDR |= MODER2(EE_PIN_SCK,  3u)         /* RM0091 GPIOx_OSPEEDR */
                          | MODER2(EE_PIN_MOSI, 3u);        /* 11 = high speed      */

    EE_SPI_GPIO->PUPDR = (EE_SPI_GPIO->PUPDR                /* RM0091 GPIOx_PUPDR   */
                         & ~MODER2_MASK(EE_PIN_MISO))
                       | MODER2(EE_PIN_MISO, 1u);           /* 01 = pull-up         */

    /* TODO 2.8  SPI_CR2: 8-bit data frames, and a receive FIFO threshold that
     *           reports a received byte after 8 bits. Read the RM0091
     *           description of the FIFO threshold carefully - on the STM32F0
     *           the reset value does not suit single-byte transfers.
     *           Configure CR2 BEFORE enabling the peripheral. */
    EE_SPI->CR2 = SPI_CR2_FRXTH                             /* RM0091 SPI_CR2       */
                | (7UL << SPI_CR2_DS_Pos);                  /* DS = 0111: 8-bit     */

    /* TODO 2.9  SPI_CR1: master mode, your baud-rate divider, the clock
     *           polarity and phase the EEPROM supports (EEPROM datasheet), and
     *           the bit order.
     *           You are driving CS yourself on a GPIO. Read RM0091 on
     *           slave-select (NSS) management in master mode: if the
     *           peripheral believes its NSS input is low it will leave master
     *           mode on its own, and you will see no clock at all. */
    EE_SPI->CR1 = SPI_CR1_MSTR                              /* RM0091 SPI_CR1       */
                | (EE_SPI_BR << SPI_CR1_BR_Pos)             /* BR = 100: /32        */
                | SPI_CR1_SSM                               /* software NSS         */
                | SPI_CR1_SSI;                              /* internal NSS = HIGH  */
                /* CPOL = 0, CPHA = 0 (mode 0), LSBFIRST = 0 (MSB first),
                   SPE = 0 (enabled last, TODO 2.10)                            */

    /* Task 5 fault case. Leave this call exactly here: after your CR1 and CR2
     * configuration, before the peripheral is enabled. It does nothing unless
     * RUN_TASK is 5. */
    task5_fault_hook();

    /* TODO 2.10  Enable the peripheral. */
    EE_SPI->CR1 |= SPI_CR1_SPE;                             /* RM0091 SPI_CR1 SPE   */

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
