#include "spi_z42.h"

void spi0_init(void) {
    /* 1. Habilita o clock do módulo SPI0 e das portas C e D */
    SIM->SCGC4 |= SIM_SCGC4_SPI0_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;

    /* 2. Configura os pinos para a função alternativa SPI0 (MUX = ALT2) */
    PORTD->PCR[2] = PORT_PCR_MUX(2); /* PTD2 = SPI0_MOSI */
    PORTD->PCR[3] = PORT_PCR_MUX(2); /* PTD3 = SPI0_MISO */
    PORTC->PCR[5] = PORT_PCR_MUX(2); /* PTC5 = SPI0_SCK  */

    /* 3. CSN (PTD0) e CE (PTD5) ficam como GPIO comum (MUX = ALT1).
     *    O CSN precisa permanecer em nível baixo durante uma transação
     *    inteira (vários bytes), o que o Chip Select automático do
     *    hardware SPI não faz (ele pulsa a cada byte) — por isso é
     *    controlado manualmente, assim como o CE. */
    PORTD->PCR[0] = PORT_PCR_MUX(1); /* PTD0 = GPIO (CSN) */
    PORTD->PCR[5] = PORT_PCR_MUX(1); /* PTD5 = GPIO (CE)  */
    GPIOD->PDDR |= (1 << 0) | (1 << 5);
    GPIOD->PSOR = (1 << 0); /* CSN = 1 -> módulo inativo no barramento SPI */
    GPIOD->PCOR = (1 << 5); /* CE  = 0 -> módulo em Standby */

    /* 4. Configura o SPI0: mestre, modo 0 (CPOL=0, CPHA=0), MSB primeiro */
    SPI0->C1 = SPI_C1_MSTR_MASK;
    SPI0->C2 = 0;

    /* 5. Baud rate = bus_clock / (SPPR+1) / 2^(SPR+1).
     *    Com SPPR=0 e SPR=2 temos bus_clock/8 (~1,5 MHz para bus_clock
     *    de ~12 MHz do KL25Z em FEI), bem abaixo do limite de 10 MHz
     *    do nRF24L01+. Ajuste se necessário para o seu clock de sistema. */
    SPI0->BR = SPI_BR_SPPR(0) | SPI_BR_SPR(2);

    /* 6. Habilita o módulo SPI0 */
    SPI0->C1 |= SPI_C1_SPE_MASK;
}

uint8_t spi0_transfer(uint8_t data) {
    /* Espera o buffer de transmissão (SPTEF) ficar livre para escrita */
    while (!(SPI0->S & SPI_S_SPTEF_MASK)) {}
    SPI0->D = data;

    /* Espera o dado recebido (SPRF) ficar disponível para leitura */
    while (!(SPI0->S & SPI_S_SPRF_MASK)) {}
    return SPI0->D;
}
