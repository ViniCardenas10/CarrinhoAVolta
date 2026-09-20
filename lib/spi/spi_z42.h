#ifndef SPI_Z42_H
#define SPI_Z42_H

#include <stdint.h>
#include "MKL25Z4.h"

/* Inicializa o SPI0 em modo mestre para comunicação com o nRF24L01+
 * Pinagem (igual à do slide 13 da apresentação):
 *   PTD2 = MOSI
 *   PTD3 = MISO
 *   PTC5 = SCK
 *   PTD0 = CSN (controlado como GPIO comum, não como PCS de hardware)
 *   PTD5 = CE  (controlado como GPIO comum)
 */
void spi0_init(void);

/* Transfere 1 byte por SPI0 (full-duplex): envia 'data' e retorna o byte recebido */
uint8_t spi0_transfer(uint8_t data);

#endif /* SPI_Z42_H */
