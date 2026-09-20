#ifndef NRF24_Z42_H
#define NRF24_Z42_H

#include <stdint.h>
#include <stdbool.h>

/* ---- Comandos SPI do nRF24L01+ (datasheet, seção 8.3.1) ---- */
#define NRF_CMD_R_REGISTER    0x00
#define NRF_CMD_W_REGISTER    0x20
#define NRF_CMD_R_RX_PAYLOAD  0x61
#define NRF_CMD_W_TX_PAYLOAD  0xA0
#define NRF_CMD_FLUSH_TX      0xE1
#define NRF_CMD_FLUSH_RX      0xE2
#define NRF_CMD_NOP           0xFF

/* ---- Endereços dos registradores usados ---- */
#define NRF_REG_CONFIG      0x00
#define NRF_REG_EN_AA       0x01
#define NRF_REG_EN_RXADDR   0x02
#define NRF_REG_SETUP_AW    0x03
#define NRF_REG_SETUP_RETR  0x04
#define NRF_REG_RF_CH       0x05
#define NRF_REG_RF_SETUP    0x06
#define NRF_REG_STATUS      0x07
#define NRF_REG_RX_ADDR_P0  0x0A
#define NRF_REG_TX_ADDR     0x10
#define NRF_REG_RX_PW_P0    0x11
#define NRF_REG_FIFO_STATUS 0x17

/* ---- Bits do registrador STATUS ---- */
#define NRF_STATUS_RX_DR    (1 << 6) /* dado recebido      */
#define NRF_STATUS_TX_DS    (1 << 5) /* dado enviado       */
#define NRF_STATUS_MAX_RT   (1 << 4) /* retransmissões esgotadas */

/* Payload de 1 byte: '1' liga o LED, '0' desliga */
#define NRF_PAYLOAD_SIZE    1

/* Inicializa SPI0, CE/CSN e os registradores do módulo (fica em Standby) */
void nrf24_init(void);

/* Coloca o módulo em modo Transmissor (PWR_UP=1, PRIM_RX=0) */
void nrf24_set_as_tx(void);

/* Coloca o módulo em modo Receptor (PWR_UP=1, PRIM_RX=1, CE=1) */
void nrf24_set_as_rx(void);

/* Envia 1 byte (bloqueia até TX_DS ou MAX_RT, ou até estourar o timeout) */
void nrf24_send(uint8_t data);

/* Retorna true se há um pacote pronto para leitura (bit RX_DR = 1) */
bool nrf24_data_available(void);

/* Lê o byte recebido do FIFO de RX e limpa a flag RX_DR */
uint8_t nrf24_read_payload(void);

#endif /* NRF24_Z42_H */
