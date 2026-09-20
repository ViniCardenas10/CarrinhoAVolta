#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <spi.h>
#include <nrf24l01.h>
#include <string.h>

// Pins - Atualizados para a FRDM KL25Z conforme a imagem
#define CE_PORT DEVICE_DT_NAME(DT_NODELABEL(gpiod))
#define CE_PIN 5 // PTD5

#define CSN_PORT DEVICE_DT_NAME(DT_NODELABEL(gpiod))
#define CSN_PIN 0 // PTD0

#define IRQ_PORT DEVICE_DT_NAME(DT_NODELABEL(gpioa))
#define IRQ_PIN 13 // PTA13

// Settings
uint8_t rx_address[5] = { 0x37, 0xa7, 0xe0, 0xb3, 0x97 };	// Read pipe address
uint8_t tx_address[5] = { 0x37, 0xa7, 0xe0, 0xb3, 0x97 };	// Write pipe address
#define READ_PIPE		0

#define AUTO_ACK		true								// Auto acknowledgment
#define DATARATE		RF_DR_1MBPS							// 250kbps, 1mbps, 2mbps
#define POWER			POWER_MAX							// Set power (MAX 0dBm..HIGH -6dBm..LOW -12dBm.. MIN -18dBm)
#define CHANNEL			0x6D								// 2.4GHz-2.5GHz channel selection (0x01 - 0x7C)
#define DYN_PAYLOAD		true								// Dynamic payload enabled
#define CONTINUOUS		false								// Continuous carrier transmit mode (not tested)

#define IRQ_RX_DR       true
#define IRQ_TX_DS       false
#define IRQ_MAX_RT      false

uint8_t send_to_spi;
const struct device *ce_port_dev, *csn_port_dev, *irq_port_dev;

void ce_low(){
    gpio_pin_set(ce_port_dev, CE_PIN, 0);
}

void ce_high(){
    gpio_pin_set(ce_port_dev, CE_PIN, 1);
}

void csn_low(){
    gpio_pin_set(csn_port_dev, CSN_PIN, 0);
}

void csn_high(){
    gpio_pin_set(csn_port_dev, CSN_PIN, 1);
}

void nrf24_clear_irq_flags(){
    send_to_spi=
    (1 << RX_DR) |	// RX FIFO
	(1 << TX_DS) |	// TX FIFO
	(1 << MAX_RT);  // MAX RT
    nrf24_write(STATUS,&send_to_spi,1);
}

void nrf24_init_gpio(){
    ce_port_dev=device_get_binding(CE_PORT);
    if (!ce_port_dev) {
        printk("Erro ao acessar porta %s\n", CE_PORT);
    }
    if (gpio_pin_configure(ce_port_dev, CE_PIN, GPIO_OUTPUT) != 0) {
        printk("Erro ao configurar pino %d\n", CE_PIN);
    }

    csn_port_dev=device_get_binding(CSN_PORT);
    if (!csn_port_dev) {
        printk("Erro ao acessar porta %s\n", CSN_PORT);
    }
    if (gpio_pin_configure(csn_port_dev, CSN_PIN, GPIO_OUTPUT) != 0) {
        printk("Erro ao configurar pino %d\n", CSN_PIN);
    }

    irq_port_dev=device_get_binding(IRQ_PORT);
    if (!irq_port_dev) {
        printk("Erro ao acessar porta %s\n", IRQ_PORT);
    }
    if (gpio_pin_configure(csn_port_dev, IRQ_PIN, GPIO_INPUT) != 0) {
        printk("Erro ao configurar pino %d\n", IRQ_PIN);
    }
    csn_high();
    ce_low();
}


void nrf24_init(){
    // Initialize GPIO (CSN HIGH and CE LOW)
    nrf24_init_gpio();
    csn_high();
    ce_low();

    // Initialize SPI - Atualizado para SPI_0
    spi_init(SPI_0, ALT_0, PRESCALE_0, DIVISOR_1, CS_MAN);
    
    // Power on reset: 100 ms
    k_msleep(100);

    // Initial configuration
    // CONFIG
    send_to_spi=
    (!(IRQ_RX_DR) << MASK_RX_DR)|
    (!(IRQ_TX_DS) << MASK_TX_DS)|
    (!(IRQ_MAX_RT) << MASK_MAX_RT)|
    (1 << EN_CRC)|
    (1 << CRC0)|
    (1 << PWR_UP)|
    (1 << PRIM_RX);
    nrf24_write(CONFIG, &send_to_spi, 1);

    // EN_AA
    send_to_spi=
    (AUTO_ACK << ENAA_P5)|
    (AUTO_ACK << ENAA_P4)|
    (AUTO_ACK << ENAA_P3)|
    (AUTO_ACK << ENAA_P2)|
    (AUTO_ACK << ENAA_P1)|
    (AUTO_ACK << ENAA_P0);
    nrf24_write(EN_AA, &send_to_spi, 1);

    // SETUP_AW
    send_to_spi=0x03;
    nrf24_write(SETUP_AW, &send_to_spi, 1);

    // SETUP_RETR
    send_to_spi=0xfa; // RT delay: 4000us & up to 2 RT
    nrf24_write(SETUP_RETR, &send_to_spi, 1);

    // RF_CH
    send_to_spi=CHANNEL;
    nrf24_write(RF_CH, &send_to_spi, 1);

    // RF_SETUP
    send_to_spi=
    (CONTINUOUS << CONT_WAVE) |					// Continuous carrier transmit
	((DATARATE >> RF_DR_HIGH) << RF_DR_HIGH) |	// Data rate
	((POWER >> RF_PWR) << RF_PWR);				// PA level
    nrf24_write(RF_SETUP, &send_to_spi, 1);

    // Clear IRQ flags
    nrf24_clear_irq_flags();

    // Dynamic payload on all pipes
	send_to_spi =
	(DYN_PAYLOAD << DPL_P0) |
	(DYN_PAYLOAD << DPL_P1) |
	(DYN_PAYLOAD << DPL_P2) |
	(DYN_PAYLOAD << DPL_P3) |
	(DYN_PAYLOAD << DPL_P4) |
	(DYN_PAYLOAD << DPL_P5);
	nrf24_write(DYNPD, &send_to_spi,1);

	// Enable dynamic payload
	send_to_spi =
	(DYN_PAYLOAD << EN_DPL) |
	(AUTO_ACK << EN_ACK_PAY) |
	(AUTO_ACK << EN_DYN_ACK);
	nrf24_write(FEATURE,&send_to_spi,1);

    // Flush TX/RX
	nrf24_send_spi(FLUSH_RX,0,0);
    nrf24_send_spi(FLUSH_TX,0,0);
	
	// Open pipes
    nrf24_write(RX_ADDR_P0 + READ_PIPE,rx_address,5);
    nrf24_write(TX_ADDR,tx_address,5);    
	send_to_spi = (1 << READ_PIPE) | 0x03;
	nrf24_write(EN_RXADDR,&send_to_spi,1);
}


uint8_t nrf24_send_spi(uint8_t register_address, void *data, unsigned int bytes){
	uint8_t status;
	csn_low();
    // Atualizado para SPI_0
	status = spi_exchange(SPI_0, register_address);
	for (unsigned int i = 0; i < bytes; i++)
	    ((uint8_t*)data)[i] = spi_exchange(SPI_0,((uint8_t*)data)[i]);
	csn_high();
	return status;
}

uint8_t nrf24_read(uint8_t register_address, uint8_t *data, unsigned int bytes){
	return nrf24_send_spi(R_REGISTER | register_address, data, bytes);
}

uint8_t nrf24_write(uint8_t register_address, uint8_t *data, unsigned int bytes){
	return nrf24_send_spi(W_REGISTER | register_address, data, bytes);
}


uint8_t nrf24_send_message(char *tx_message){
    uint8_t status, fifo_status, config_register;

    // Message length
	uint8_t length = strlen(tx_message);
    
	nrf24_read(CONFIG,&config_register,1);
    
    // Set the CONFIG bit PRIM_RX low
    config_register &= ~(1 << PRIM_RX);
    nrf24_write(CONFIG, &config_register, 1);

    // Flush TX/RX and clear TX interrupt
    nrf24_send_spi(FLUSH_RX,0,0);
    nrf24_send_spi(FLUSH_TX,0,0);
    nrf24_clear_irq_flags();

    nrf24_read(FIFO_STATUS, &fifo_status, 1);
    
    // Start SPI, load message into TX_PAYLOAD (Atualizado para SPI_0)
    csn_low();
	if (AUTO_ACK) spi_send(SPI_0, W_TX_PAYLOAD);
	else spi_send(SPI_0, W_TX_PAYLOAD_NOACK);
	uint8_t i=0;
    while (i<=length){
        spi_send(SPI_0, tx_message[i]);
        i++;
    }
    spi_send(SPI_0, 0); // Send '\0' (NULL terminator)
    csn_high();

    // Send message by pulling CE high for more than 10us
	ce_high();

    // Read status
    nrf24_read(FIFO_STATUS, &fifo_status, 1);
    nrf24_read(STATUS,&status,1);
	while(!(status & (1 << TX_DS)) && !(status & (1 << MAX_RT))) nrf24_read(STATUS,&status,1);

    status=nrf24_read(FIFO_STATUS, &fifo_status, 1);
    printk("STATUS: 0x%x - FIFO STATUS: 0x%x\n", status, fifo_status);

    if((status & (1 << MAX_RT)) || (status & (TX_FULL))){
        nrf24_send_spi(FLUSH_TX,0,0);
        nrf24_clear_irq_flags();
        ce_low(); // Set CE low to enter standby-I mode
        printk("Reached MAX_RT or TX_FULL.\n");
        return 0;
    }

    // Flush TX/RX and clear TX interrupt
	nrf24_send_spi(FLUSH_TX,0,0);
    nrf24_clear_irq_flags();
    ce_low(); // Set CE low to enter standby-I mode
    
    return 1;
}

char * nrf24_read_message(void){
    uint8_t config_register, width, status, fifo_status;

    // Message placeholder
	static char rx_message[32];
	memset(rx_message,0,32);

    // CE low
    ce_low();

    // Set the CONFIG bit PRIM_RX high
    nrf24_read(CONFIG, &config_register, 1);
    config_register |= (1 << PRIM_RX);
    nrf24_write(CONFIG, &config_register, 1);

    // A high pulse CE starts the active RX mode
    ce_high();

    // After 130us, nRF24L01+ monitors the air for incoming communication.
    k_usleep(130);

    // Read status
    status=nrf24_read(FIFO_STATUS, &fifo_status,1);
    printk("STATUS: 0x%x - FIFO STATUS: 0x%x\n", status, fifo_status);

    nrf24_read(STATUS,&status,1);
    while(!(status & (1 << RX_DR))) nrf24_read(STATUS,&status,1);

    ce_low();

    status=nrf24_read(FIFO_STATUS, &fifo_status,1);
    printk("STATUS: 0x%x - FIFO STATUS: 0x%x\n", status, fifo_status);

    // Recover the payload width
    nrf24_read(R_RX_PL_WID,&width,1);

    // Recover the payload - Read message
    if (width > 32){
        nrf24_send_spi(FLUSH_RX,0,0);
        nrf24_clear_irq_flags();
        return "failed";
    }
	if (width > 0) nrf24_send_spi(R_RX_PAYLOAD,&rx_message,width);
    printk("Message received. PL_WID: %u - MSG: %s\n", width, rx_message);

    // Flush TX/RX and clear TX interrupt
	nrf24_write(FLUSH_RX,0,0);
    nrf24_clear_irq_flags();

    // Check if there is message in array
	if (strlen(rx_message) > 0) return rx_message;
	return "failed";
}