#include "spi.h"
 
bool spi_init(bool spi,bool alt,uint8_t pre, uint16_t div,bool cs)
{
	switch(spi)
	{
		case SPI_0:
			switch(alt)
			{
				case ALT_0: // Atualizado para a pinagem mista C e D
					SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;    // Turn on clock to C module
					SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;    // Turn on clock to D module
					
					if(cs == CS_AUT)					   // Chip Select Auto
						PORTD->PCR[0] = PORT_PCR_MUX(0x2); // Set PTD0 to mux 2 [SPI0_PCS0]
						
					PORTC->PCR[5] = PORT_PCR_MUX(0x2);     // Set PTC5 to mux 2 [SPI0_SCK]
					PORTD->PCR[2] = PORT_PCR_MUX(0x2);     // Set PTD2 to mux 2 [SPI0_MOSI]
					PORTD->PCR[3] = PORT_PCR_MUX(0x2);     // Set PTD3 to mux 2 [SPIO_MISO]
				break;
 
				case ALT_1:
					SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;    
					if(cs == CS_AUT)						
						PORTA->PCR[14] = PORT_PCR_MUX(0x2);   
					PORTA->PCR[15] = PORT_PCR_MUX(0x2);    	
					PORTA->PCR[16] = PORT_PCR_MUX(0x2);    	
					PORTA->PCR[17] = PORT_PCR_MUX(0x2);    	
				break;
 
				default:
					return false;
				break;
			}
			//Enable SPI0 clock
			SIM->SCGC4 |= SIM_SCGC4_SPI0_MASK;
 
			//Chip Select Auto
			if(cs == CS_AUT)
				SPI0->C1 = SPI_C1_MSTR_MASK | SPI_C1_SSOE_MASK;
			else
				SPI0->C1 = SPI_C1_MSTR_MASK;
 
			// Configure SPI Register C2
			SPI0->C2 = SPI_C2_MODFEN_MASK;   
 
			// Set baud rate prescale divisor to 6 & set baud rate divisor to 4 for baud rate of 1 Mhz
			SPI0->BR = (SPI_BR_SPPR(pre) | SPI_BR_SPR(div));    
 
			// Enable SPI0
			SPI0->C1 |= SPI_C1_SPE_MASK;
 
			return true;
		break;
 
		case SPI_1:
			switch(alt)
			{
				case ALT_0:
					SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;      
					if(cs == CS_AUT)						
						PORTE->PCR[4] = PORT_PCR_MUX(0x2);     
					PORTE->PCR[2] = PORT_PCR_MUX(0x2);    		
					PORTE->PCR[1] = PORT_PCR_MUX(0x2);    		
					PORTE->PCR[3] = PORT_PCR_MUX(0x2);    		
				break;
 
				case ALT_1:
					SIM->SCGC5  |= SIM_SCGC5_PORTB_MASK;     
					if(cs == CS_AUT)						
						PORTB->PCR[10] = PORT_PCR_MUX(0x2);    
					PORTB->PCR[11] = PORT_PCR_MUX(0x2);    	
					PORTB->PCR[16] = PORT_PCR_MUX(0x2);    	
					PORTB->PCR[17] = PORT_PCR_MUX(0x2);    	
				break;
 
				default:
					return false;
				break;
			}
			//Enable SPI1 clock
			SIM->SCGC4 |= SIM_SCGC4_SPI1_MASK;
 
			// Chip Select Auto
			if(cs == CS_AUT)
				SPI1->C1 = SPI_C1_MSTR_MASK | SPI_C1_SSOE_MASK;
			else
				SPI1->C1 = SPI_C1_MSTR_MASK;
 
			// Configure SPI Register C2
			SPI1->C2 = SPI_C2_MODFEN_MASK;   
 
			// Set baud rate prescale divisor to 6 & set baud rate divisor to 4 for baud rate of 1 Mhz
			SPI1->BR = (SPI_BR_SPPR(pre) | SPI_BR_SPR(div));    
 
			// Enable SPI0
			SPI1->C1 |= SPI_C1_SPE_MASK;
 
			return true;
		break;
 
		default:
			return false; // error
		break;
	}
}
 
bool spi_send(bool spi,uint8_t data)
{
	switch(spi)
	{
		case SPI_0:
			while(!(SPI_S_SPTEF_MASK & SPI0->S))
			{
				__asm("nop");
			}
			SPI0->D = data;
 
			return true;
		break;
 
		case SPI_1:
			while(!(SPI_S_SPTEF_MASK & SPI1->S))
			{
				__asm("nop");
			}
			SPI1->D = data;
 
			return true;
		break;
 
		default:
			return false; 
		break;
	}
}
 
uint8_t spi_read(bool spi)
{
	switch(spi)
	{
		case SPI_0:
			  while(!(SPI0->S & SPI_S_SPRF_MASK))
			  {
			    __asm("nop");
			  }
			  return SPI0->D;
		break;
 
		case SPI_1:
			  while(!(SPI1->S & SPI_S_SPRF_MASK))
			  {
			    __asm("nop");
			  }
			  return SPI1->D;
		break;
 
		default:
			return false;
		break;
	}
}

uint8_t spi_exchange(bool spi,uint8_t data)
{
	switch(spi)
	{
		case SPI_0:
			while(!(SPI_S_SPTEF_MASK & SPI0->S))
			{
				__asm("nop");
			}
			SPI0->D = data;
			while(!(SPI0->S & SPI_S_SPRF_MASK))
			{
				__asm("nop");
			}
			return SPI0->D;
		break;
 
		case SPI_1:
			while(!(SPI_S_SPTEF_MASK & SPI1->S))
			{
				__asm("nop");
			}
			SPI1->D = data;
			while(!(SPI1->S & SPI_S_SPRF_MASK))
			{
				__asm("nop");
			}
			return SPI1->D;
		break;
 
		default:
			return false; 
		break;
	}
}