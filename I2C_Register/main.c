#include "stm32f4xx.h"                  // Device header

void delay(uint32_t time)
{
		while(time--);
}
void Clck_Config()
{
	RCC->CR |= 0x00010000;                 /* Chose  High Speed External (HSEON Enable)*/
	while(!(RCC->CR & 0x00020000));        /* Waiting HSE Ready flag */
	RCC->CR |= 0x00080000;                 /* CSSON Enable */
	RCC->CR |= 0x01000000;                 /* PLLON Enable */
  RCC->PLLCFGR |= 0x00400000;	           /* Chose HSE for PLL */
	RCC->PLLCFGR |= 0x00000004;            /* PLLM = 4 (000100) */
  RCC->PLLCFGR |= 0x00002A00;            /* PLLN = 168 */
  RCC->PLLCFGR |= 0x00000000;            /* PLLP = 2 */
  RCC->CFGR |= 0x00000000;               /* AHB Prescaler = 1 */	
	RCC->CFGR |= 0x00008000;               /* ABP2 = 2 */ 
	RCC->CFGR |= 0x00001400;               /* ABP1 = 4 */
	RCC->CIR |= 0x00080000;                /* HSERDY Flag = 1  */
	RCC->CIR |= 0x00800000;
}

void GPIO_Config()
{
	RCC->AHB1ENR |= (1<<1) | (1<<2);            				// GPIOA and GPIOB Clock Activated
	
	GPIOC->MODER |= (0<<26) | (0<<27);
	GPIOC->PUPDR |= (2<<26);
	
	GPIOB->AFR[1] |= (4<<8) | (4<<12);				// PB10 and PB11 AF
	GPIOB->MODER |= (2<<20) | (2<<22) ;       // PB10 and PB11 AF 
	GPIOB->OTYPER |= (1<<11) | (1<<12);				// PB10 and PB11 Open drain for I2C
	
}

void I2C_Config()
{
	RCC->APB1ENR |= (1<<22);									// I2C activated
	I2C2->CR2 |= 0x0008;											// I2C clock 8 MHz
	I2C2->CCR |= 0x0028;											// I2C 100 Khz
	I2C2->TRISE |= 0x09;
	I2C2->CR1 |= 0x01;												// I2C enable
}

void I2C2_Write(uint8_t adress, uint8_t data)
{
	I2C2->CR1 |= (1<<8);											// Send Start bit.
	while(!(I2C2->SR1 & 0x0001));							// Wait until send start bit
	I2C2->DR = 0x4E;												  // Slave Adress
	while(!(I2C2->SR1 & 0x0002));							// Wait until send adress
	while(!(I2C2->SR2 & 0x0001));							// Wait Master/Slave
	// I2C2->DR = adress;											// The address where we will write the data in slave 
	while(!(I2C2->SR1 & 0x0080));							// Wait TXE 
	I2C2->DR = data;													// Write data on data buffer
	while(!(I2C2->SR1 & 0x0080));							// Wait TXE
	while(!(I2C2->SR1 & 0x0004));							// Wait BTF(Data send succesful)
	I2C2->CR1 |= (1<<9);											// Send Stop bit
	
}

int main()
{
	uint8_t i = 0;
	Clck_Config();
	GPIO_Config();
	I2C_Config();
	
	while(1)
	{
			while(GPIOC->IDR & 0x2000);		
			i++;
			delay(1680000);
			if(i == 8)
			{
				i = 0;
			}			
		switch(i)
		{
			case 0:
				I2C2_Write(0x4E,0x00);
				break;
			case 1:
				I2C2_Write(0x4E,0x01);
		}
	}
}
