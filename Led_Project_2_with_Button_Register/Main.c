#include "stm32f4xx.h"                  // Device header

void delay(uint32_t time)
{
	while(time--);
}


void CLK_CONFIG()
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
	RCC->CIR |= 0x00800000;                /* CSS Flag = 1 */
	RCC->AHB1ENR |= 0x00000005;             /* GPIOA and GPIOC Enable */
		
	GPIOA->MODER = 0x00000400;             /* fifth port of A is general purpose output mode*/
	GPIOA->OTYPER = 0x00000000;            /* fifth port of A is push pull */
	GPIOA->OSPEEDR = 0x00000C00;           /* Medium speed */
	GPIOA->PUPDR = 0x00000000;             /* No Pushpull No PullUp */
	
}




int main(void) 
{
	int count = 0;
	CLK_CONFIG ();
	while(1)
	{
		if(GPIOC->IDR & 0x00002000)
		{
			while(GPIOC->IDR & 0x00002000);
			delay(1680000);
			count++ ;
		}
		if(count %2 == 0)
		{	
			GPIOA->ODR = 0x00000000 ;
	  }
		else
		{
			GPIOA->ODR = 0x00000020 ;
		}
	}
}	
