#include "stm32f4xx.h"                  // Device header

void delay(uint32_t time)
{
	while(time--);
}

void clck_config()
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
	
	
	}
	
void GPIO_Config()
{
	RCC->AHB1ENR |= 0x00000005;             /* GPIOA and GPIOC Enable */
	GPIOA->MODER |= 0x00000400;             /* GPIOA Pin 5 chose General Purpose output mode*/
	GPIOA->OSPEEDR |= 0x00000C00;      			/* GPIOA Speed  High Speed. */
	GPIOC->MODER |= 0x01100000;             /* GPIOC Pin 10 and Pin 12  General Purpose output mode */
	GPIOC->OSPEEDR |= 0x03300000;           /* GPIOC Pin 10 and Pin 12  High Speed */

}

void EXTI_Config()
{
	RCC->APB2ENR |= 0x00004000;             /* SYS CNFG active (External Interrupt) */
	SYSCFG->EXTICR [0] = 0x00000000;
	SYSCFG->EXTICR [1] = 0x00000000;
	NVIC_EnableIRQ(EXTI0_IRQn);							/* Ext. Interrupt 0 activated with NVIC */
	NVIC_EnableIRQ(EXTI1_IRQn);             /* Ext. Interrupt 1 activated with NVIC */
	NVIC_EnableIRQ(EXTI2_IRQn);             /* Ext. Interrupt 2 activated with NVIC */
	
	NVIC_SetPriority(EXTI0_IRQn , 0);       /* ilk Butona en öncelikli sira verildi */   
	NVIC_SetPriority(EXTI1_IRQn , 1);
	NVIC_SetPriority(EXTI2_IRQn , 2);
	EXTI->IMR = 0x00000003 ;                /* Interrupt olarak kullanicagimizi belirttik event olsaydi emr secilecekti.*/
	EXTI->RTSR = 0x00000003 ;               /* Interrupt yükselen kenar olarak secildi. */
	
}

void EXTI_Control1()
{
		if(EXTI->PR & 0x00000001)
			{
				GPIOA->ODR = 0x00000020;
				delay(16800000);
				EXTI->PR = 0x00000002;
			}
}

void EXTI_Control2()
{
		if(EXTI->PR & 0x00000002)
			{
				GPIOC->ODR = 0x00000400;
				delay(16800000);
				EXTI->PR = 0x00000002;
			}
}

void EXTI_Control3() 
{
		if(EXTI->PR & 0x00000004)
			{
				GPIOC->ODR = 0x00001000;
				delay(16800000);
				EXTI->PR = 0x00000004;
			}
}

int main ()
{
	clck_config();
  GPIO_Config ();
	EXTI_Config ();
	while(1)
		{
			GPIOC->ODR = 0x00001400;               /* GPIOC Pin 10 and Pin 12 ODR */

		}
}
