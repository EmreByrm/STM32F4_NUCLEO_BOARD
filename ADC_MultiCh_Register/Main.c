#include "stm32f4xx.h"                  // Device header

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

void ADC_Config(void)
{
	
	// 1. Set ADC and GPIO Clock
	RCC->APB2ENR |= 0x00000100;  // ADC1 Clock Enable
	RCC->AHB1ENR |= 0x00000001;  // GPIOA Clock Activated
	
	// 2. Set Prescaler in the CCR(Common Control Register)
	// Our clock 168 MHz HSE and APB2 ADC clock 84 MHz so We will choose divider /4
	ADC->CCR |= (1<<16); // PCLK2 divided by 4
	
	// 3. Set the Scan mode and Resolution in CR1 
	ADC1->CR1  |= (1<<8);  // Scan mode enable
	ADC1->CR1  |= (1<<24); // Resolution 10 bit
	
	// 4. Set the Continuous Conversion, EOC and Data alignment in CR2 
	ADC1->CR2 |= (1<<1); // Continuous Conversion enable
	ADC1->CR2 |= (1<<10); // EOC after each conversion 
	ADC1->CR2 |= (0<<11); // Data right alignment 
	
	// 5. Set the Sampling time for the channels in ADC_SMPRx
	ADC1->SMPR2 |= (2<<0)|(2<<3)|(2<<6)|(2<<9);
	
	// 6. Set the Regular channel sequence length in ADC_SQRx
	ADC1->SQR1 |= (3<<20); // Number of conversion 4
	
	// 7. Set the Respective GPIO PINs in the analog mode 
	GPIOA->MODER |= (3<<0)|(3<<2)|(3<<4)|(3<<6);           // GPIOA Pin 0,1,2,3 Analog mode

}

void ADC_Enable()
{
	// 1. Enable to ADC ADON bit in CR2
	ADC1->CR2 |= 1<<0;  // ADON=1 enable ADC
	
	// Wait for ADC stabilize aroun 10 us
	uint32_t delay = 10000;
	while(delay--);
	
}

void ADC_Start(int channel)
{
	// 1. Set the channel sequence in the SQR register
	ADC1->SQR3 = 0;
	ADC1->SQR3 |= (channel<<0);

  // 2. Clear the Status register
	ADC1->SR = 0;  
	
	// 3. Start to conversion by setting the SWSTART bit in CR2
	ADC1->CR2 |= (1<<30);
}

void ADC_WaitForConversion()
{
	// EOC flag will be set, once the conversion is finished
	while(!(ADC1->SR & (1<<1)));
	
}

uint16_t ADC_GetValue(void)
{
	return ADC1->DR;
}

void ADC_Disable()
{
	ADC1->CR2 &= ~(1<<0); // Disable ADC
}

uint16_t ADC_Value[4]={0,0,0,0};


int main (void)
{
	
	Clck_Config();
	ADC_Config();
	ADC_Enable();
	
	while(1)
	{
		ADC_Start(0);
		ADC_WaitForConversion();
		ADC_Value[0] = ADC_GetValue();
		
		ADC_Start(1);
		ADC_WaitForConversion();
		ADC_Value[1] = ADC_GetValue();
		
		ADC_Start(2);
		ADC_WaitForConversion();
		ADC_Value[2] = ADC_GetValue();
		
		ADC_Start(3);
		ADC_WaitForConversion();
		ADC_Value[3] = ADC_GetValue();
		
	}
	
}
