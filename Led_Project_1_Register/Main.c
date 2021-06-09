#include "stm32f4xx.h"                  // Device header



void GPIO_Cnfg() 
{
	RCC->AHB1ENR |=0x00000001;                  // AHB1 = 1 Enable
	GPIOA->MODER = 0x00000400;                 // Mod 4 farkli cesit. Biz General Purpose output sectik.
	GPIOA->OTYPER =0x00000000;                 // Push pull yada open drain olarak 2 secenek var. Push pull secildi.
	GPIOA->OSPEEDR =0x00000C00;                // Low Medium HighSpeed Very High speed 4 cesit. Very High Speed secildi. 
	GPIOA->PUPDR = 0x00000000;                 // No pull up No pull down secildi.
}	


void delay (uint32_t time)
{
  while (time--); 
	
}	
	int main ()
{
   
	 GPIO_Cnfg();
	
	while(1)
		{
			GPIOA->ODR = 0x00000020;        
		}
}	


