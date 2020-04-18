/*
 * 002ledwithbutton.c
 *
 *  Created on: Mar 12, 2020
 *      Author: Ankit maurya
 */
#include "stm32f446.h"
#include "stm32f446_gpio_driver.h"



int main(void)
{
	     GPIO_HANDLE_t Gpiobutton;
		 Gpiobutton.pGPIOx=GPIOC;
		 Gpiobutton.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_13;    //These data are updated to structure GPIOpinconfig
		 Gpiobutton.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;         //MODE IS INTERRUPT FALLING EDGE
		 Gpiobutton.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
		// Gpiobutton.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
		 Gpiobutton.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PU;
		 GPIO_PeriClockControl(GPIOC, ENABLE);
		 GPIO_Init(&Gpiobutton);
		 GPIO_IRQConfig(IRQ_NO_EXTI10_15,ENABLE);





	 GPIO_HANDLE_t Gpioled;
	 Gpioled.pGPIOx=GPIOA;
	 Gpioled.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_5;    //These data are updated to structure GPIOpinconfig
	 Gpioled.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	 Gpioled.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	 Gpioled.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	 Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	 GPIO_PeriClockControl(GPIOA, ENABLE);
	 GPIO_Init(&Gpioled);  //THID FUNCTION WILL INITIALISES ALL REGISTER WITH VALUE WE HAVE GIVEN IN GPIOCONFIG STRUCTURE.


      while(1);

	return 0;
}
//HERE WE DEFINE FUNCTION WHICH IS PRESENT IN startup_stm32f446retx.s
//since our button interrupt pin is 13 we used 15_10 handler
void EXTI15_10_IRQHandler(void)
    {
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
	}

