/*
 *  001ledtoggle.c
 *
 *  Created on: Mar 8, 2020
 *  Author: Ankit maurya
 */
#include "stm32f446.h"
#include "stm32f446_gpio_driver.h"

void delay(void)
{
	for(uint32_t i=0;i<500000;i++);
	}

int main(void)
{
	 GPIO_HANDLE_t Gpioled;
	 Gpioled.pGPIOx=GPIOA;
	 Gpioled.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_5;    //These data are updated to structure GPIOpinconfig
	 Gpioled.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	 Gpioled.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	 Gpioled.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	 Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PU;

	 GPIO_PeriClockControl(GPIOA, ENABLE);
	 GPIO_Init(&Gpioled);  //THID FUNCTION WILL INITIALISES ALL REGISTER WITH VALUE WE HAVE GIVEN IN GPIOCONFIG STRUCTURE.
	 while(1)
	 {
		 GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		 delay();
	 }



	return 0;
}

