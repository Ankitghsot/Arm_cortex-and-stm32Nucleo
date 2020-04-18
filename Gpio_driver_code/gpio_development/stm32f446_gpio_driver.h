/*
 *   stm32f446_gpio_driver.h
 *
 *  Created on: Mar 7, 2020
 *  Author: Ankit maurya
 */
#include "stm32f446.h"   //driver header file must always contain mcu specific header file

#ifndef INC_STM32F446_GPIO_DRIVER_H_
#define INC_STM32F446_GPIO_DRIVER_H_

typedef struct
{ uint8_t GPIO_PinNumber;
 uint8_t GPIO_PinMode;                          //POSSIBLE VALUES ARE GIVEN BELOW  @FOR GPIO INIT MACROS FOR ALL
 uint8_t GPIO_PinSpeed;
 uint8_t GPIO_PinPuPdControl;
 uint8_t GPIO_PinOPType;                      //0 push pull ,1 open drain
 uint8_t GPIO_PinAltFunctionMode;
}GPIO_PinConfig_t;

/*handler structure of a gpio pin*/

typedef struct
{
	//pointer to hold the address of gpio peripheral
	GPIOregdef *pGPIOx;  //THIS HOLDS THE ADDRESS OF GPIO PORT TO WHICH PIN BELONGS
	GPIO_PinConfig_t GPIO_PinConfig;  //this holds config settings

}GPIO_HANDLE_t;
/*@FOR GPIO INIT MACROS*********************************************************************************************************************/

/*GPIO PIN NUMBER *********************************************/
#define GPIO_PIN_NO_0      0
#define GPIO_PIN_NO_1      1
#define GPIO_PIN_NO_2      2
#define GPIO_PIN_NO_3      3
#define GPIO_PIN_NO_4      4
#define GPIO_PIN_NO_5      5
#define GPIO_PIN_NO_6      6
#define GPIO_PIN_NO_7      7
#define GPIO_PIN_NO_8      8
#define GPIO_PIN_NO_9      9
#define GPIO_PIN_NO_10     10
#define GPIO_PIN_NO_11     11
#define GPIO_PIN_NO_12     12
#define GPIO_PIN_NO_13     13
#define GPIO_PIN_NO_14     14
#define GPIO_PIN_NO_15     15
/*gpio pin possible modes macros ********** *********************/
#define GPIO_MODE_IN       0
#define GPIO_MODE_OUT      1
#define GPIO_MODE_ALTFN    2     //2==10
#define GPIO_MODE_ANALOG   3    //3=11
#define GPIO_MODE_IT_FT    4     //INTERUPT FOR FALLING EDGE TRIGGERED
#define GPIO_MODE_IT_RT    5
#define GPIO_MODE_IT_FRT    6    //INTRRUPT FOR FALLING AND RISING EDGE TRIGGERED

/*GPIO PIN POSSIBLE OUTPUT TYPES***********************************/
#define GPIO_OP_TYPE_PP    0       //PUSHPULL
#define GPIO_OP_TYPE_OD    1        //OPEN DRAIN

/*GPIO PIN OUTPUT SPEED TYPES*************************************/
#define GPIO_SPEED_LOW     0
#define GPIO_SPEED_MEDIUM   1
#define GPIO_SPEED_FAST     2
#define GPIO_SPEED_HIGH     3

/*GPIO PIN PULL UP NAD PULL DOWN RESISTOR CONFIGURATION MACROS*************/
#define GPIO_NO_PUPD        0
#define GPIO_PU             1
#define GPIO_PD             2






//designing gpio prototype
void GPIO_Init(GPIO_HANDLE_t *pGPIOHandle);  //initialises gpio port
void GPIO_DeInit(GPIOregdef *pGPIOx);       /* deinitialises gpio port */

void GPIO_PeriClockControl(GPIOregdef *pGPIOx,uint8_t EnOrDi); //enordi is clock enble or disable

uint8_t GPIO_ReadFromInputPin(GPIOregdef *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIOregdef *pGPIOx);
void GPIO_WriteToOutputPin(GPIOregdef *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIOregdef *pGPIOx,uint8_t Value);
void GPIO_ToggleOutputPin(GPIOregdef *pGPIOx,uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber); //it handles when interrupt occours to process that interupt
void IRQPrioity_Config  (uint8_t IRQNumber,uint32_t IRQPriority);






#endif /* INC_STM32F446_GPIO_DRIVER_H_ */
