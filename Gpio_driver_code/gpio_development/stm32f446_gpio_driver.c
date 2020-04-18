/*
 * stm32f446.c
 *
 *  Created on: Mar 7, 2020
 *   Author: Ankit maurya
 */


#include "stm32f446_gpio_driver.h"   //driver source file must always contains driver header file

/**********************************************************************************************************
 *@fn                 -GPIO_Init
 @brief               -this function initialises gpio port

 @param[in]           - base address and pin configure structure declared in header
 @param[in]           -
 @param[in]           -

 @return              -none
 @note                -none
 */

void GPIO_Init(GPIO_HANDLE_t *pGPIOHandle)
{
	//CONFIGURE THE mode of a gpio pin
    uint32_t temp=0;
	if(pGPIOHandle-> GPIO_PinConfig.GPIO_PinMode  <= GPIO_MODE_ANALOG)  //IF MODE LESS THAN 4 IT IS NON INTERRUPT MODE
	{                                                                  //
		temp =(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));            //so suppose user has given 01 for pin no.4
		pGPIOHandle->pGPIOx->MODER &=~(0x3 <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));//clear that section bit
		pGPIOHandle->pGPIOx->MODER |=temp;                                                                            //then pin mode value 01 left shifted by 2*4=8
	} else                                                                                                           //and stored at 4th section ,see gpio port mode register in reference manual
	{
		//this part will be coded for interrupt
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //reset rtsr so that only falling edge activated
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FRT)
		{
			EXTI->FTSR |=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//CONFIGURE THE GPIO PORT SELECTION IN SYSCFG_EXITCR
		uint8_t temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /4;
		uint8_t temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %4;
		uint8_t portcode=GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1]=portcode<<(temp2*4);


		//CONFIGURING THE EXTI INTERRUPT DELIVERY USING IMR
		EXTI->IMR |=1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}
	// configure the speed
	temp=0;
	temp =(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->OSPEEDER &=~(0x3 <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER |=temp;

	//configure the pupd settings
	temp=0;
	temp =(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->PUPDR &=~(0x3 <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx-> PUPDR |=temp;


	//configure the output type
    temp=0;
    temp =(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
    pGPIOHandle->pGPIOx->OTYPER &=~(0x1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER |=temp;

	//configure the alternate functionality mode
    if(pGPIOHandle-> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN )
       { uint32_t temp1,temp2;
         temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /8;
         temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinMode %8;
         pGPIOHandle->pGPIOx-> AFR[temp1] &=~(0xF<<(4*temp2));
         pGPIOHandle->pGPIOx-> AFR[temp1] |=(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunctionMode<<(4*temp2));

       }
	}

/**********************************************************************************************************
 *@fn                 -GPIO_DeInit
 @brief               -this function Deinitialises gpio port

 @param[in]           - base address of gpiox
 @param[in]           -
 @param[in]           -

 @return              -none
 @note                -none
 */
void GPIO_DeInit(GPIOregdef *pGPIOx)/* deinitialises gpio port */
{
	 //for reset we have to go to rcc which is used for reset and clock control
	if(pGPIOx == GPIOA)
	       {
		    GPIOA_REG_RESET();
	       }
	   else if(pGPIOx==GPIOB)
	       {
		   GPIOB_REG_RESET();
	       }
	   else if(pGPIOx==GPIOC)
	       {
		   GPIOC_REG_RESET();
	       }
	   else if(pGPIOx==GPIOD)
	       {
		   GPIOD_REG_RESET();
	       }
	   else if(pGPIOx==GPIOE)
	       {
		   GPIOE_REG_RESET();
	       }
	   else if(pGPIOx==GPIOG)
	          {
		   GPIOG_REG_RESET();
	          }
	   else if(pGPIOx==GPIOH)
	          {
		   GPIOH_REG_RESET();
	          }

	   else if ( pGPIOx == GPIOF))  //DONT KNOW THE REASON BUT ERROR WHEN ) REMOVED
	         {
	        	 GPIOF_REG_RESET();
	         }


}
/**********************************************************************************************************
 *@fn                 -GPIO_PeriClockControl
 @brief               -this function enable or disable peripheral clock for given gpio port

 @param[in]           - base address of gpio peripheral
 @param[in]           - enable or diable macros
 @param[in]           -

 @return              -none
 */
void GPIO_PeriClockControl(GPIOregdef *pGPIOx,uint8_t EnOrDi)//enordi is clock enble or disable
{
if(EnOrDi==ENABLE)
 {  if(pGPIOx == GPIOA)
       {
	      GPIOA_PCLK_EN();
       }
   else if(pGPIOx==GPIOB)
       {
	   GPIOB_PCLK_EN();
       }
   else if(pGPIOx==GPIOC)
       {
   	   GPIOC_PCLK_EN();
       }
   else if(pGPIOx==GPIOD)
       {
   	   GPIOD_PCLK_EN();
       }
   else if(pGPIOx==GPIOE)
       {
   	   GPIOE_PCLK_EN();
       }
   else if(pGPIOx==GPIOG)
          {
   	   GPIOG_PCLK_EN();
          }
   else if(pGPIOx==GPIOH)
          {
   	   GPIOH_PCLK_EN();
          }

   else if ( pGPIOx == GPIOF))  //DONT KNOW THE REASON BUT ERROR WHEN ) REMOVED
         {
     	   GPIOF_PCLK_EN();
         }

 }
else{

	   if(pGPIOx == GPIOA)
	       {
		      GPIOA_PCLK_DI();
	       }
	   else if(pGPIOx==GPIOB)
	       {
		   GPIOB_PCLK_DI();
	       }
	   else if(pGPIOx==GPIOC)
	       {
	   	   GPIOC_PCLK_DI();
	       }
	   else if(pGPIOx==GPIOD)
	       {
	   	   GPIOD_PCLK_DI();
	       }
	   else if(pGPIOx==GPIOE)
	       {
	   	   GPIOE_PCLK_DI();
	       }
	   else if(pGPIOx==GPIOG)
	          {
	   	   GPIOG_PCLK_DI();
	          }
	   else if(pGPIOx==GPIOH)
	          {
	   	   GPIOH_PCLK_DI();
	          }

	   else if ( pGPIOx == GPIOF))     //DONT KNOW THE REASON BUT ERROR WHEN ) REMOVED
	         {
	     	   GPIOF_PCLK_DI();
	         }

    }


}

/**********************************************************************************************************
 *@fn                 -GPIO_ReadFromInputPin
 @brief               -this function read gpiox pin

 @param[in]           - base address of gpiox
 @param[in]           - pin number of gpiox
 @param[in]           -

 @return              -uint8_t
 @note                -none
 */

uint8_t GPIO_ReadFromInputPin(GPIOregdef *pGPIOx,uint8_t PinNumber)
  {
	uint8_t value;
	value=(uint8_t)((pGPIOx->IDR >>PinNumber)& 0x00000001) ;    //this is done to shift desired bit in last and mask all others bit to 0 and read it
	return value;
  }

/**********************************************************************************************************
 *@fn                 -GPIO_ReadFromInputPort
 @brief               -this function read gpiox port

 @param[in]           - base address of gpiox
 @param[in]           -
 @param[in]           -

 @return              -uint16_t
 @note                -none
 */

uint16_t GPIO_ReadFromInputPort(GPIOregdef *pGPIOx)
{
	uint16_t value;
	value=(uint16_t)pGPIOx->IDR;
	return value;
	}
/**********************************************************************************************************
 *@fn                 -GPIO_WriteToOutputPin
 @brief               -this function write to gpio pin

 @param[in]           - base address
 @param[in]           - pin number
 @param[in]           - and value to write

 @return              -none
 @note                -none
 */
void GPIO_WriteToOutputPin(GPIOregdef *pGPIOx,uint8_t PinNumber,uint8_t Value)
{   if(Value==GPIO_PIN_SET)
     {
	   pGPIOx->ODR |=(1<<PinNumber);
     }
   else
     {
	   pGPIOx->ODR &=~(1<<PinNumber);
     }

}
/**********************************************************************************************************
 *@fn                 -GPIO_WriteToOutputPort
 @brief               -this function write to gpio port

 @param[in]           - base address of gpiox
 @param[in]           - value to write in base address
 @param[in]           -

 @return              -none
 @note                -none
 */
void GPIO_WriteToOutputPort(GPIOregdef *pGPIOx,uint8_t Value)
{   pGPIOx->ODR=Value;
	}
/**********************************************************************************************************
 *@fn                 -GPIO_ToggleOutputPin
 @brief               -this function toggles output pin

 @param[in]           - base address of gpiox
 @param[in]           - pin no. that we have to toggle
 @param[in]           -

 @return              -none
 @note                -none
 */
void GPIO_ToggleOutputPin(GPIOregdef *pGPIOx,uint8_t PinNumber)
{
   pGPIOx->ODR=pGPIOx->ODR ^ (1<<PinNumber) ;
}
/**********************************************************************************************************
 *@fn                 -GPIO_IRQConfig
 @brief               -this function configures interrupts

 @param[in]           - interrupt request no.
 @param[in]           - interrupt priority
 @param[in]           - enable or disable

 @return              -none
 @note                -none
 */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t EnOrDi)
{
	 if(EnOrDi==ENABLE)
	   {   if(IRQNumber<=31)
	      {//program iser0 register
		    *NVIC_ISER0  |=(1<<IRQNumber);
	      }else if(IRQNumber>31 && IRQNumber<64)
	      {//program iser1 register
	    	  *NVIC_ISER1  |=(1<<(IRQNumber%32));
	      }else if(IRQNumber>=64 && IRQNumber<96)
	      { //program iser2 register
	    	  *NVIC_ISER2  |=(1<<(IRQNumber%64));
	      }

	  }else

		           if(IRQNumber<=31)
		   	      { //program icer0 register
		        	   *NVIC_ICER0  |=(1<<IRQNumber);
		   	      }else if(IRQNumber>31 && IRQNumber<64)
		   	      {  //program icer1 register
		   	    	*NVIC_ICER1  |=(1<<IRQNumber%32);
		   	      }else if(IRQNumber>=64 && IRQNumber<96)
		   	      {  //program icer2 register
		   	    	*NVIC_ICER2  |=(1<<IRQNumber%64);

		   	      }



	}
/**********************************************************************************************************
 *@fn                 -IRQPriority_config
 @brief               -this function sets priority

 @param[in]           - only priority rqd
 @param[in]           -
 @param[in]           -

 @return              -none
 @note                -none
 */

void IRQPrioity_Config  (uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx=IRQNumber/4;
	uint8_t iprx_section=IRQNumber%4;
	uint8_t shift= (8*iprx_section) +(8- NO_OF_PRIORITYBITS_IMPLEMENTED);
	*(NVIC_PR + (iprx))|=(IRQPriority<<shift);

	}

/**********************************************************************************************************
 *@fn                 -GPIO_IRQHandling
 @brief               -this function handles interrupt when it occurs

 @param[in]           - only pin no. required
 @param[in]           -
 @param[in]           -

 @return              -none
 @note                -none
 */
void GPIO_IRQHandling(uint8_t PinNumber)//it handles when interrupt occours to process that interupt
{
	//CLEAR THE EXTI PR REGISTER CORRESPONDING TO THAT PIN NUMBER
	if(EXTI->PR & 1<<PinNumber)
	{   //CLEAR BY WRITING 1
		EXTI->PR |=(1<<PinNumber);
	}


	}
