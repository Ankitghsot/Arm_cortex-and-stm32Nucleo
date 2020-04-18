/*
 * stm32f466.h
 *
 *  Created on: Mar 6, 2020
 *  Author: Ankit maurya
 */
#include <stdint.h> // for uint32_t
#ifndef INC_STM32F446_H_
#define INC_STM32F446_H_



/**********************************processor specific details ******************/
//arm coertex m4 specific nvic iserx register addressses
#define NVIC_ISER0           (volatile uint32_t*)  0xE000E100
#define NVIC_ISER1            (volatile uint32_t*) 0xE000E104U
#define NVIC_ISER2             (volatile uint32_t*) 0xE000E108U
#define NVIC_ISER3             (volatile uint32_t*) 0xE000E10CU

//ICERX REGISTERS
#define NVIC_ICER0             (volatile uint32_t*) 0xE000E180U
#define NVIC_ICER1             (volatile uint32_t*) 0xE000E184U
#define NVIC_ICER2              (volatile uint32_t*)0xE000E188U
#define NVIC_ICER3              (volatile uint32_t*)0xE000E18CU

//priority registers
#define NVIC_PR                (volatile uint32_t*) 0xE000E400U



#define FLASH_BASEADDR         0x08000000U            //MEMORY BASE ADDRESS
#define SRAM1_BASEADDR         0x20000000U
#define SRAM2_BASEADDR         0x2001C000U
#define ROM_BASEADDR           0x1FFF0000U
#define SRAM                   SRAM1_BASEADDR

#define PERIPH_BASEADDR         0x40000000U          //PERIPHERALS BASE ADDRESS
#define APB1PERIPH_BASEADDR     PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR     0x40010000U
#define AHB1PERIPH_BASEADDR     0x40020000U
#define AHB2PERIPH_BASEADDR     0x50000000U

#define GPIOA_BASEADDR         (AHB1PERIPH_BASEADDR + 0x0000)    //BASE ADDRESSES PF PERIPHERALS HANGING ON AHB1 BUS
#define GPIOB_BASEADDR         (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR         (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR         (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR         (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR         (AHB1PERIPH_BASEADDR + 0x1400
#define GPIOG_BASEADDR         (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR         (AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR            (AHB1PERIPH_BASEADDR+ 0x3800)

#define I2C1_BASEADDR          (APB1PERIPH_BASEADDR +  0x5400)     //BASE ADDR OF PERIPH HANGING ON APB1 BUS
#define I2C2_BASEADDR          (APB1PERIPH_BASEADDR +  0x5800)
#define I2C3_BASEADDR          (APB1PERIPH_BASEADDR +  0x5C00)
#define SPI3_BASEADDR          (APB1PERIPH_BASEADDR +  0x3C00)
#define SPI2_BASEADDR          (APB1PERIPH_BASEADDR +  0x3800)
#define USART2_BASEADDR          (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR          (APB1PERIPH_BASEADDR +  0x4800)
#define UART4_BASEADDR          (APB1PERIPH_BASEADDR +  0x4COO)
#define UART5_BASEADDR          (APB1PERIPH_BASEADDR +  0x5000)

#define EXTI_BASEADDR          (APB2PERIPH_BASEADDR +  0x3C00)      //APB2 BUS
#define USART1_BASEADDR          (APB2PERIPH_BASEADDR +  0x1000)
#define USART6_BASEADDR          (APB2PERIPH_BASEADDR +  0x1400)
#define SPI1_BASEADDR          (APB2PERIPH_BASEADDR +  0x3000)
#define SYSCFG_BASEADDR          (APB2PERIPH_BASEADDR +  0x3800)


// instead of defining all gpios features seperately that is for gpioa,gpiob,..... we are
//creating a structure for all general features of gpios
typedef struct
{
	volatile uint32_t MODER;        // offset of 0*00
	volatile uint32_t OTYPER;       //offset of 0*04
	volatile uint32_t OSPEEDER;
	volatile uint32_t PUPDR;
    volatile uint32_t IDR;          //INPUT DATA REGISTER
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];   // afr[0] alternate function low reg  afr[1] alternate function high register
} GPIOregdef;
  //   GPIOregdef *p=(GPIOregdef *) 0x40000000;one way of doing this p is pointer of type GPIOregdef

#define GPIOA ((GPIOregdef *)GPIOA_BASEADDR)   // OTHER WAY IS THIS
#define GPIOB ((GPIOregdef *)GPIOB_BASEADDR)
#define GPIOC ((GPIOregdef *)GPIOC_BASEADDR)
#define GPIOD ((GPIOregdef *)GPIOD_BASEADDR)
#define GPIOE ((GPIOregdef *)GPIOE_BASEADDR)
#define GPIOF ((GPIOregdef *)GPIOF_BASEADDR)
#define GPIOG ((GPIOregdef *)GPIOG_BASEADDR)
#define GPIOH ((GPIOregdef *)GPIOH_BASEADDR)

//structure for spi
typedef struct
{
	uint8_t CR1;
	uint8_t CR2;
	uint8_t SR;
	uint8_t DR;
	uint8_t CRCPR;
	uint8_t RXCRCR;
	uint8_t TXCRCR;
	uint8_t I2SCFGR;
	uint8_t I2SPR;
}SPIregdef;

#define SPI1  ((SPIregdef*)SPI1_BASEADDR )
#define SPI2  ((SPIregdef*)SPI2_BASEADDR )
#define SPI3  ((SPIregdef*)SPI3_BASEADDR )

//RCC structure
typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
	volatile uint32_t CKGATENR;
	volatile uint32_t DCKCFGR2;


}RCCregdef;
#define RCC   ((RCCregdef*)RCC_BASEADDR)   //this is equal to rcc=rccregdef* adress;
                                             //so this will give value stored at address
typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
    volatile uint32_t SWTER;
	volatile uint32_t PR;

} EXTIregdef;

#define EXTI ((EXTIregdef*)EXTI_BASEADDR)

typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	 uint32_t RESERVED1[2];
	volatile uint32_t CMPCR;
     uint32_t RESERVED2[2];
	volatile uint32_t CFGR;

} SYSCONFIGregdef;
#define SYSCFG ((SYSCONFIGregdef*)SYSCFG_BASEADDR )

//CLOCK ENABLE MACROS FOR GPIOX PERIPHERALS
#define GPIOA_PCLK_EN()    (RCC->AHB1ENR |=(1<<0)) //(*RCC).AHB1ENR=(*RCC_BASEADDR).AHB1ENR
#define GPIOB_PCLK_EN()    (RCC->AHB1ENR |=(1<<1))  //IN SIMPLE TERM   AHB1ENR VALUE GETS UPDATED TO 0th BIT 1;
#define GPIOC_PCLK_EN()    (RCC->AHB1ENR |=(1<<2))
#define GPIOD_PCLK_EN()    (RCC->AHB1ENR |=(1<<3))
#define GPIOE_PCLK_EN()    (RCC->AHB1ENR |=(1<<4))
#define GPIOF_PCLK_EN()    (RCC->AHB1ENR |=(1<<5))
#define GPIOG_PCLK_EN()    (RCC->AHB1ENR |=(1<<6))
#define GPIOH_PCLK_EN()    (RCC->AHB1ENR |=(1<<7))

//CLOCK ENABLE MICROS FOR I2C
#define I2C1_PCLK_EN()    (RCC->APB1ENR |=(1<<21))   //HERE PCLK MEANS PERIPHERAL CLOCK
#define I2C2_PCLK_EN()    (RCC->APB1ENR |=(1<<22))
#define I2C3_PCLK_EN()    (RCC->APB1ENR |=(1<<23))

//CLOCK ENABLE FOR SPIX
#define SPI1_PCLK_EN()    (RCC->APB2ENR |=(1<<12))
#define SPI2_PCLK_EN()    (RCC->APB1ENR |=(1<<14))
#define SPI3_PCLK_EN()    (RCC->APB1ENR |=(1<<15))
#define SPI4_PCLK_EN()    (RCC->APB2ENR |=(1<<13))

//CLOCK ENABLE FOR USART
#define USART1_PCLK_EN()    (RCC->APB2ENR |=(1<<4))
#define USART2_PCLK_EN()    (RCC->APB1ENR |=(1<<17))
#define USART3_PCLK_EN()    (RCC->APB1ENR |=(1<<18))
#define UART4_PCLK_EN()    (RCC->APB1ENR |=(1<<19))
#define UART5_PCLK_EN()    (RCC->APB1ENR |=(1<<20))
#define USART6_PCLK_EN()    (RCC->APB2ENR |=(1<<5))

//CLOCK ENABLE MACRO FOR SYSCFG PERIPHERAL
#define SYSCFG_PCLK_EN()    (RCC->APB2ENR |=(1<<14))

//CLOCK DISABLE MACRO FOR GPIOX PERIPHERALS

#define GPIOA_PCLK_DI()    (RCC->AHB1ENR &=~(1<<0))
#define GPIOB_PCLK_DI()    (RCC->AHB1ENR &=~ (1<<1))
#define GPIOC_PCLK_DI()    (RCC->AHB1ENR &=~(1<<2))
#define GPIOD_PCLK_DI()    (RCC->AHB1ENR &=~(1<<3))
#define GPIOE_PCLK_DI()    (RCC->AHB1ENR &=~(1<<4))
#define GPIOF_PCLK_DI()    (RCC->AHB1ENR &=~(1<<5))
#define GPIOG_PCLK_DI()    (RCC->AHB1ENR &=~(1<<6))
#define GPIOH_PCLK_DI()    (RCC->AHB1ENR &=~(1<<7))

//CLOCK DISABLE FOR I2CX
#define I2C1_PCLK_DI()    (RCC->APB1ENR &=~(1<<21))
#define I2C2_PCLK_DI()    (RCC->APB1ENR &=~(1<<22))
#define I2C3_PCLK_DI()    (RCC->APB1ENR &=~(1<<23))

//CLOCK DISABLE FOR SPIX
#define SPI1_PCLK_DI()    (RCC->APB2ENR &=~(1<<12))
#define SPI2_PCLK_DI()    (RCC->APB1ENR &=~(1<<14))
#define SPI3_PCLK_DI()    (RCC->APB1ENR &=~(1<<15))
#define SPI4_PCLK_DI()    (RCC->APB2ENR &=~(1<<13))

//CLOCK DISABLE  FOR USART
#define USART1_PCLK_DI()    (RCC->APB2ENR &=~(1<<4))
#define USART2_PCLK_DI()    (RCC->APB1ENR &=~(1<<17))
#define USART3_PCLK_DI()    (RCC->APB1ENR &=~(1<<18))
#define UART4_PCLK_DI()    (RCC->APB1ENR &=~(1<<19))
#define UART5_PCLK_DI()    (RCC->APB1ENR &=~(1<<20))
#define USART6_PCLK_DI()    (RCC->APB2ENR &=~(1<<5))

//CLOCK DISABLE FOR SYSCFG
#define SYSCFG_PCLK_DI()    (RCC->APB2ENR &=~(1<<14))

//macros to reset GPIOX peripherals
#define GPIOA_REG_RESET()            do{ (RCC->AHB1RSTR |=(1<<0));  (RCC->AHB1ENR &=~(1<<0)); }while(0)    //WE HAVE TO RESET THAT BIT AND SET IT AGAIN AHB1RSTR BIT OTHERWISE IT WILL RESET IT WHOLE TIME
#define GPIOB_REG_RESET()            do{ (RCC->AHB1RSTR |=(1<<1));  (RCC->AHB1ENR &=~(1<<1)); }while(0)    //AND do while is a technique in c programming to execute multiple c statements using single c macros
#define GPIOC_REG_RESET()            do{ (RCC->AHB1RSTR |=(1<<2));  (RCC->AHB1ENR &=~(1<<2)); }while(0)    //and the condition is zero so loop is executed only 1 time
#define GPIOD_REG_RESET()            do{ (RCC->AHB1RSTR |=(1<<3));  (RCC->AHB1ENR &=~(1<<3)); }while(0)
#define GPIOE_REG_RESET()            do{ (RCC->AHB1RSTR |=(1<<4));  (RCC->AHB1ENR &=~(1<<4)); }while(0)
#define GPIOF_REG_RESET()            do{ (RCC->AHB1RSTR |=(1<<5));  (RCC->AHB1ENR &=~(1<<5)); }while(0)
#define GPIOG_REG_RESET()            do{ (RCC->AHB1RSTR |=(1<<6));  (RCC->AHB1ENR &=~(1<<6)); }while(0)
#define GPIOH_REG_RESET()            do{ (RCC->AHB1RSTR |=(1<<7));  (RCC->AHB1ENR &=~(1<<7)); }while(0)
//this is used for detecting sysconfig extr in driver c file
#define GPIO_BASEADDR_TO_CODE(x)         ((x==GPIOA)?0:\
		                                  (x==GPIOB)?1:\
                                          (x==GPIOC)?2:\
                                          (x==GPIOD)?3:\
                                          (x==GPIOE)?4:0)

//IRQ no of NVIC
#define IRQ_NO_EXTI0    6
#define IRQ_NO_EXTI1    7
#define IRQ_NO_EXTI2    8
#define IRQ_NO_EXTI3    9
#define IRQ_NO_EXTI4    10
#define IRQ_NO_EXTI5_9    23
#define IRQ_NO_EXTI10_15    40


//some generic macros
#define ENABLE               1
#define DISABLE              0
#define SET                  ENABLE
#define RESET                DISABLE
#define GPIO_PIN_SET         SET
#define GPIO_PIN_RESET       RESET
#define NO_OF_PRIORITYBITS_IMPLEMENTED    4

//spi registers macros
//bit position defination of spi_cr1
#define SPI_CR1_CPHA     0
#define SPI_CR1_CPOL     1
#define SPI_CR1_MSTR     2
#define SPI_CR1_BR       3
#define SPI_CR1_SPE      6
#define SPI_CR1_LSBFIRST 7
#define SPI_CR1_SSI      8
#define SPI_CR1_SSM       9
#define SPI_CR1_RXONLY    10
#define SPI_CR1_DFF       11
#define SPI_CR1_CRCNEXT   12
#define SPI_CR1_CRCEN     13
#define SPI_CR1_BIDIOE    14
#define SPI_CR1_BIDIMODE  15

//BIT POSITION DEF OF SPI_CR2
#define SPI_CR2_RXDMAEN      0
#define SPI_CR2_TXDMAEN      1
#define SPI_CR2_SSOE         2
#define SPI_CR2_FRF          4
#define SPI_CR2_ERRIE        5
#define SPI_CR2_RXNEIE       6
#define SPI_CR2_TXEIE        7

//BIT POSITION OD SPI_SR
#define SPI_SR_RXNE     0
#define SPI_SR_TXE      1
#define SPI_SR_CHSIDE   2
#define SPI_SR_UDR      3
#define SPI_SR_CRCERR   4
#define SPI_SR_MODF     5
#define SPI_SR_OVR      6
#define SPI_SR_BSY      7
#define SPI_SR_FRE      8











#endif /* INC_STM32F446_H_ */
