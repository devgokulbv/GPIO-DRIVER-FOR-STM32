/*
 * STM32F407xx.h
 *
 *  Created on: 08-Mar-2023
 *      Author: DEVGOKUL
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_
#include<stdint.h>


/* Base addresses of flash memory and SRAM */
#define FLASH_BASE                 0x08000000UL //BASE ADDRESS OF FLASH MEMORY
#define SRAM1_BASE                 0x20000000UL //BASE ADDRESS OF SRAM1
#define SRAM2_BASE                 0x2001C000UL //BASE ADDRESS OF SRAM2
#define ROM_BASE                   0x1FFF0000UL //BASE ADDRESS OF SYSTEM MEMORY OR ROM
/* End of base address of memory */

/* BASE ADDRESSES OF AHBx AND APBx PERIPHERALS */
#define PERIPHERAL_BASE            0x40000000UL    // BASE ADDRESS OF ALL PERIPHERALS
#define APB1_BASE                  PERIPHERAL_BASE // BASE ADDRESS OF APB1 BUS
#define APB2_BASE                  0x40010000UL    // BASE OF APB2 BUS
#define AHB1_BASE                  0x40020000UL    // BASE OF AHB1 BUS
#define AHB2_BASE                  0x50000000UL    // BASE OF AHB2 BUS
#define RCC_BASE                   0x40023800UL
/*End of base address of bus */

/* AHB1 BUS PERIPHERAL BASE ADDRESS */
#define GPIOA_BASE                 0x40020000UL
#define GPIOB_BASE                 0x40020400UL
#define GPIOC_BASE                 0x40020800UL
#define GPIOD_BASE                 0x40020C00UL
#define GPIOE_BASE                 0x40021000UL
#define GPIOF_BASE                 0x40021400UL
#define GPIOG_BASE                 0x40021800UL
#define GPIOH_BASE                 0x40021C00UL
#define GPIOI_BASE                 0x40022000UL
/* finish */

/* APB1 BUS PERIPHERAL REGISTER */
#define I2C3_BASE                  0x40005C00UL
#define I2C2_BASE                  0x40005800UL
#define I2C1_BASE                  0x40005400UL
#define UART5_BASE                 0x40005000UL
#define UART4_BASE                 0x40004C00UL
#define USART3_BASE                0x40004800UL
#define USART2_BASE                0x40004400UL
#define SPI2_BASE                  0x40003800UL
#define SPI3_BASE                  0x40003C00UL
/* end */

/* APB2 BUS PERIPHERAl REGISTER */
#define EXTI_BASE                  0x40013C00UL
#define USART1_BASE                0x40011000UL
#define USART6_BASE                0x40011400UL
#define SPI1_BASE                  0x40013000UL
#define SYSCFG_BASE                0x40013800UL
/* end */

/**********************peripheral register definition structure*****************/
typedef struct
{
	volatile uint32_t MODER;     //ADDRESS OFFSET 0x00
	volatile uint32_t OTYPER;    //ADDRESS OFFSET 0x04
	volatile uint32_t OSPEEDR;   //ADDRESS OFFSET 0x08
	volatile uint32_t PUPDR;     //ADDRESS OFFSET 0x0C
	volatile uint32_t IDR;       //ADDRESS OFFSET 0x10
	volatile uint32_t ODR;       //ADDRESS OFFSET 0x14
	volatile uint32_t BSRR;      //ADDRESS OFFSET 0x18
	volatile uint32_t LCKR;      //ADDRESS OFFSET 0x1C
	volatile uint32_t AFR[2];      //ADDRESS OFFSET 0x20 & AFRH-0x24


}GPIO_REGDEF_X;

/* peripheral base address type caste to GPIO_REGDEF_X */
#define GPIOA ((GPIO_REGDEF_X*)GPIOA_BASE)
#define GPIOB ((GPIO_REGDEF_X*)GPIOB_BASE)
#define GPIOC ((GPIO_REGDEF_X*)GPIOC_BASE)
#define GPIOD ((GPIO_REGDEF_X*)GPIOD_BASE)
#define GPIOE ((GPIO_REGDEF_X*)GPIOE_BASE)
#define GPIOF ((GPIO_REGDEF_X*)GPIOF_BASE)
#define GPIOG ((GPIO_REGDEF_X*)GPIOG_BASE)
#define GPIOH ((GPIO_REGDEF_X*)GPIOH_BASE)
#define GPIOI ((GPIO_REGDEF_X*)GPIOI_BASE)
#define RCC   ((RCC_REGDEF_X*)RCC_BASE)  //RCC pointer
/* end */
/*RCC PERIPHERAL REGISTERS STRUCTURE */
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
	uint32_t RESERVED1;
	uint32_t RESERVED2;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED3;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED4;
	uint32_t RESERVED5;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED6;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED7;
	uint32_t RESERVED8;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED9;
	uint32_t RESERVED10;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
}RCC_REGDEF_X;

/* end of rcc registers */

/* enable clock for gpiox */
#define GPIOA_CLK_EN()   (RCC -> AHB1ENR |= (1 << 0))
#define GPIOB_CLK_EN()   (RCC -> AHB1ENR |= (1 << 1))
#define GPIOC_CLK_EN()   (RCC -> AHB1ENR |= (1 << 2))
#define GPIOD_CLK_EN()   (RCC -> AHB1ENR |= (1 << 3))
#define GPIOE_CLK_EN()   (RCC -> AHB1ENR |= (1 << 4))
#define GPIOF_CLK_EN()   (RCC -> AHB1ENR |= (1 << 5))
#define GPIOG_CLK_EN()   (RCC -> AHB1ENR |= (1 << 6))
#define GPIOH_CLK_EN()   (RCC -> AHB1ENR |= (1 << 7))
#define GPIOI_CLK_EN()   (RCC -> AHB1ENR |= (1 << 8))
//end

/* clock enable macros for I2Cx peripherals */
#define I2C1_CLK_EN()    (RCC -> APB1ENR |= (1 << 21))
#define I2C2_CLK_EN()    (RCC -> APB1ENR |= (1 << 22))
#define I2C3_CLK_EN()    (RCC -> APB1ENR |= (1 << 23))
//end

/*clock enable macros for SPIx peripheral */
#define SPI1_CLK_EN()     (RCC -> APB2ENR |= (1 << 12))
#define SPI2_CLK_EN()     (RCC -> APB1ENR |= (1 << 14))
#define SPI3_CLK_EN()     (RCC -> APB1ENR |= (1 << 15))
//end

 /* clock enable macros for UART AND USART AND SYSCFG*/
#define USART2_CLK_EN()     (RCC -> APB1ENR |= (1 << 17))
#define USART3_CLK_EN()     (RCC -> APB1ENR |= (1 << 18))
#define UART4_CLK_EN()      (RCC -> APB1ENR |= (1 << 19))
#define UART5_CLK_EN()      (RCC -> APB1ENR |= (1 << 20))
#define USART1_CLK_EN()     (RCC -> APB2ENR |= (1 << 4))
#define USART6_CLK_EN()     (RCC -> APB2ENR |= (1 << 5))
#define SYSCFG_CLK_EN()     (RCC -> APB2ENR |= (1 << 14))
//end

/* disable clock for gpiox */
#define GPIOA_CLK_DI()   (RCC -> AHB1ENR &= ~(1 << 0))
#define GPIOB_CLK_DI()   (RCC -> AHB1ENR &= ~(1 << 1))
#define GPIOC_CLK_DI()   (RCC -> AHB1ENR &= ~(1 << 2))
#define GPIOD_CLK_DI()   (RCC -> AHB1ENR &= ~(1 << 3))
#define GPIOE_CLK_DI()   (RCC -> AHB1ENR &= ~(1 << 4))
#define GPIOF_CLK_DI()   (RCC -> AHB1ENR &= ~(1 << 5))
#define GPIOG_CLK_DI()   (RCC -> AHB1ENR &= ~(1 << 6))
#define GPIOH_CLK_DI()   (RCC -> AHB1ENR &= ~(1 << 7))
#define GPIOI_CLK_DI()   (RCC -> AHB1ENR &= ~(1 << 8))
//end

/* clock disable macros for I2Cx peripherals */
#define I2C1_CLK_DI()    (RCC -> APB1ENR &= ~(1 << 21))
#define I2C2_CLK_DI()    (RCC -> APB1ENR &= ~(1 << 22))
#define I2C3_CLK_DI()    (RCC -> APB1ENR &= ~(1 << 23))
//end

/*clock disable macros for SPIx peripheral */
#define SPI1_CLK_DI()     (RCC -> APB2ENR &= ~(1 << 12))
#define SPI2_CLK_DI()     (RCC -> APB1ENR &= ~(1 << 14))
#define SPI3_CLK_DI()     (RCC -> APB1ENR &= ~(1 << 15))
//end

 /* clock diable macros for UART AND USART AND SYSCFG*/
#define USART2_CLK_DI()     (RCC -> APB1ENR &= ~(1 << 17))
#define USART3_CLK_DI()     (RCC -> APB1ENR &= ~(1 << 18))
#define UART4_CLK_DI()      (RCC -> APB1ENR &= ~(1 << 19))
#define UART5_CLK_DI()      (RCC -> APB1ENR &= ~(1 << 20))
#define USART1_CLK_DI()     (RCC -> APB2ENR &= ~(1 << 4))
#define USART6_CLK_DI()     (RCC -> APB2ENR &= ~(1 << 5))
#define SYSCFG_CLK_DI()     (RCC -> APB2ENR &= ~(1 << 14))
//end

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

/*MACROS TO DISABLE REGISTERS OR RESET */
#define GPIOA_REG_RESET()   do{(RCC -> AHB1RSTR |= (1 << 0));(RCC -> AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()   do{(RCC -> AHB1RSTR |= (1 << 1));(RCC -> AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()   do{(RCC -> AHB1RSTR |= (1 << 2));(RCC -> AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()   do{(RCC -> AHB1RSTR |= (1 << 3));(RCC -> AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()   do{(RCC -> AHB1RSTR |= (1 << 4));(RCC -> AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()   do{(RCC -> AHB1RSTR |= (1 << 5));(RCC -> AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()   do{(RCC -> AHB1RSTR |= (1 << 6));(RCC -> AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()   do{(RCC -> AHB1RSTR |= (1 << 7));(RCC -> AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()   do{(RCC -> AHB1RSTR |= (1 << 8));(RCC -> AHB1RSTR &= ~(1 << 8));}while(0)

#include "STM32F407xx_GPIO_DRIVER.h"
#endif /* ST32F407XX_H_ */

