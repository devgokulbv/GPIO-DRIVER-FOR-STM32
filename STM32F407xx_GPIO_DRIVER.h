/*
 * STM32F407xx_GPIO_DRIVER.h
 *
 *  Created on: 11-Mar-2023
 *      Author: hp
 */

#ifndef STM32F407XX_GPIO_DRIVER_H_
#define STM32F407XX_GPIO_DRIVER_H_
#include "STM32F407xx.h"

typedef struct
{
	uint8_t GPIO_PIN_NAME;
	uint8_t GPIO_PIN_NUMBER;
	uint8_t GPIO_PIN_MODE;
	uint8_t GPIO_PIN_SPEED;
	uint8_t GPIO_PIN_PUPDR;
	uint8_t GPIO_PIN_OTYPE;
	uint8_t GPIO_PIN_AF;
}GPIO_PINCONFIG_X;

typedef struct
{
	GPIO_REGDEF_X *pGPIOx;
	GPIO_PINCONFIG_X GPIO_PIN_CONFIG;
}GPIO_HANDLE_X;

/**************APIs supported by this driver**************/

void GPIO_Init(GPIO_HANDLE_X *pGPIOHandle);
void GPIO_DeInit(GPIO_REGDEF_X *pGPIOx);
void GPIO_PeriClockControl(GPIO_REGDEF_X *pGPIOx,uint8_t EnorDi);
void GPIO_WriteToOutputPin(GPIO_REGDEF_X *pGPIOx,uint8_t PinNumber,uint8_t value);
void GPIO_WriteToOutputPort(GPIO_REGDEF_X *pGPIOx,uint16_t value);
uint8_t GPIO_ReadFromInputPin(GPIO_REGDEF_X *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_REGDEF_X *pGPIOx);
void GPIO_ToggleOutputPin(GPIO_REGDEF_X *pGPIOx,uint8_t PinNumber);
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void delay(void);
//GPIO PIN NUMBER
#define GPIO_PIN_0 0
#define GPIO_PIN_1 1
#define GPIO_PIN_2 2
#define GPIO_PIN_3 3
#define GPIO_PIN_4 4
#define GPIO_PIN_5 5
#define GPIO_PIN_6 6
#define GPIO_PIN_7 7
#define GPIO_PIN_8 8
#define GPIO_PIN_9 9
#define GPIO_PIN_10 10
#define GPIO_PIN_11 11
#define GPIO_PIN_12 12
#define GPIO_PIN_13 13
#define GPIO_PIN_14 14
#define GPIO_PIN_15 15



//GPIO MODE TYPES

#define GPIO_MODE_IN      0
#define GPIO_MODE_OUT     1
#define GPIO_MODE_ALTFN   2
#define GPIO_MODE_ANALOG  3
#define GPIO_MODE_IT_FT   4
#define GPIO_MODE_IT_RT   5
#define GPIO_MODE_IT_RFT  6

//GPIO OUTPUT TYPES
#define GPIO_OP_PP 0
#define GPIO_OP_OD 1

//GPIO SPEED TYPES
#define GPIO_SPEED_LS 0
#define GPIO_SPEED_MS 1
#define GPIO_SPEED_HS 2
#define GPIO_SPEED_VHS 3

//GPIO PIN PULL UP PULL DOWN MACROS

#define GPIO_PUPDR_NO 0
#define GPIO_PUPDR_PU 1
#define GPIO_PUPDR_PD 2















#endif /* STM32F407XX_GPIO_DRIVER_H_ */
