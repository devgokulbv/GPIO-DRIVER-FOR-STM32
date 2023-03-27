/*
 * STM32F407xx_GPIO_DRIVER.c
 *
 *  Created on: 11-Mar-2023
 *      Author: hp
 */
#include "STM32F407xx_GPIO_DRIVER.h"

void GPIO_Init(GPIO_HANDLE_X *pGPIOHandle)
{
	uint32_t temp =0 ;//temp register

	//CONFIGURE GPIO PIN MODES
	if(pGPIOHandle -> GPIO_PIN_CONFIG.GPIO_PIN_MODE <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle -> GPIO_PIN_CONFIG.GPIO_PIN_MODE<< (2 * pGPIOHandle -> GPIO_PIN_CONFIG.GPIO_PIN_NUMBER));
	    pGPIOHandle ->pGPIOx ->MODER &= ~(0x3<< pGPIOHandle ->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER);
		pGPIOHandle -> pGPIOx->MODER |= temp;
	    temp =0;
	}
	else
	{

	}
	//CONFIGURE GPIO PIN SPEED
	temp =0;
	temp = (pGPIOHandle ->GPIO_PIN_CONFIG.GPIO_PIN_SPEED << (2* pGPIOHandle ->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER));
	pGPIOHandle ->pGPIOx ->OSPEEDR &= ~(0x3<< pGPIOHandle ->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER);
	pGPIOHandle -> pGPIOx ->OSPEEDR |= temp;
    temp =0;

    //CONFIGURE GPIO PULL UP PULL DOWN REGISTER
    temp =0;
    temp = (pGPIOHandle ->GPIO_PIN_CONFIG.GPIO_PIN_PUPDR << (2 * pGPIOHandle ->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER));
    pGPIOHandle ->pGPIOx ->PUPDR &= ~(0x3<< pGPIOHandle ->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER);
    pGPIOHandle ->pGPIOx->PUPDR |= temp;
    temp =0;

    //CONFIGURE THE GPIO PORT OUTPUT TYPE REGISTER
    temp =0;
    temp =(pGPIOHandle ->GPIO_PIN_CONFIG.GPIO_PIN_OTYPE << (1 * pGPIOHandle ->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER));
    pGPIOHandle ->pGPIOx ->OTYPER &= ~(0x1 << pGPIOHandle ->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER);
    pGPIOHandle ->pGPIOx->OTYPER |=temp;
    temp =0;

    //CONFIGURE THE GPIO PORT ALTERNATE FUNCTION
    temp =0;
    if(pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_MODE == GPIO_MODE_ALTFN )
    {
    	uint32_t temp1,temp2;
    	temp1 = (pGPIOHandle ->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER) /8 ; //this is to find afr reg num
    	temp2 = (pGPIOHandle ->GPIO_PIN_CONFIG.GPIO_PIN_NUMBER) %8;  //this is to find pin number
    	pGPIOHandle ->pGPIOx ->AFR[temp1] &= ~(0xf << (4*temp2));
        pGPIOHandle ->pGPIOx ->AFR[temp1] |= (pGPIOHandle->GPIO_PIN_CONFIG.GPIO_PIN_AF << (4*temp2));
    }
    temp =0;


}


void GPIO_DeInit(GPIO_REGDEF_X *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
	    GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

void GPIO_PeriClockControl(GPIO_REGDEF_X *pGPIOx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_CLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_CLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_CLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_CLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_CLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_CLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_CLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_CLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_CLK_EN();
		}
	}
	else if(EnorDi == DISABLE)
	{

	    if(pGPIOx == GPIOA)
		{
			GPIOA_CLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_CLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_CLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_CLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_CLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_CLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_CLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_CLK_DI();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_CLK_DI();
		}
    }
}

void GPIO_WriteToOutputPin(GPIO_REGDEF_X *pGPIOx,uint8_t PinNumber,uint8_t value)
{
    if(value == GPIO_PIN_SET)
    {
    	pGPIOx ->ODR |= (1 << PinNumber);
    }
    else
    {
    	pGPIOx ->ODR &= ~(1 << PinNumber);
    }

}

void GPIO_WriteToOutputPort(GPIO_REGDEF_X *pGPIOx,uint16_t value)
{
    pGPIOx ->ODR = value;
}

uint8_t GPIO_ReadFromInputPin(GPIO_REGDEF_X *pGPIOx,uint8_t PinNumber)
{
    uint8_t data =0;
    data = (uint8_t)((pGPIOx ->IDR >> PinNumber) & 0x00000001);
    return data;
}

uint16_t GPIO_ReadFromInputPort(GPIO_REGDEF_X *pGPIOx)
{
	uint16_t data =0;
	data = (uint16_t)pGPIOx ->IDR;
	return data;
}

void GPIO_ToggleOutputPin(GPIO_REGDEF_X *pGPIOx,uint8_t PinNumber)
{
    pGPIOx ->ODR ^= ( 1 << PinNumber);
}

void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnorDi)
{

}

void GPIO_IRQHandling(uint8_t PinNumber)
{

}

void delay(void)
{
	volatile long i;
	for(i=0;i<500000;i++);
}
