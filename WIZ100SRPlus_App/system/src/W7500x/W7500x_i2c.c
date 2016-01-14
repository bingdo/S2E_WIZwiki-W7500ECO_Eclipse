/**
  ******************************************************************************
  * @file    W7500x_stdPeriph_Driver/src/W7500x_i2c.c    
  * @author  IOP Team
  * @version v1.0.0
  * @date    01-May-2015
  * @brief   This file contains all the functions prototypes for the i2c
  *          firmware library.
  ******************************************************************************
  *
  ******************************************************************************
  */
/*include -------------------------------------*/
#include <stdio.h>
#include "W7500x_i2c.h"
#include "W7500x_gpio.h"

GPIO_InitTypeDef GPIO_InitDef;

uint32_t I2C_Init(void)
{
    //SCL setting
    GPIO_InitDef.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOC, &GPIO_InitDef);
    GPIO_SetBits(GPIOC, GPIO_Pin_4);
    
    //SDA setting
    GPIO_InitDef.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOC, &GPIO_InitDef);
    GPIO_ResetBits(GPIOC, GPIO_Pin_5);
    
    PAD_AFConfig((PAD_Type) PAD_PC, GPIO_Pin_4, (PAD_AF_TypeDef) PAD_AF1);
    PAD_AFConfig((PAD_Type) PAD_PC, GPIO_Pin_5, (PAD_AF_TypeDef) PAD_AF1);

    return 0;
}

void I2C_WriteBitSCL(uint8_t data)
{
    if(data == 1)
        GPIO_SetBits(GPIOC, GPIO_Pin_4);
    else
        GPIO_ResetBits(GPIOC, GPIO_Pin_4);
}

void I2C_WriteBitSDA(uint8_t data)
{
    if(data == 1)
        GPIOC->OUTENCLR = GPIO_Pin_5;
    else
        GPIOC->OUTENSET = GPIO_Pin_5;
}

uint8_t I2C_ReadBitSDA(void)
{
    if(GPIOC->DATA & GPIO_Pin_5)
        return 1;
    else
        return 0;
    
    return 0;
}

void I2C_delay_us(uint16_t num)
{
	uint8_t i,j;

	for(i=0;i<num;i++)
		for(j=0;j<20;j++);
}

void I2C_Start(void)
{
    I2C_WriteBitSDA(1);
    I2C_WriteBitSCL(1);
    I2C_delay_us(4);
    I2C_WriteBitSDA(0);
    I2C_delay_us(4);
    I2C_WriteBitSCL(0);
}

void I2C_Stop(void)
{
    I2C_WriteBitSCL(0);
    I2C_WriteBitSDA(0);
    I2C_delay_us(4);
    I2C_WriteBitSCL(1);
    I2C_WriteBitSDA(1);
    I2C_delay_us(4);
}

uint8_t I2C_Wait_Ack(void)
{
	uint16_t ucErrTime=0;

	I2C_WriteBitSDA(1);
    I2C_delay_us(1);
	I2C_WriteBitSCL(1);
    I2C_delay_us(1);

    while(I2C_ReadBitSDA())
    {
    	ucErrTime++;
    	if(ucErrTime>2500)
    	{
    		I2C_Stop();
    		return 1;
    	}
    }

    I2C_WriteBitSCL(0);

    return 0;
}

void I2C_SendACK(void)
{
    I2C_WriteBitSCL(0);
    I2C_WriteBitSDA(0);
    I2C_delay_us(2);
    I2C_WriteBitSCL(1);
    I2C_delay_us(2);
    I2C_WriteBitSCL(0);
}

void I2C_SendNACK(void)
{
    I2C_WriteBitSCL(0);
    I2C_WriteBitSDA(1);
    I2C_delay_us(2);
    I2C_WriteBitSCL(1);
    I2C_delay_us(2);
    I2C_WriteBitSCL(0);
}

void I2C_WriteByte(uint8_t data)
{
    int i;

    I2C_WriteBitSCL(0);
    
    for(i=0; i<8; i++)
    {
        if(data&0x80)
            I2C_WriteBitSDA(1);
        else
            I2C_WriteBitSDA(0);
        
        data<<=1;
        I2C_delay_us(2);
        I2C_WriteBitSCL(1);
        I2C_delay_us(2);
        I2C_WriteBitSCL(0);
        I2C_delay_us(2);
    }
}

uint8_t I2C_ReadByte(uint8_t ack)
{
	uint8_t i,receive=0;

	I2C_WriteBitSDA(1);

	for(i=0; i<8; i++)
	{
        I2C_WriteBitSCL(0);
        I2C_delay_us(2);
        I2C_WriteBitSCL(1);
        receive<<=1;

        if(I2C_ReadBitSDA()) receive++;

        I2C_delay_us(1);
	}

    if(ack)
    	I2C_SendACK();
    else
    	I2C_SendNACK();

    return receive;
}
