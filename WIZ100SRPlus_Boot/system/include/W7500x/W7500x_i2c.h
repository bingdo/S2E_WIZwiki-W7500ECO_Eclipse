/**
  ******************************************************************************
  * @file    W7500x_stdPeriph_Driver/inc/W7500x_i2c.h
  * @author  IOP Team
  * @version V1.0.0
  * @date    01-May-2015
  * @brief   This file contains all the functions prototypes for the I2C 
  *          firmware library.
  ******************************************************************************
  *
  ******************************************************************************
  */

/*include -------------------------------------*/
#ifndef __W7500X_I2C_H
#define __W7500X_I2C_H

#include "W7500x.h"


typedef enum {
    I2C_PA_5    = 0x05,
    I2C_PA_6    = 0x06,
    I2C_PA_9    = 0x09,
    I2C_PA_10   = 0x0A,
    I2C_PC_4    = 0x24,
    I2C_PC_5    = 0x25,
    I2C_PC_8    = 0x28,
    // Not connected
    I2C_NC = (int)0xFFFFFFFF
} I2C_PinName;

typedef struct
{
    I2C_PinName scl;
    I2C_PinName sda;
}I2C_ConfigStruct;


#define I2C_PORT(X) (((uint32_t)(X) >> 4) & 0xF)    // port number (0=A, 1=B, 2=C, 3=D)
#define I2C_PIN_INDEX(X)  (1 << ((uint32_t)(X) & 0xF))    // pin index : flag bit 
 
uint32_t I2C_Init(void);
void I2C_WriteBitSCL(uint8_t data);
void I2C_WriteBitSDA(uint8_t data);
uint8_t I2C_ReadBitSDA(void);

void I2C_delay_us(uint16_t num);
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_Wait_Ack(void);
void I2C_SendACK(void);
void I2C_SendNACK(void);
void I2C_WriteByte(uint8_t data);
uint8_t I2C_ReadByte(uint8_t ack);

#endif //__W7500X_I2C_H

