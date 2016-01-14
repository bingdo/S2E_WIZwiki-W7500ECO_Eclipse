
#ifndef __GPIOHANDLER_H__
#define __GPIOHANDLER_H__

#include <stdint.h>
#include <stdbool.h>
#include "W7500x_gpio.h"

#define LEDn                             2

#define LED1_PIN						GPIO_Pin_1
#define LED1_GPIO_PORT					GPIOA
//#define LED1_GPIO_PAD					PAD_PA
//#define LED1_GPIO_AF					PAD_AF1

#define LED2_PIN						GPIO_Pin_2
#define LED2_GPIO_PORT					GPIOA
//#define LED2_GPIO_PAD					PAD_PA
//#define LED2_GPIO_AF					PAD_AF1

#define BOOT_PIN						GPIO_Pin_6
#define BOOT_GPIO_PORT					GPIOC
//#define BOOT_GPIO_PAD					PAD_PC
//#define BOOT_GPIO_AF					PAD_AF1

#define FACT_PIN						GPIO_Pin_7
#define FACT_GPIO_PORT					GPIOC
//#define FACT_GPIO_PAD					PAD_PC
//#define FACT_GPIO_AF					PAD_AF1

typedef enum
{
  LED1 = 0,
  LED2 = 1
} Led_TypeDef;

void LED_Init(Led_TypeDef Led);
void LED_On(Led_TypeDef Led);
void LED_Off(Led_TypeDef Led);
void LED_Toggle(Led_TypeDef Led);
uint8_t get_LED_Status(Led_TypeDef Led);
void BOOT_Pin_Init();
uint8_t get_bootpin_Status();
bool Board_factory_get(void);
void Board_factory_Init(void);

#endif
