
#include <stdbool.h>
#include "extiHandler.h"
#include "gpioHandler.h"

GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED1_PIN, LED2_PIN};
//PAD_Type GPIO_PAD[LEDn] = {LED1_GPIO_PAD, LED2_GPIO_PAD};
//PAD_AF_TypeDef GPIO_AF[LEDn] = {LED1_GPIO_AF, LED2_GPIO_AF};

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  * @retval None
  */
void LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

  GPIO_Init(GPIO_PORT[Led], &GPIO_InitStructure);

  PAD_AFConfig(PAD_PA, GPIO_PIN[Led], PAD_AF1);

  delay_cnt(10000);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  * @retval None
  */
void LED_On(Led_TypeDef Led)
{
  GPIO_ResetBits(GPIO_PORT[Led], GPIO_PIN[Led]);
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  * @retval None
  */
void LED_Off(Led_TypeDef Led)
{
  GPIO_SetBits(GPIO_PORT[Led], GPIO_PIN[Led]);
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  * @retval None
  */
void LED_Toggle(Led_TypeDef Led)
{
  GPIO_PORT[Led]->DATAOUT ^= GPIO_PIN[Led];
}

uint8_t get_LED_Status(Led_TypeDef Led)
{
	return GPIO_ReadOutputDataBit(GPIO_PORT[Led], GPIO_PIN[Led]);
}

/**
  * @brief	Configures GPIO for D0~D15
  * @param	io Specifies the GPIO to be configured.
  *   This parameter can be one of following parameters:
  * 	@arg D0 @arg D1 @arg D2 @arg D3 @arg D4
  * 	@arg D5 @arg D6 @arg D7 @arg D8 @arg D9
  * 	@arg D10 @arg D11 @arg D12 @arg D13 @arg D14 @arg D15
  * @return None
  */
void BOOT_Pin_Init()
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
  	/* Configure the Boot trigger pin */
  	GPIO_InitStructure.GPIO_Pin = BOOT_PIN;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	
  	GPIO_Init(BOOT_GPIO_PORT, &GPIO_InitStructure);

    PAD_AFConfig(PAD_PC, BOOT_PIN, PAD_AF1);
}

/**
  * @brief	To get the status of GPIO.
  * @param	io Specifies the GPIO to get a status
  *   This parameter can be one of following parameters:
  * 	@arg D0 @arg D1 @arg D2 @arg D3 @arg D4
  * 	@arg D5 @arg D6 @arg D7 @arg D8 @arg D9
  * 	@arg D10 @arg D11 @arg D12 @arg D13 @arg D14 @arg D15
  * @return The status of GPIO
  */
uint8_t get_bootpin_Status()
{
	return GPIO_ReadInputDataBit(BOOT_GPIO_PORT, BOOT_PIN);
}

bool Board_factory_get(void)
{
	return GPIO_ReadInputDataBit(BOOT_GPIO_PORT, FACT_PIN);
}

void Board_factory_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
    EXTI_InitTypeDef 	EXTI_InitDef;

  	/* Configure the Boot trigger pin */
  	GPIO_InitStructure.GPIO_Pin = FACT_PIN;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;

  	GPIO_Init(FACT_GPIO_PORT, &GPIO_InitStructure);

    PAD_AFConfig(PAD_PC, FACT_PIN, PAD_AF1);

    /* Set to GPIO_Pin_0 to External Interrupt Port */
    EXTI_InitDef.EXTI_Line = FACT_PIN; // Connecting FACT_PIN(EXTI Input)
    EXTI_InitDef.EXTI_Trigger = EXTI_Trigger_Falling; // Set to Trigger to Falling
    EXTI_Init(PAD_PC, &EXTI_InitDef); // Set to PAD_PA
    EXTI_Polarity_Set(PAD_PC,FACT_PIN,EXTI_Trigger_Falling); // Set to Polarity
}

