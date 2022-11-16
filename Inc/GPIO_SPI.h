#ifndef GPIO_SPI_H
#define GPIO_SPI_H

#include "stm32f4xx_hal.h"

#include <inttypes.h>

#ifdef HAL_MODULE_ENABLED

struct SPI_Port_t
{
	  GPIO_TypeDef* 	SCS_Port;
		uint16_t 				SCS_Pin;
	  GPIO_TypeDef* 	SCLK_Port;
		uint16_t 				SCLK_Pin;
		GPIO_TypeDef* 	MOSI_Port; 
		uint16_t 				MOSI_Pin;
		GPIO_TypeDef* 	MISO_Port; 
		uint16_t 				MISO_Pin;
};

struct LCD_Port_t
{
		GPIO_TypeDef* 	RESET_Port; 
		uint16_t 				RESET_Pin;
		//GPIO_TypeDef* 	SCS_LCD_Port; 
		//uint16_t 				SCS_LCD_Pin;
		//GPIO_TypeDef* 	SCLK_Port; 
		//uint16_t 				SCLK_Pin;
		//GPIO_TypeDef* 	MOSI_Port; 
		//uint16_t 				MOSI_Pin;
	  struct SPI_Port_t SPI_Port;
};

void ConfigPort(struct LCD_Port_t LCD_Port_struct);
#endif

void Set_Rst(GPIO_PinState state);
void Delay(uint32_t ms);

void Set_SCLK(GPIO_PinState state);
void Set_MOSI(GPIO_PinState state);
void Set_SCS(GPIO_PinState state);

void SPI_Write_Byte(uint8_t data);

void SPI_Write_Cmd(uint8_t data);
void SPI_Write_Data(uint8_t data);

void Init_SPI(void);


#endif // !GPIO_SPI_H
