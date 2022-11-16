#include "GPIO_SPI.h"


#ifdef HAL_MODULE_ENABLED

struct LCD_Port_t	lh_LCD_Port_info;
struct SPI_Port_t g_SPI_Port;
void ConfigPort(struct LCD_Port_t LCD_Port_struct)
{
	lh_LCD_Port_info=LCD_Port_struct;
	//g_SPI_Port=LCD_Port_struct.SPI_Port;
}

/********************************************************************************************/
/* function Config_SPI_Port config GPIO for only SPI														        		*/
/*		SPI_Port  -  struct SPI_Port_t		  					    							                      */
/*	return:																																					        */
/*                                                                                          */
/* Version 15 november 2020		  																										        */
/********************************************************************************************/
void Config_SPI_Port(struct SPI_Port_t SPI_Port)
{
	g_SPI_Port=SPI_Port;
}
#endif

void Set_Rst(GPIO_PinState state)
{
    HAL_GPIO_WritePin(lh_LCD_Port_info.RESET_Port,lh_LCD_Port_info.RESET_Pin,state);
}

void Delay(uint32_t ms)
{
    HAL_Delay(ms);
}

void Set_SCS(GPIO_PinState state)
{
    //HAL_GPIO_WritePin(lh_LCD_Port_info.SCS_LCD_Port,lh_LCD_Port_info.SCS_LCD_Pin,state);
		HAL_GPIO_WritePin(lh_LCD_Port_info.SPI_Port.SCS_Port,lh_LCD_Port_info.SPI_Port.SCS_Pin,state);
}

void Set_SCLK(GPIO_PinState state)
{
    //HAL_GPIO_WritePin(lh_LCD_Port_info.SCLK_Port,lh_LCD_Port_info.SCLK_Pin,state);
	HAL_GPIO_WritePin(lh_LCD_Port_info.SPI_Port.SCLK_Port,lh_LCD_Port_info.SPI_Port.SCLK_Pin,state);
	//HAL_GPIO_WritePin(g_SPI_Port.SCLK_Port,g_SPI_Port.SCLK_Pin,state);
}

void Set_MOSI(GPIO_PinState state)
{
    //HAL_GPIO_WritePin(lh_LCD_Port_info.MOSI_Port,lh_LCD_Port_info.MOSI_Pin,state);
	HAL_GPIO_WritePin(lh_LCD_Port_info.SPI_Port.MOSI_Port,lh_LCD_Port_info.SPI_Port.MOSI_Pin,state);
	//HAL_GPIO_WritePin(g_SPI_Port.MOSI_Port,g_SPI_Port.MOSI_Pin,state);
}

/********************************************************************************************/
/* function SPI_Write_16_bit write 16 bit to MOSI 															        		*/
/*		data  -  uint16_t writing data		  					    							                      */
/*	return:																																					        */
/*                                                                                          */
/* Version 15 november 2020		  																										        */
/********************************************************************************************/
void SPI_Write_16_bit(uint16_t data)
{
	for (uint8_t i = 0; i < 16; ++i)
  {
    if ((data << i) & 0x8000)
		{
			Set_MOSI(GPIO_PIN_SET);
		}
		else
		{
			Set_MOSI(GPIO_PIN_RESET);
		}
		Set_SCLK(GPIO_PIN_RESET);
		Set_SCLK(GPIO_PIN_SET);
    }
}

void SPI_Write_Byte(uint8_t data)
{
		//Set_Clk(GPIO_PIN_RESET);
	
    for (uint8_t i = 0; i < 8; ++i)
    {
        if ((data << i) & 0x80)
		{
			Set_MOSI(GPIO_PIN_SET);
		}
		else
		{
			Set_MOSI(GPIO_PIN_RESET);
		}
		Set_SCLK(GPIO_PIN_RESET);
		Set_SCLK(GPIO_PIN_SET);
    }
}

void SPI_Write_Cmd(uint8_t data)
{
    Set_SCS(GPIO_PIN_RESET);
    Set_MOSI(GPIO_PIN_RESET);
    Set_SCLK(GPIO_PIN_RESET);
    Set_SCLK(GPIO_PIN_SET);

    SPI_Write_Byte(data);
    Set_SCS(GPIO_PIN_SET);
}

void SPI_Write_Data(uint8_t data)
{
    Set_SCS(GPIO_PIN_RESET);
		Set_SCLK(GPIO_PIN_RESET);
    Set_MOSI(GPIO_PIN_SET);
    Set_SCLK(GPIO_PIN_SET);

    SPI_Write_Byte(data);
    Set_SCS(GPIO_PIN_SET);
}


