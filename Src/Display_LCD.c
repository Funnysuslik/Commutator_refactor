#include "Display_LCD.h"
#include "TFT_Font_8x8.h"
#include <string.h>

uint16_t g_bckgr_clr=0x00; //global background color, default - black
uint16_t g_ink_color=0xffff;
uint16_t point_matrix[MAX_WIDTH][MAX_HEIGHT];

//struct LCD_Port_t	lh_LCD_Port_info;

/************************************************************************************/
/* function change_background_color set background color 														*/
/*		color	- 16 bit RGB Color												  					                  */
/*	return:																																					*/
/* Version 31 august 2020																														*/
/************************************************************************************/
void change_background_color(uint16_t color)
{
	g_bckgr_clr=color;
}

/************************************************************************************/
/* function change_ink_color set ink color 																					*/
/*		color	- 16 bit RGB Color												  					                  */
/*	return:																																					*/
/* Version 31 august 2020																														*/
/************************************************************************************/
void change_ink_color(uint16_t color)
{
	g_ink_color=color;
}

void cls(void)
{
	uint16_t lx,ly;
	for (lx=0;lx<MAX_WIDTH;lx++)
	{
		for (ly=0;ly<MAX_HEIGHT;ly++)
		{
			point_matrix[lx][ly]=g_bckgr_clr;
			SPI_Write_Data(g_bckgr_clr&0xff);
			SPI_Write_Data((g_bckgr_clr>>8)&0xff);
		}
	}
}

void cls_and_set_bcgrnd(uint16_t new_bg_color)
{
	g_bckgr_clr=new_bg_color;
	uint16_t lx,ly;
	for (lx=0;lx<MAX_WIDTH;lx++)
	{
		for (ly=0;ly<MAX_HEIGHT;ly++)
		{
			point_matrix[lx][ly]=g_bckgr_clr;
			SPI_Write_Data(g_bckgr_clr&0xff);
			SPI_Write_Data((g_bckgr_clr>>8)&0xff);
		}
	}
}
/************************************************************************************/
/* function ConfigDisplayPort set SPI Display port on GPIO pins											*/
/*		LCD_Port	- config LCD GPIO Port info						  					                  */
/*	return:																																					*/
/* Version 30 may 2020																															*/
/************************************************************************************/
void ConfigDisplayPort(struct LCD_Port_t	LCD_Port)
{
	ConfigPort(LCD_Port);
}
/************************************************************************************/
/* function draw_line draw line on LCD																							*/
/*		x1	- graphic x coordinate of point 1 				    							              */
/*		y1	- graphic x coordinate of point 1 			  					    			            */
/*		x2	- graphic x coordinate of point 2 				    							              */
/*		y2	- graphic x coordinate of point 2 			  					    			            */
/*		color	- colour of line						  					    							              */
/*	return:																																					*/
/* Version 31 august 2020																														*/
/************************************************************************************/
void draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	uint16_t dx, dy;
	//uint16_t steps=0;
	uint16_t segment_length=0;
	
	dx=x2-x1;
	dy=y2-y1;
	
	if (dx>dy)
	{
		segment_length=dx/(dy+1);
		uint16_t x=x1;
		for (uint16_t ly=y1; ly<=y2; ly++)
		{			
			for (uint16_t lx=x; lx<=x+segment_length; lx++)
			{
				point_matrix[lx][ly]=color;
			}
			x=x+segment_length;
		}
	}
	
	if (dy>dx)
	{
		segment_length=dy/(dx+1);
		uint16_t y=y1;
		for (uint16_t lx=x1; lx<=x2; lx++)
		{			
			for (uint16_t ly=y; ly<=y+segment_length; ly++)
			{
				point_matrix[lx][ly]=color;
			}
			y=y+segment_length;
		}
	}
	
	if (dx==dy)
	{
		segment_length=0;
		uint16_t x=x1;
		uint16_t y=y1;
		for (uint16_t ly=y1; ly<=y+segment_length; ly++)
		{			
			for (uint16_t lx=x; lx<=x+segment_length; lx++)
			{
				point_matrix[lx][ly]=color;
			}
			x=x+segment_length;
		}
	}
	
	for (uint16_t lx=0;lx<MAX_WIDTH;lx++)
	{
		for (uint16_t ly=0;ly<MAX_HEIGHT;ly++)
		{
			SPI_Write_Data((point_matrix[lx][ly])&0xff);			
			SPI_Write_Data(((point_matrix[lx][ly])>>8)&0xff);			
		}
	}
}

/************************************************************************************/
/* function draw_line_to_buff draw line to buffer of program LCD										*/
/*		x1	- graphic x coordinate of point 1 				    							              */
/*		y1	- graphic x coordinate of point 1 			  					    			            */
/*		x2	- graphic x coordinate of point 2 				    							              */
/*		y2	- graphic x coordinate of point 2 			  					    			            */
/*		color	- colour of line						  					    							              */
/*	return:																																					*/
/* Version 31 august 2020																														*/
/************************************************************************************/
void draw_line_to_buff(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	uint16_t dx, dy;
	//uint16_t steps=0;
	uint16_t segment_length=0;
	
	dx=x2-x1;
	dy=y2-y1;
	
	if (dx>dy)
	{
		segment_length=dx/(dy+1);
		uint16_t x=x1;
		for (uint16_t ly=y1; ly<=y2; ly++)
		{			
			for (uint16_t lx=x; lx<=x+segment_length; lx++)
			{
				point_matrix[lx][ly]=color;
			}
			x=x+segment_length;
		}
	}
	
	if (dy>dx)
	{
		segment_length=dy/(dx+1);
		uint16_t y=y1;
		for (uint16_t lx=x1; lx<=x2; lx++)
		{			
			for (uint16_t ly=y; ly<=y+segment_length; ly++)
			{
				point_matrix[lx][ly]=color;
			}
			y=y+segment_length;
		}
	}
	
	if (dx==dy)
	{
		segment_length=0;
		uint16_t x=x1;
		uint16_t y=y1;
		for (uint16_t ly=y1; ly<=y+segment_length; ly++)
		{			
			for (uint16_t lx=x; lx<=x+segment_length; lx++)
			{
				point_matrix[lx][ly]=color;
			}
			x=x+segment_length;
		}
	}
}
/************************************************************************************/
/* function draw_string draw srting on LCD																					*/
/*		x	- graphic x coordinate					  					    							              */
/*		y	- graphic y coordinate					  					    							              */
/*		*string	- pointer to string array	  					    							              */
/*	return:																																					*/
/* Version 31 august 2020																														*/
/************************************************************************************/
void draw_string(uint16_t x, uint16_t y, char* string)
{
	uint16_t lx=x, ly=y;
	uint16_t size_of_string=strlen(string);
	
	for (uint16_t i=0; i<size_of_string; i++)
	{
		draw_symbol(lx,ly,string[i]);
		lx=lx+7;
	}
}
/************************************************************************************/
/* function draw_symbol draw one symbol on LCD																			*/
/*		x	- graphic x coordinate					  					    							              */
/*		y	- graphic y coordinate					  					    							              */
/*		symbol - code of symbol						  					    							              */
/*	return:																																					*/
/* Version 31 august 2020																														*/
/************************************************************************************/
void draw_symbol(uint16_t x, uint16_t y, char symbol)
{
	uint16_t lx=x, ly=y+7; //local (X;Y)
	uint16_t index=(symbol-32)*9; //SYMBOL index in TFT_Font8x8
	
	for (uint16_t i=0; i<8; i++)
	{
		for (uint16_t j=0; j<8; j++)
		{
			if ((TFT_Font_8x8[index+i+1]>>j)&0x01)
			{
				point_matrix[lx][ly]=g_ink_color; //INK color
			}
			else
			{
				point_matrix[lx][ly]=g_bckgr_clr; //background color
			}
			
			if (ly==y)
			{
				ly=y+7;
			}
			else
			{
				ly--;
			}
		}
		lx++;
	}	
}

void Init_LCD(void)
{
		Set_SCS(GPIO_PIN_SET);

    Set_Rst(GPIO_PIN_SET);
    Delay(1000);
    Set_Rst(GPIO_PIN_RESET);
    Delay(100);

    Set_Rst(GPIO_PIN_SET);
    Delay(500); 
    SPI_Write_Cmd(0x11);
    
    Delay(150);

    SPI_Write_Cmd(0x3a);    
		//SPI_Write_Data(0x03);  	//03: 12bit color  
		SPI_Write_Data(0x05); 	// 05: 16bit color 65K
		//SPI_Write_Data(0x06); 	// 06: 18bit color 262K
	
		/*
	  // comment for red
	  // uncomment for blue
		SPI_Write_Cmd(0x36);    
		SPI_Write_Data(orient); 
		//SPI_Write_Data(0x08); 
		//SPI_Write_Data(0xA0); 
	
	
		//ST7735S Frame rate setting
		
    SPI_Write_Cmd(0xB1); //Frame Rate Control (In normal mode/ Full colors)   
    SPI_Write_Data(0x05);
		SPI_Write_Data(0x3A);
		SPI_Write_Data(0x3A);
		*/
		/*
		SPI_Write_Cmd(0xB2); //Frame Rate Control (In Idle mode/ 8-colors)
		SPI_Write_Data(0x05);
		SPI_Write_Data(0x3A);
		SPI_Write_Data(0x3A);
    
    
		
		SPI_Write_Cmd(0xB3); //Frame Rate Control (In Partial mode/ full colors)
		SPI_Write_Data(0x05);
		SPI_Write_Data(0x3A);
		SPI_Write_Data(0x3A);
    SPI_Write_Data(0x05);
		SPI_Write_Data(0x3A);
		SPI_Write_Data(0x3A);
	
		SPI_Write_Cmd(0xB4); //Dot Inversion  
		SPI_Write_Data(0x03);  
	
		//ST7735S Power setting
		SPI_Write_Cmd(0xC0);    //Power Control 1
		SPI_Write_Data(0x62);
    SPI_Write_Data(0x02);
    SPI_Write_Data(0x04);

		SPI_Write_Cmd(0xC1);    //Power Control 2  
		SPI_Write_Data(0xC0);
		
		
    SPI_Write_Cmd(0xC2);    //Power Control 3 (in Normal mode/ Full colors)  
		SPI_Write_Data(0x0D);
    SPI_Write_Data(0x00);    
		
		SPI_Write_Cmd(0xC3);    //Power Control 4 (in Idle mode/ 8-colors)
		SPI_Write_Data(0x8D);
    SPI_Write_Data(0x6A);

		SPI_Write_Cmd(0xC4);    //Power Control 5 (in Partial mode/ full-colors)
		SPI_Write_Data(0x8D);
    SPI_Write_Data(0xEE);

    SPI_Write_Cmd(0xC5);    //VCOM Control 1
		SPI_Write_Data(0x12);
		
		//ST7735S Gamma Setting
		SPI_Write_Cmd(0xE0); //Gamma (??+??polarity) Correction Characteristics Setting
		SPI_Write_Data(0x03);
		SPI_Write_Data(0x1B);
		SPI_Write_Data(0x12);
		SPI_Write_Data(0x11);
		SPI_Write_Data(0x3F);
		SPI_Write_Data(0x3A);
		SPI_Write_Data(0x32);
		SPI_Write_Data(0x34);
		SPI_Write_Data(0x2F);
		SPI_Write_Data(0x2B);
		SPI_Write_Data(0x30);
		SPI_Write_Data(0x3A);
		SPI_Write_Data(0x00);
		SPI_Write_Data(0x01);
    SPI_Write_Data(0x02);
		SPI_Write_Data(0x05);
   
		SPI_Write_Cmd(0xE1); //Gamma ??-??polarity Correction Characteristics Setting
		SPI_Write_Data(0x03);
		SPI_Write_Data(0x1B);
		SPI_Write_Data(0x12);
		SPI_Write_Data(0x11);
		SPI_Write_Data(0x32);
		SPI_Write_Data(0x2F);
		SPI_Write_Data(0x2A);
		SPI_Write_Data(0x2F);
		SPI_Write_Data(0x2E);
		SPI_Write_Data(0x2C);
		SPI_Write_Data(0x35);
		SPI_Write_Data(0x3F);
		SPI_Write_Data(0x00);
		SPI_Write_Data(0x00);
    SPI_Write_Data(0x01);
		SPI_Write_Data(0x05);

    SPI_Write_Cmd(0xFC); //Enable Gate power save mode 
		SPI_Write_Data(0x8C);
	
		SPI_Write_Cmd(0x2A); //Column Address Set
		SPI_Write_Data(0x00);
		SPI_Write_Data(0x00);//02 : old FPC | 00 : New FPC
		SPI_Write_Data(0x00);//             |
		SPI_Write_Data(0x7F);//129 : old FPC| 128 : New FPC  
	
		SPI_Write_Cmd(0x2B); //Row Address Set
		SPI_Write_Data(0x00);
		SPI_Write_Data(0x00);//01 : Old  | 00 : New FPC
		SPI_Write_Data(0x00);//          |
		SPI_Write_Data(0x9F);//160 : Old | 159 : New FPC 
		*/

		SPI_Write_Cmd(0x29);  // Display on
		SPI_Write_Cmd(0x2C); // GRAM
}

void set_bckgrnd(uint16_t color)
{
	g_bckgr_clr=color;
}

void draw_point(uint16_t x, uint16_t y, uint16_t color)
{
	uint16_t lx,ly;
	point_matrix[x][y]=color;
	for (lx=0;lx<MAX_WIDTH;lx++)
	{
		for (ly=0;ly<MAX_HEIGHT;ly++)
		{
			SPI_Write_Data((point_matrix[lx][ly])&0xff);
			SPI_Write_Data(((point_matrix[lx][ly])>>8)&0xff);
		}
	}
}

void Paint_Color(uint8_t color)
{
  for (uint8_t i = 0; i < MAX_HEIGHT; ++i)
    {
      for (uint8_t j = 0; j < MAX_WIDTH; ++j)
      {
        SPI_Write_Data(color);
        SPI_Write_Data(color);
      }
    }
}

void SetPointBuf(uint16_t x, uint16_t y, uint16_t color)
{
	point_matrix[x][y]=color;
}

void UpdateBuf(void)
{
	uint16_t lx,ly;
	for (lx=0;lx<MAX_WIDTH;lx++)
	{
		for (ly=0;ly<MAX_HEIGHT;ly++)
		{
			SPI_Write_Data((point_matrix[lx][ly])&0xff);
			SPI_Write_Data(((point_matrix[lx][ly])>>8)&0xff);
			//SPI_Write_Data((point_matrix[lx][ly])&0xff);
		}
	}
}

/************************************************************************************/
/* function UpdateDisplayBuf redraw full LCD from buffer														*/
/*	return:																																					*/
/* Version 31 august 2020																														*/
/************************************************************************************/
void UpdateDisplayBuf(void)
{
	uint16_t lx,ly;
	for (lx=0;lx<MAX_WIDTH;lx++)
	{
		for (ly=0;ly<MAX_HEIGHT;ly++)
		{
			SPI_Write_Data((point_matrix[lx][ly])&0xff);
			SPI_Write_Data(((point_matrix[lx][ly])>>8)&0xff);
		}
	}
}
