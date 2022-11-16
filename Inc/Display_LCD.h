#include "GPIO_SPI.h"

#ifndef DISPLAY_LCD
#define DISPLAY_LCD

#define MAX_WIDTH 0
#define MAX_HEIGHT 0

#define WF18FTLAADNN0

#ifdef WF18FTLAADNN0
#undef MAX_WIDTH
#define MAX_WIDTH 160
#undef MAX_HEIGHT
#define MAX_HEIGHT 128
#endif //WF18FTLAADNN0

/************************************************************************************/
/* function change_background_color set background color 														*/
/*		color	- 16 bit RGB Color												  					                  */
/*	return:																																					*/
/* Version 31 august 2020																														*/
/************************************************************************************/
void change_background_color(uint16_t color); //set background in <color>
/************************************************************************************/
/* function change_ink_color set ink color 																					*/
/*		color	- 16 bit RGB Color												  					                  */
/*	return:																																					*/
/* Version 31 august 2020																														*/
/************************************************************************************/
void change_ink_color(uint16_t color);
void cls(void); //paint screen in global background
void cls_and_set_bcgrnd(uint16_t new_bg_color); //paint screen and set global background in <color>
/************************************************************************************/
/* function ConfigDisplayPort set SPI Display port on GPIO pins											*/
/*		LCD_Port	- config LCD GPIO Port info						  					                  */
/*	return:																																					*/
/* Version 31 may 2020																															*/
/************************************************************************************/
void ConfigDisplayPort(struct LCD_Port_t	LCD_Port);
void Init_LCD(void);
void set_bckgrnd(uint16_t color); //set background in <color>
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
void draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
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
void draw_line_to_buff(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void draw_point(uint16_t x, uint16_t y, uint16_t color); //draw a dot with (x,y) coordinates and paint it in <color>
/************************************************************************************/
/* function draw_string draw srting on LCD																					*/
/*		x	- graphic x coordinate					  					    							              */
/*		y	- graphic y coordinate					  					    							              */
/*		*string	- pointer to string array	  					    							              */
/*	return:																																					*/
/* Version 31 august 2020																														*/
/************************************************************************************/
void draw_string(uint16_t x, uint16_t y, char* string);
/************************************************************************************/
/* function draw_symbol draw one symbol on LCD																			*/
/*		x	- graphic x coordinate					  					    							              */
/*		y	- graphic y coordinate					  					    							              */
/*		symbol - code of symbol						  					    							              */
/*	return:																																					*/
/* Version 31 august 2020																														*/
/************************************************************************************/
void draw_symbol(uint16_t x, uint16_t y, char symbol);
void SetPointBuf(uint16_t x, uint16_t y, uint16_t color); //set buffer of points
void UpdateBuf(void); //update screen
/************************************************************************************/
/* function UpdateDisplayBuf redraw full LCD from buffer														*/
/*	return:																																					*/
/* Version 31 august 2020																														*/
/************************************************************************************/
void UpdateDisplayBuf(void);
#endif
