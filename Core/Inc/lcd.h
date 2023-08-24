/*
 * lcd.h
 *
 *  Created on: Aug 23, 2023
 *      Author: MINH
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "main.h"

#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))

//color define
#define LCD_WIDTH    	240
#define LCD_HEIGHT   	320

#define FONT_1206    	12
#define FONT_1608    	16
#define FONT_GB2312  	16

#define WHITE          	0xFFFF
#define BLACK          	0x0000
#define BLUE           	0x001F
#define BRED           	0XF81F
#define GRED 		   	0XFFE0
#define GBLUE		   	0X07FF
#define RED            	0xF800
#define MAGENTA        	0xF81F
#define GREEN          	0x07E0
#define CYAN           	0x7FFF
#define YELLOW         	0xFFE0
#define BROWN 		   	0XBC40
#define BRRED 		   	0XFC07
#define GRAY  		   	0X8430

/* PB6 ( BKL )  	PB7( LCD_CS )  	*/
/* PB8( DC )	PB2( RST )		*/
#define LCD_DC_H()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, SET)
#define LCD_DC_L()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, RESET)

#define LCD_CS_H()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET)
#define LCD_CS_L()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, RESET)

#define LCD_BKL_H()    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET)
#define LCD_BKL_L()    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, RESET)

#define LCD_RST_H()    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, SET)
#define LCD_RST_L()    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, RESET)

#define LCD_CMD                0
#define LCD_DATA               1

void lcd_init(void);
void lcd_set_cursor(uint16_t hwXpos, uint16_t hwYpos);
void lcd_draw_dot(uint16_t hwXpos, uint16_t hwYpos, uint16_t hwColor);
void lcd_draw_bigdot(uint32_t color_front,uint32_t x,uint32_t y );
void lcd_clear_screen(uint16_t hwColor);
void lcd_display_char(uint16_t hwXpos,uint16_t hwYpos,uint8_t chChr,uint8_t chSize,uint16_t hwColor);
void lcd_display_num(uint16_t hwXpos,uint16_t hwYpos,uint32_t chNum,uint8_t chLen,uint8_t chSize,uint16_t hwColor);
void lcd_display_string(uint16_t hwXpos,uint16_t hwYpos,const uint8_t *pchString,uint8_t chSize,uint16_t hwColor);
void lcd_draw_line(uint16_t hwXpos0,uint16_t hwYpos0,uint16_t hwXpos1,uint16_t hwYpos1,uint16_t hwColor);
void lcd_draw_circle(uint16_t hwXpos,uint16_t hwYpos,uint16_t hwRadius,uint16_t hwColor);
void lcd_fill_rect(uint16_t hwXpos,uint16_t hwYpos,uint16_t hwWidth,uint16_t hwHeight,uint16_t hwColor);
void lcd_draw_v_line(uint16_t hwXpos,uint16_t hwYpos,uint16_t hwHeight,uint16_t hwColor);
void lcd_draw_h_line(uint16_t hwXpos,uint16_t hwYpos,uint16_t hwWidth,uint16_t hwColor);
void lcd_draw_rect(uint16_t hwXpos,uint16_t hwYpos,uint16_t hwWidth,uint16_t hwHeight,uint16_t hwColor);
void lcd_clear_Rect(uint32_t color_front,uint32_t x0,uint32_t y0,uint32_t x1,uint32_t y1);

#endif /* INC_LCD_H_ */
