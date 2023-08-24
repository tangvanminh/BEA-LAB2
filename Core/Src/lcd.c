/*
 * lcd.c
 *
 *  Created on: Aug 23, 2023
 *      Author: MINH
 */

#include "lcd.h"
#include "lcd_lib.h"
#include "math.h"
#include <stdlib.h>

/******************************************************************************
function :	write eight bits' data to LCD
parameter:
  chByte : send data
  chCmd  : command or data
******************************************************************************/
void lcd_write_byte(uint8_t chByte, uint8_t chCmd)
{
    if (chCmd) {
        LCD_DC_H();
    } else {
        LCD_DC_L();
    }

    LCD_CS_L();
    HAL_SPI_Transmit(&hspi1, &chByte, 1, 1000);
    LCD_CS_H();
}

/******************************************************************************
function :	write sixteen bits' data to LCD
parameter:
  chByte : send data
  chCmd  : command or data
******************************************************************************/
void lcd_write_word(uint16_t hwData)
{
    LCD_DC_H();
    LCD_CS_L();
    uint8_t highBitsHwData = hwData >> 8;
    uint8_t lowBitsHwData = hwData & 0xFF;
    HAL_SPI_Transmit(&hspi1, &highBitsHwData, 1, 1000);
    HAL_SPI_Transmit(&hspi1, &lowBitsHwData, 1, 1000);

    LCD_CS_H();
}

/******************************************************************************
function :	write data to LCD register
parameter:
  chByte : send data
  chCmd  : command or data
******************************************************************************/
void lcd_write_command(uint8_t chRegister, uint8_t chValue)
{
	lcd_write_byte(chRegister, LCD_CMD);
	lcd_write_byte(chValue, LCD_DATA);
}

/******************************************************************************
Function Name  : delay
	  parameter: ms
******************************************************************************/
void delay_ms(uint32_t ms)
{
	uint32_t j=5000;
	for(;ms>2;ms--)
	for(;j>2;j--){

	}
}

/******************************************************************************
Function Name  : initials lcd control pin
******************************************************************************/
void lcd_init(void)
{
	LCD_RST_H();

	LCD_CS_H();
	LCD_BKL_H();
	LCD_RST_H();
	delay_ms(5);
	LCD_RST_L();
	delay_ms(5);
	LCD_RST_H();
	delay_ms(5);
	LCD_CS_H();

	lcd_write_byte(0x11,LCD_CMD);
	delay_ms(10);
	lcd_write_command(0x36,0x00);
	lcd_write_command(0x3a,0x05);
	lcd_write_byte(0xb2,LCD_CMD);
	lcd_write_byte(0x0c,LCD_DATA);
	lcd_write_byte(0x0c,LCD_DATA);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte(0x33,LCD_DATA);
	lcd_write_byte(0x33,LCD_DATA);
	lcd_write_command(0xb7,0x35);
	lcd_write_command(0xbb,0x28);
	lcd_write_command(0xc0,0x3c);
	lcd_write_command(0xc2,0x01);
	lcd_write_command(0xc3,0x0b);
	lcd_write_command(0xc4,0x20);
	lcd_write_command(0xc6,0x0f);
	lcd_write_byte(0xD0,LCD_CMD);
	lcd_write_byte(0xa4,LCD_DATA);
	lcd_write_byte(0xa1,LCD_DATA);
	lcd_write_byte(0xe0,LCD_CMD);
	lcd_write_byte(0xd0,LCD_DATA);
	lcd_write_byte(0x01,LCD_DATA);
	lcd_write_byte(0x08,LCD_DATA);
	lcd_write_byte(0x0f,LCD_DATA);
	lcd_write_byte(0x11,LCD_DATA);
	lcd_write_byte(0x2a,LCD_DATA);
	lcd_write_byte(0x36,LCD_DATA);
	lcd_write_byte(0x55,LCD_DATA);
	lcd_write_byte(0x44,LCD_DATA);
	lcd_write_byte(0x3a,LCD_DATA);
	lcd_write_byte(0x0b,LCD_DATA);
	lcd_write_byte(0x06,LCD_DATA);
	lcd_write_byte(0x11,LCD_DATA);
	lcd_write_byte(0x20,LCD_DATA);
	lcd_write_byte(0xe1,LCD_CMD);
	lcd_write_byte(0xd0,LCD_DATA);
	lcd_write_byte(0x02,LCD_DATA);
	lcd_write_byte(0x07,LCD_DATA);
	lcd_write_byte(0x0a,LCD_DATA);
	lcd_write_byte(0x0b,LCD_DATA);
	lcd_write_byte(0x18,LCD_DATA);
	lcd_write_byte(0x34,LCD_DATA);
	lcd_write_byte(0x43,LCD_DATA);
	lcd_write_byte(0x4a,LCD_DATA);
	lcd_write_byte(0x2b,LCD_DATA);
	lcd_write_byte(0x1b,LCD_DATA);
	lcd_write_byte(0x1c,LCD_DATA);
	lcd_write_byte(0x22,LCD_DATA);
	lcd_write_byte(0x1f,LCD_DATA);
	lcd_write_byte(0x29,LCD_CMD);
	lcd_write_command(0x51,0xff);
	lcd_write_command(0x55,0xB0);

	lcd_clear_screen(WHITE);
}

/******************************************************************************
Function Name  : set lcd cursor
	  parameter:
		 hwXpos: x axis position
		 hwYpos: y axis position
******************************************************************************/
void lcd_set_cursor(uint16_t hwXpos, uint16_t hwYpos){
	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}
	lcd_write_byte(0x2A,LCD_CMD);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte(hwXpos,LCD_DATA);
	lcd_write_byte(0x2B,LCD_CMD);
	lcd_write_byte((hwYpos>>8)&0xff,LCD_DATA);
	lcd_write_byte(hwYpos&0xff,LCD_DATA);
}

/******************************************************************************
Function Name  : lcd draw a dot
	  parameter:
		 hwXpos: 	x axis position
		 hwYpos: 	y axis position
		hwColor:	dot color
******************************************************************************/
void lcd_draw_dot(uint16_t hwXpos, uint16_t hwYpos, uint16_t hwColor)
{
	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}
	lcd_set_cursor(hwXpos, hwYpos);
	lcd_write_byte(0x2C, LCD_CMD);
	lcd_write_word(hwColor);
}

/******************************************************************************
Function Name  : lcd draw a big dot
	  parameter:
	color_front:	dot color
		 hwXpos: x axis position
		 hwYpos: y axis position
******************************************************************************/
void lcd_draw_bigdot(uint32_t color_front,
                     uint32_t x,uint32_t y )
{
    lcd_draw_dot(color_front,x,y);
    lcd_draw_dot(color_front,x,y+1);
    lcd_draw_dot(color_front,x,y-1);

    lcd_draw_dot(color_front,x+1,y);
    lcd_draw_dot(color_front,x+1,y+1);
    lcd_draw_dot(color_front,x+1,y-1);

    lcd_draw_dot(color_front,x-1,y);
    lcd_draw_dot(color_front,x-1,y+1);
    lcd_draw_dot(color_front,x-1,y-1);

}

/******************************************************************************
Function Name  : clear lcd screen
	  parameter:
		hwColor: background color
******************************************************************************/
void lcd_clear_screen(uint16_t hwColor)
{
	uint32_t i, wCount = LCD_WIDTH;
	wCount *= LCD_HEIGHT;
	lcd_write_byte(0x2A,LCD_CMD);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte((LCD_WIDTH-1)&0xff,LCD_DATA);
	lcd_write_byte(0x2B,LCD_CMD);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte(((LCD_HEIGHT-1)>>8)&0xff,LCD_DATA);
	lcd_write_byte((LCD_HEIGHT-1)&0xff,LCD_DATA);
	lcd_write_byte(0x2C,LCD_CMD);
	LCD_CS_L();
	LCD_DC_H();

	uint8_t highBitsHwColor = hwColor>>8;
	uint8_t lowBitsHwColor = hwColor&0xFF;
	for(i=0;i<wCount;i++){
		HAL_SPI_Transmit(&hspi1, &highBitsHwColor, 1, 1000);
		HAL_SPI_Transmit(&hspi1, &lowBitsHwColor, 1, 1000);
	}
	LCD_CS_H();
}

/******************************************************************************
Function Name  : lcd display char
	  parameter:
		 hwXpos: x axis position
		 hwYpos: y axis position
		  chChr: display character
		 chSize: character size
		hwColor: character color
******************************************************************************/
void lcd_display_char(	 uint16_t hwXpos,
                         uint16_t hwYpos,
                         uint8_t chChr,
                         uint8_t chSize,
                         uint16_t hwColor)
{
	uint8_t i, j, chTemp;
	uint16_t hwYpos0 = hwYpos, hwColorVal = 0;

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}
	lcd_write_byte(0x2A,LCD_CMD);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte(hwXpos,LCD_DATA);
	lcd_write_byte((hwXpos) >> 8,LCD_DATA);
	lcd_write_byte((hwXpos) & 0xFF,LCD_DATA);

	lcd_write_byte(0x2B,LCD_CMD);
	lcd_write_byte(hwYpos >> 8,LCD_DATA);
	lcd_write_byte(hwYpos & 0xFF,LCD_DATA);
	lcd_write_byte((hwYpos) >> 8,LCD_DATA);
	lcd_write_byte((hwYpos) & 0xFF,LCD_DATA);
	lcd_write_byte(0x2C, LCD_CMD);

    for (i = 0; i < chSize; i ++) {
		if (FONT_1206 == chSize) {
			chTemp = c_chFont1206[chChr - 0x20][i];
		}
		else if (FONT_1608 == chSize) {
			chTemp = c_chFont1608[chChr - 0x20][i];
		}
		for (j = 0; j < 8; j ++) {
			if (chTemp & 0x80) {
				hwColorVal = hwColor;
				lcd_draw_dot(hwXpos, hwYpos, hwColorVal);
			}
			chTemp <<= 1;
			hwYpos ++;
			if ((hwYpos - hwYpos0) == chSize) {
				hwYpos = hwYpos0;
				hwXpos ++;
				break;
			}
		}
    }
}

/******************************************************************************
Function Name  : calculate N power
	  parameter:
			 m :  value
			 n :  exponent
******************************************************************************/
static uint32_t _pow(uint8_t m, uint8_t n)
{
	uint32_t result = 1;

	while(n --) result *= m;
	return result;
}

/******************************************************************************
Function Name  : lcd display number
	  parameter:
		 hwXpos: x axis position
		 hwYpos: y axis position
		  chNum: number
		  chLen: number length
		 chSize: number size
		hwColor: number color
******************************************************************************/
void lcd_display_num(	uint16_t hwXpos,  uint16_t hwYpos,
                        uint32_t chNum,   uint8_t chLen,
                        uint8_t chSize,   uint16_t hwColor)
{
	uint8_t i;
	uint8_t chTemp, chShow = 0;

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

	for(i = 0; i < chLen; i ++) {
		chTemp = (chNum / _pow(10, chLen - i - 1)) % 10;
		if(chShow == 0 && i < (chLen - 1)) {
			if(chTemp == 0) {
				lcd_display_char(hwXpos + (chSize / 2) * i, hwYpos, ' ', chSize, hwColor);
				continue;
			} else {
				chShow = 1;
			}
		}
	 	lcd_display_char(hwXpos + (chSize / 2) * i, hwYpos, chTemp + '0', chSize, hwColor);
	}
}

/******************************************************************************
Function Name  : lcd display string
	  parameter:
		 hwXpos: x axis position
		 hwYpos: y axis position
  	  pchString: display string
		 chSize: string size
	    hwColor: string color
******************************************************************************/
void lcd_display_string(	uint16_t hwXpos,uint16_t hwYpos,
							const uint8_t *pchString,
							uint8_t chSize,uint16_t hwColor)
{

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

    while (*pchString != '\0') {
        if (hwXpos > (LCD_WIDTH - chSize / 2)) {
			hwXpos = 0;
			hwYpos += chSize;
			if (hwYpos > (LCD_HEIGHT - chSize)) {
				hwYpos = hwXpos = 0;
				lcd_clear_screen(0x00);
			}
		}

        lcd_display_char(hwXpos, hwYpos, (uint8_t)*pchString, chSize, hwColor);
        hwXpos += chSize / 2;
        pchString ++;
    }
}

/******************************************************************************
Function Name  : lcd draw a line
	  parameter:
		hwXpos0: x axis start position
		hwYpos0: y axis start position
		hwXpos1: x axis end position
		hwYpos1: y axis end position
		hwColor: line color
******************************************************************************/
void lcd_draw_line(		uint16_t hwXpos0, uint16_t hwYpos0,
                      uint16_t hwXpos1, uint16_t hwYpos1,
                      uint16_t hwColor)
{
	int x = hwXpos1 - hwXpos0;
	int y = hwYpos1 - hwYpos0;
	int dx = abs(x), sx = hwXpos0 < hwXpos1 ? 1 : -1;
	int dy = -abs(y), sy = hwYpos0 < hwYpos1 ? 1 : -1;
	int err = dx + dy, e2;

	if (hwXpos0 >= LCD_WIDTH || hwYpos0 >= LCD_HEIGHT || hwXpos1 >= LCD_WIDTH || hwYpos1 >= LCD_HEIGHT) {
		return;
	}

    for (;;){
        lcd_draw_dot(hwXpos0, hwYpos0 , hwColor);
        e2 = 2 * err;
        if (e2 >= dy) {
            if (hwXpos0 == hwXpos1) break;
            err += dy; hwXpos0 += sx;
        }
        if (e2 <= dx) {
            if (hwYpos0 == hwYpos1) break;
            err += dx; hwYpos0 += sy;
        }
    }
}

/******************************************************************************
Function Name  : lcd draw a circle
	  parameter:
		 hwXpos: x axis  position
		 hwYpos: y axis  position
	   hwRadius: circle radius
		hwColor: cirlce color
******************************************************************************/
void lcd_draw_circle(		uint16_t hwXpos, uint16_t hwYpos,
                        uint16_t hwRadius,uint16_t hwColor)
{
	int x = -hwRadius, y = 0, err = 2 - 2 * hwRadius, e2;

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

    do {
        lcd_draw_dot(hwXpos - x, hwYpos + y, hwColor);
        lcd_draw_dot(hwXpos + x, hwYpos + y, hwColor);
        lcd_draw_dot(hwXpos + x, hwYpos - y, hwColor);
        lcd_draw_dot(hwXpos - x, hwYpos - y, hwColor);
        e2 = err;
        if (e2 <= y) {
            err += ++ y * 2 + 1;
            if(-x == y && e2 <= x) e2 = 0;
        }
        if(e2 > x) err += ++ x * 2 + 1;
    } while(x <= 0);
}

/******************************************************************************
Function Name  :  fill a rectangle on lcd
	  parameter:
		 hwXpos: x axis  position
		 hwYpos: y axis  position
	    hwWidth: rectangle width
	   hwHeight: rectangle height
	    hwColor: rectangle color
******************************************************************************/
void lcd_fill_rect(uint16_t hwXpos,
                   uint16_t hwYpos, uint16_t hwWidth,
                   uint16_t hwHeight,uint16_t hwColor)
{
	uint16_t i, j;

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

	for(i = 0; i < hwHeight; i ++){
		for(j = 0; j < hwWidth; j ++){
			lcd_draw_dot(hwXpos + j, hwYpos + i, hwColor);
		}
	}
}

/******************************************************************************
Function Name  : draw a vertical line at the specified position on lcd
	  parameter:
		 hwXpos: x axis  position
		 hwYpos: y axis  position
	   hwHeight: line height
		hwColor: vertical linc color
******************************************************************************/
void lcd_draw_v_line(		uint16_t hwXpos,uint16_t hwYpos,
                        uint16_t hwHeight,uint16_t hwColor)
{
	uint16_t i, y1 = MIN(hwYpos + hwHeight, LCD_HEIGHT - 1);

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

    for (i = hwYpos; i < y1; i ++) {
        lcd_draw_dot(hwXpos, i, hwColor);
    }
}

/******************************************************************************
Function Name  : draw a horizonal line at the specified position on lcd
	  parameter:
		 hwXpos: x axis  position
		 hwYpos: y axis  position
		hwWidth: line width
		hwColor: horizonal linc color
******************************************************************************/
void lcd_draw_h_line(		uint16_t hwXpos, uint16_t hwYpos,
                        uint16_t hwWidth,uint16_t hwColor)
{
	uint16_t i, x1 = MIN(hwXpos + hwWidth, LCD_WIDTH - 1);

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

    for (i = hwXpos; i < x1; i ++) {
        lcd_draw_dot(i, hwYpos, hwColor);
    }
}

/******************************************************************************
Function Name  : draw a rectangle on lcd
	  parameter:
		 hwXpos: x axis  position
		 hwYpos: y axis  position
		hwWidth: rectangle width
	   hwHeight: rectangle height
		hwColor: rectangle color
******************************************************************************/
void lcd_draw_rect(		uint16_t hwXpos,
                      uint16_t hwYpos,uint16_t hwWidth,
                      uint16_t hwHeight,uint16_t hwColor)
{
	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

	lcd_draw_h_line(hwXpos, hwYpos, hwWidth, hwColor);
	lcd_draw_h_line(hwXpos, hwYpos + hwHeight, hwWidth, hwColor);
	lcd_draw_v_line(hwXpos, hwYpos, hwHeight, hwColor);
	lcd_draw_v_line(hwXpos + hwWidth, hwYpos, hwHeight + 1, hwColor);
}

/******************************************************************************
Function Name  : clear rectangle on lcd
	  parameter:
		 hwXpos: x axis  position
		 hwYpos: y axis  position
		hwXpos1: rectangle width
		hwYpos1: rectangle height
	color_front: rectangle color
******************************************************************************/
void lcd_clear_Rect(	uint32_t color_front,
											uint32_t hwXpos,uint32_t hwYpos,
											uint32_t hwXpos1,uint32_t hwYpos1)
{
	uint16_t i, j;

	if (hwXpos1 >= LCD_WIDTH || hwYpos1 >= LCD_HEIGHT) {
		return;
	}

	for(i = 0; i < hwYpos1-hwYpos+1; i ++){
		for(j = 0; j < hwXpos1-hwXpos+1; j ++){
			lcd_draw_dot(hwXpos + j, hwYpos + i, color_front);
		}
	}
}

