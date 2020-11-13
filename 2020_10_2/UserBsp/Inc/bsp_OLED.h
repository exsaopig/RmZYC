#ifndef __BSP_OLED_H
#define __BSP_OLED_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "sys_config.h"
#include "bsp_can.h"

#define OLED_SCL_UP   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
#define OLED_SCL_DOWN HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
#define OLED_SDA_UP   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
#define OLED_SDA_DOWN HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
#define OLED_RST_UP   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
#define OLED_RST_DOWN HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
#define OLED_DC_UP    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_5, GPIO_PIN_SET);
#define OLED_DC_DOWN  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_5, GPIO_PIN_RESET);

#define XLevelL		0x00
#define XLevelH		0x10
#define XLevel		((XLevelH&0x0F)*16+XLevelL)
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xCF 

#define X_WIDTH 128
#define Y_WIDTH 64

typedef struct 
{
	uint16_t pagination;
	uint16_t pagination_max;
	uint16_t pagination_min;
}page_t;

typedef enum
{
	pid
}para_t;

extern para_t para;

extern page_t page;

void LCD_WrDat(unsigned char dat);
void LCD_CLS(void);
void LCD_Fill(unsigned char bmp_dat);
void LCD_Set_Pos(unsigned char x, unsigned char y);
void LCD_WrCmd(unsigned char cmd);
void LCD_Init(void);
void LCD_P14x16Ch(unsigned char x,unsigned char y,unsigned char N);
void LCD_P8x16Str(unsigned char x,unsigned char y,unsigned char ch[]);
void LCD_P6x8StrW(unsigned char x,unsigned char y, char ch[]);
void LCD_P6x8Str(unsigned char x,unsigned char y,char ch[]);
void oledprintf(uint8_t x, uint8_t y,float a, ...);
void oledprintfw(uint8_t x, uint8_t y,float a, ...);
void oledprintf_int(uint8_t x, uint8_t y,int a, ...);

void int2char(int slope, char *buffer);

void float2char(float slope,char*buffer,int n);

void page_init(void);
void paging(void);
void pageshow(void);
void Draw_pq(void);
void Draw_riding(void);
void Draw_mm(void);
void Draw_skpq(void);

#endif
