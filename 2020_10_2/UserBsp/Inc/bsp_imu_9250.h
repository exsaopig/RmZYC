#ifndef __BSP_IMU_9250_H__
#define __BSP_IMU_9250_H__
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "main.h"
#include "stdio.h"
#include "stdio.h"
#include "remote_info.h"
#include "sys_config.h"

void IIC_Init(void);
void IIC_Start(void);
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
void IIC_Stop(void);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(unsigned char ack);
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);
#endif

