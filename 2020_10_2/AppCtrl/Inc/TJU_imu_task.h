#ifndef __TJU_IMU_TASK_H
#define __TJU_IMU_TASK_H
#include "stm32f4xx_hal.h"
#include "main.h"
#include "sys_config.h"
#include "cmsis_os.h"
#include "reg.h"
#include "bsp_imu.h"

typedef struct
{
	float mx;
	float my;
	float mz;
}imu_acce_t;

typedef struct
{
  float mx;
	float my;
	float mz;	
}imu_angVel_t;
typedef struct
{
	float mx;
	float my;
	float mz;	
}magnetic_t;
typedef struct
{
	float mx;
	float my;
	float mz;		
}angle_t;
typedef struct
{
	imu_acce_t imu_acce;
	imu_angVel_t imu_angVel;
	magnetic_t magnetic;
	angle_t angle;
}imu_value_t;

extern imu_value_t imu_value;
void TJU_imu_task(void const * argument);
extern UBaseType_t imu_stack_surplus;

#endif
