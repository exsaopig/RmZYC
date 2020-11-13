/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file bsp_io.c
 *  @version 1.0
 *  @date June 2017
 *
 *  @brief basic IO port operation
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "bsp_io.h"
#include "sys_config.h"
#include "tim.h"
void turn_on_laser(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

void turn_off_laser(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

//uint8_t get_trigger_key_state(void)
//{
//  return HAL_GPIO_ReadPin(TRIG_GPIO_Port, TRIG_Pin);
//}


//void turn_on_friction_wheel(uint16_t spd)
//{
////  LEFT_FRICTION  = spd;
////  RIGHT_FIRCTION = spd;
//	  left_spd  = spd;
//	  right_spd = spd;
//}

//void turn_off_friction_wheel(void)
//{
////  LEFT_FRICTION  = 0;
////  RIGHT_FIRCTION = 0;
//	  left_spd  = 0;
//	  right_spd = 0;
//}

void mpu_heat_ctrl(uint16_t pwm_pulse)
{
  IMU_PWM_PULSE = pwm_pulse;
}

void pwm_device_init(void)
{
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //imu温度控制
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //蜂鸣器
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //摩擦轮
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //摩擦轮
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //舵机
}

uint8_t Infrared_ray_1(void)  
{
	return HAL_GPIO_ReadPin(Infrared_1_GPIO_Port, Infrared_1_Pin);
}
uint8_t Infrared_ray_2(void)
{
	return HAL_GPIO_ReadPin(Infrared_2_GPIO_Port, Infrared_2_Pin);
}

