/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of?
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.? See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file judgement_info.h
 *  @version 1.0
 *  @date June 2017
 *
 *  @brief the information from judgement system
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#ifndef __JUDGEMENT_INFO_H__
#define __JUDGEMENT_INFO_H__

#include "stm32f4xx_hal.h"

#define JUDGE_FIFO_BUFLEN 512

/** 
  * @brief  FrameHeader structure definition
  */
typedef __packed struct
{
    uint8_t  sof;
    uint16_t dataLenth;
    uint8_t  seq;
    uint8_t  crc8;
} tFrameHeader;

/** 
  * @brief  judgement data command id
  */
typedef enum
{
	EVENT_DATA_ID      = 0x0101,
	PROJECTILE_ID			 = 0x0102,
	ROBOT_STATE_ID 		 = 0x0201,
	CHASSIS_INFO_ID		 = 0x0202,
	BUFF_ID						 = 0x0204,
	SHOOT_INFO_ID      = 0x0207,
	COMMUNICATE_ID     = 0x0301,
} judge_data_id_e;


typedef __packed struct
{
uint8_t game_type : 4;
uint8_t game_progress : 4;
uint16_t stage_remain_time;
} ext_game_state_t;

typedef __packed struct
{
uint8_t winner;
} ext_game_result_t;

typedef __packed struct
{
uint16_t robot_legion;
} ext_game_robot_survivors_t;

typedef __packed struct
{
uint32_t event_type;
} ext_event_data_t;

typedef __packed struct
{
uint8_t supply_projectile_id;
uint8_t supply_robot_id;
uint8_t supply_projectile_step;
uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef __packed struct
{
uint8_t supply_projectile_id;
uint8_t supply_robot_id;
uint8_t supply_num;
} ext_supply_projectile_booking_t;


typedef __packed struct
{
 uint8_t robot_id;
 uint8_t robot_level;
 uint16_t remain_HP;
 uint16_t max_HP;
 uint16_t shooter_heat0_cooling_rate;
 uint16_t shooter_heat0_cooling_limit;
 uint16_t shooter_heat1_cooling_rate;
 uint16_t shooter_heat1_cooling_limit;
 uint8_t shooter_heat0_speed_limit;
 uint8_t shooter_heat1_speed_limit;
 uint8_t max_chassis_power;
 uint8_t mains_power_gimbal_output : 1;
 uint8_t mains_power_chassis_output : 1;
 uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

typedef __packed struct
{
float x;
float y;
float z;
float yaw;
} ext_game_robot_pos_t;

typedef __packed struct
{
uint8_t power_rune_buff;
}ext_buff_musk_t;

typedef __packed struct
{
uint8_t energy_point;
uint8_t attack_time;
} aerial_robot_energy_t;

typedef __packed struct
{
uint8_t armor_id : 4;
uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct
{
uint8_t bullet_type;
uint8_t bullet_freq;
float bullet_speed;
} ext_shoot_data_t;

typedef __packed struct
{
 uint16_t chassis_volt;            //mV
 uint16_t chassis_current;         //mA
 float chassis_power;              //W
 uint16_t chassis_power_buffer;    //J
 uint16_t shooter_heat0;
 uint16_t shooter_heat1;
 uint16_t mobile_shooter_heat2;
 uint16_t chassis_current_last;
} ext_power_heat_data_t;



typedef struct
{
	ext_supply_projectile_action_t projectile_data;
	ext_event_data_t       event_data;
	ext_buff_musk_t        buff_data;
	ext_game_robot_state_t robot_state;
	ext_power_heat_data_t chassis_power_info;
	ext_shoot_data_t shoot_data;
} receive_judge_t;


extern receive_judge_t judge_rece_mesg;
extern uint8_t  judge_buf[];

void judgementDataHandler(void);
void judgement_task(void const *argu);

#endif
