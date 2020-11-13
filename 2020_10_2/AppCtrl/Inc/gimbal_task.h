#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "stm32f4xx_hal.h"
#include "gimbal_info.h"

typedef enum
{
  NO_ACTION = 0,
  IS_ACTION,
} action_mode_e;

typedef struct
{
  /* position loop */
  float yaw_angle_ref;//给定值
  float pit_angle_ref;
  float yaw_angle_fdb;//反馈值
  float pit_angle_fdb;
  /* speed loop */
  float yaw_spd_ref;
  float pit_spd_ref;
  float yaw_spd_fdb;
  float pit_spd_fdb;
} gim_pid_t;

typedef struct
{
  action_mode_e ac_mode;
  float         action_angle;
  uint8_t       no_action_flag;
  uint32_t      no_action_time;
} no_action_t;

typedef struct
{
  /* ctrl mode */
  gimbal_mode_e ctrl_mode;
  gimbal_mode_e last_ctrl_mode;
  
  /* gimbal information */
  gim_sensor_t  sensor;
  float         ecd_offset_angle;//yaw轴从放松态变为上电时的偏差
  float         yaw_offset_angle;//yaw轴保持不动时imu的偏差
	float         pitch_offset_angle;
  
	gim_pid_t     pid;
	no_action_t   input;
	
  /* read from flash */
  int32_t       pit_center_offset;
  int32_t       yaw_center_offset;
  
	float         yaw_buff_offset;
	float         pit_buff_offset;
} gimbal_t;

extern gimbal_t gimbal;
extern uint8_t tx_flag;
extern float track_pit,track_yaw;

static void no_action_handler(void);
static void init_mode_handler(void);
static void closed_loop_handler(void);

static void track_aimor_handler(void);
static void gimbal_patrol_handler(void);
static void pc_position_ctrl_handler(void);
static void shoot_buff_handler(void);

void gimbal_param_init(void);
void gimbal_back_param(void);

void gimbal_task(void const *argu);
void gimbal_self_check(void); //自检即初始化

uint8_t gimbal_is_controllable(void);

#endif

