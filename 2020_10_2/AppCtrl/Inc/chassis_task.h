#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "stm32f4xx_hal.h"

typedef enum
{
  CHASSIS_RELAX          = 0,//电机“不上电”
  CHASSIS_STOP           = 1,//电机停止
  MANUAL_SEPARATE_GIMBAL = 2,//手动且和云台分离
  MANUAL_FOLLOW_GIMBAL   = 3,//手动且跟随云台
  DODGE_MODE             = 4,//扭屁股模式   
  AUTO_SEPARATE_GIMBAL   = 5,//自动且与云台分离
  AUTO_FOLLOW_GIMBAL     = 6,//自动且跟随云台
} chassis_mode_e;

typedef struct
{
  float           vx; // 前后速度
  float           vy; // 左右速度
  float           vw; // 
  int16_t         rotate_x_offset;
  int16_t         rotate_y_offset;
  
  chassis_mode_e  ctrl_mode;
  chassis_mode_e  last_ctrl_mode;

  float           gyro_angle;
  float           gyro_palstance;

  int16_t         wheel_spd_fdb[4];
  int16_t         wheel_spd_ref[4];
	float           current_fbd[4];
	float           current_ref[4];
  int16_t         current[4];
	int16_t         current_last[4];
	float           current_sum;
	float           current_sum_ref;
  
  int16_t         position_ref;
  uint8_t         follow_gimbal;
} chassis_t;

extern chassis_t chassis;
extern int32_t total_cur_limit;
extern int32_t total_cur;
extern uint8_t twistflag;

uint8_t chassis_is_controllable(void);

void chassis_task(void const *argu);

void chassis_param_init(void);
void power_limit_handler(void);
void update_current(void);

static void chassis_twist_handler(void);
static void chassis_stop_handler(void);
static void separate_gimbal_handler(void);
static void follow_gimbal_handler(void);
static void pc_control_chassis(void);

static void mecanum_calc(float vx, float vy, float vw, int16_t speed[]);

float sinx(float num);
float cosx(float num);

#endif
