#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H


#include "stm32f4xx_hal.h"

/* shoot task control period time (ms) */
#define SHOT_TASK_PERIOD 5

typedef enum
{
  SHOT_DISABLE       = 0,
  REMOTE_CTRL_SHOT   = 1,
  KEYBOARD_CTRL_SHOT = 2,
  SEMIAUTO_CTRL_SHOT = 3,
  AUTO_CTRL_SHOT     = 4,
} shoot_mode_e;   //控制方式，不发弹 鼠标 键盘 半自动 自动

typedef enum
{
  TRIG_INIT       = 0,
  TRIG_PRESS_DOWN = 1,
  TRIG_BOUNCE_UP  = 2,
  TRIG_ONE_DONE   = 3,//触发方式？ 初始化，按下，弹起，按一次
} trig_state_e;

typedef enum
{
   SINGLE_SHOOT      = 0,
   TRIPLE_SHOOT      = 1,
  FULL_AUTOSHOOT     = 2,
	AUTO_CTRL_SHOOT    = 3,
} fire_mode;   //

typedef enum
{
	 LEVEL0			 = 0,
   LEVEL1      = 1,
   LEVEL2      = 2,
   LEVEL3      = 3,
} shoot_level;   //

typedef __packed struct
{
	fire_mode firemode;
	shoot_level shootlevel;
  /* shoot task relevant param */
  shoot_mode_e ctrl_mode;  
  uint8_t      shoot_cmd;
  uint32_t     c_shoot_time;   //continuous
  uint8_t      c_shoot_cmd;
  uint8_t      fric_wheel_run; //run or not
  uint16_t     fric_wheel_spd;
	uint16_t     trigger_motor_speed;
  uint16_t     shoot_bullets;
  uint16_t     remain_bullets;
	uint8_t		 empty_bullet_flag;
} shoot_t;




typedef __packed struct
{
  /* trigger motor param */
  int32_t   spd_ref;
  int32_t   pos_ref;
  int8_t    dir;
  uint8_t   key;
  uint8_t   key_last;
  uint32_t  one_time;
  int32_t   feed_bullet_spd;
  int32_t   c_shoot_spd;

  trig_state_e one_sta;
} trigger_t;


void shoot_param_init(void);
void shoot_task(void const *argu);
void block_bullet_handler(void);
void setfric_wheel_spd(void);
static void fric_wheel_ctrl(void);
void turn_on_friction_wheel(uint16_t spd);
void turn_off_friction_wheel(void);
void  empty_judge(void);


extern shoot_t   shoot;
extern trigger_t trig;
extern int32_t left_spd,right_spd;

#endif

