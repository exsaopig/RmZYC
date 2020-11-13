#ifndef __MODELSWITCH_TASK_H
#define __MODELSWITCH_TASK_H

#include "stm32f4xx_hal.h"

typedef enum
{
  MANUAL_CTRL_MODE, //纯手动
  SEMI_AUTO_MODE,   //半自动--允许pc控制辅助瞄准和打大神符
  AUTO_CTRL_MODE,   //全自动--哨兵
  SAFETY_MODE,      //防暴走
} infantry_mode_e;

void mode_switch_task(void const *argu);

static void get_main_ctrl_mode(void);
static void get_gimbal_mode(void);
static void get_chassis_mode(void);
static void get_shoot_mode(void);

static void get_global_last_mode(void);

#endif

