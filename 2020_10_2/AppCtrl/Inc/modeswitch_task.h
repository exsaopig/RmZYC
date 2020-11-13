#ifndef __MODELSWITCH_TASK_H
#define __MODELSWITCH_TASK_H

#include "stm32f4xx_hal.h"

typedef enum
{
  MANUAL_CTRL_MODE, //���ֶ�
  SEMI_AUTO_MODE,   //���Զ�--����pc���Ƹ�����׼�ʹ�����
  AUTO_CTRL_MODE,   //ȫ�Զ�--�ڱ�
  SAFETY_MODE,      //������
} infantry_mode_e;

void mode_switch_task(void const *argu);

static void get_main_ctrl_mode(void);
static void get_gimbal_mode(void);
static void get_chassis_mode(void);
static void get_shoot_mode(void);

static void get_global_last_mode(void);

#endif

