#ifndef __ERRORDETECT_TASK_H
#define __ERRORDETECT_TASK_H

#include "stm32f4xx_hal.h"

/* detect task period time (ms) */
#define DETECT_TASK_PERIOD 50

typedef enum
{
  DEV_OFFLINE     = 0,
  DEV_RUNNING_ERR = 1,
  SYS_CONFIG_ERR  = 2,
} err_type_e;
typedef enum
{
  BOTTOM_DEVICE        = 0,
  GIMBAL_GYRO_OFFLINE  = 1,
  CHASSIS_GYRO_OFFLINE = 2,
  CHASSIS_M1_OFFLINE   = 3,
  CHASSIS_M2_OFFLINE   = 4,
  CHASSIS_M3_OFFLINE   = 5,
  CHASSIS_M4_OFFLINE   = 6,
  REMOTE_CTRL_OFFLINE  = 7,
  JUDGE_SYS_OFFLINE    = 8,
  PC_SYS_OFFLINE       = 9,
  GIMBAL_YAW_OFFLINE   = 10,
  GIMBAL_PIT_OFFLINE   = 11,
  TRIGGER_MOTO_OFFLINE = 12,
  BULLET_JAM           = 13,
  CHASSIS_CONFIG_ERR   = 14,
  GIMBAL_CONFIG_ERR    = 15,
  ERROR_LIST_LENGTH    = 16,
} err_id_e;
typedef struct
{
  uint16_t set_timeout;
  /* uint32_t prevent delta_time overflow */
  uint32_t delta_time;
  uint32_t last_time;
} offline_dev_t;

typedef struct
{
  /* enable the device error detect */
  uint8_t  enable;
  /* device error exist flag */
  uint8_t  err_exist;
  /* device error priority */
  uint8_t  pri;
  /* device error type */
  uint8_t  type;
  /* the pointer of device offline param */
  offline_dev_t *dev;
} err_dev_t;

typedef struct
{
  /* the pointer of the highest priority error device */
  err_dev_t *err_now;
  err_id_e  err_now_id;
  /* all device be detected list */
  err_dev_t list[ERROR_LIST_LENGTH];

  /* error alarm relevant */
  uint16_t err_count;
  uint16_t beep_tune;
  uint16_t beep_ctrl;
} global_err_t;
extern global_err_t g_err;

void global_err_detector_init(void);
void err_detector_hook(int err_id);
void detect_task(void const *argu);
void detector_param_init(void);

static void module_offline_callback(void);
static void module_offline_detect(void);





#endif


