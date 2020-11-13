#ifndef __GIMBAL_INFO_H
#define __GIMBAL_INFO_H
#include "stm32f4xx_hal.h"

typedef enum
{
  GIMBAL_RELAX         = 0,//云台“不上电”
  GIMBAL_INIT          = 1,//云台位置初始化
  GIMBAL_NO_ARTI_INPUT = 2,//没有输入
  GIMBAL_FOLLOW_ZGYRO  = 3,//以imu角度为当前角度
  GIMBAL_TRACK_ARMOR   = 4,//追踪装甲
  GIMBAL_PATROL_MODE   = 5,//巡逻
  GIMBAL_SHOOT_BUFF    = 6,//打大神符
  GIMBAL_POSITION_MODE = 7,//位置模式
} gimbal_mode_e;

typedef struct
{
  /* unit: degree角度     */
  float pit_relative_angle;
  float yaw_relative_angle;
  float gyro_angle;
  /* uint: degree/s角速度 */
  float yaw_palstance;
  float pit_palstance;
} gim_sensor_t;

uint8_t read_gimbal_offset(int32_t *pit_offset, int32_t *yaw_offset);
void get_gimbal_info(void);

#endif


