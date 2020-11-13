#ifndef __CALIBRATE_INFO_H
#define __CALIBRATE_INFO_H

#include "stm32f4xx_hal.h"

#define CALIED_FLAG 0x55

typedef enum 
{
  CALI_GYRO    = 0,
  CALI_ACC     = 1,
  CALI_MAG     = 2,
  CALI_IMU_NUM = 3,
} cali_imu_e;

typedef enum
{
  GIMBAL_CALI_START = 1,
  GIMBAL_CALI_END   = 2,
  CAMERA_CALI_START = 3,
  CAMERA_CALI_END   = 4,
} gimbal_cali_type_t;

typedef enum
{
  CALI_GIMBAL_CENTER = 0,
  CALI_CAMERA_CENTER = 1,
  CALI_GIMBAL_NUM    = 2,
} cali_gimbal_e;

typedef __packed struct
{
  int32_t yaw_offset;
  int32_t pitch_offset;
  uint8_t cali_cmd;    //1:calibrate  0:no operate
  uint8_t calied_done; //0x55:already calied
} gim_cali_t;

typedef __packed struct
{
  int16_t offset[3];   //x,y,z
  uint8_t cali_cmd;
  uint8_t calied_done;
  char*   name;
} imu_cali_t;

typedef __packed struct
{
  uint32_t   firmware_version;
  gim_cali_t gim_cali_data[CALI_GIMBAL_NUM];
  imu_cali_t imu_cali_list[CALI_IMU_NUM];
} cali_sys_t;

extern cali_sys_t cali_param;

void gimbal_cali_hook(int32_t pit_ecd, int32_t yaw_ecd);
void imu_cali_hook(cali_imu_e cali_id, int16_t raw_xyz[]);
void cali_param_init(void);
void cali_data_read(void);

#endif

