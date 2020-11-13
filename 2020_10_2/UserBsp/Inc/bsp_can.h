#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "can.h"

/* CAN send and receive ID */
typedef enum
{
	//收
  CAN_3510_M1_ID       = 0x201,
  CAN_3510_M2_ID       = 0x202,
  CAN_3510_M3_ID       = 0x203,
  CAN_3510_M4_ID       = 0x204,
  CAN_YAW_MOTOR_ID     = 0x205,
  CAN_PITCH_MOTOR_ID   = 0x206, 
  CAN_TRIGGER_MOTOR_ID = 0x207,
  CAN_CHASSIS_ZGYRO_ID = 0x401,
	CAN2_LEFTFRIC_ID     = 0x201,
	CAN2_RIGHTFRIC_ID    = 0x202,
  //发
  CAN_ZGYRO_RST_ID     = 0x406,
  CAN_CHASSIS_ALL_ID   = 0x200,
  CAN_GIMBAL_ALL_ID    = 0x1ff,
	CAN_TOCAP_ID         = 0x301,
} can_msg_id_e;

/* can receive motor parameter structure */
#define FILTER_BUF 5
typedef struct
{
  uint16_t encoder;
  uint16_t last_encoder;
  
  int16_t  speed_rpm;
	int16_t  speed_rpm_last;
  int16_t  given_current;
	int16_t  given_current_last;
	int16_t  given_current_last_last;

  int32_t  round_count;         //圈数计数
  int32_t  total_encoder;       //总过转过的编码器的值
  int32_t  total_angle;         //总共转过的角度，角度制
  
  uint16_t offset_encoder;      //初始时的位置补偿
  uint32_t massage_count;       //消息计数
	//EC60官方车底盘电机求速率所需
	int32_t  encoder_raw_rate;    //两次编码器求差值
  int32_t  rate_buf[FILTER_BUF];//滤波数据储存
  uint8_t  buf_count;           //滤波数据计数
  int32_t  filter_rate;         //滤波后的值
	
} moto_measure_t;

typedef struct
{
  /* 4 chassis motor current */
  int16_t chassis_current[4];
  /* yaw/pitch/trigger motor current */
  int16_t gimbal_current[3];
	
	int16_t fric_current[2];
} motor_current_t;

extern motor_current_t global_current;

extern moto_measure_t moto_chassis[];
extern moto_measure_t moto_yaw, moto_pitch, moto_trigger,moto_leftfric,moto_rightfric;

void encoder_data_handler(moto_measure_t* moto, CAN_HandleTypeDef* hcan);
void get_moto_offset(moto_measure_t* moto, CAN_HandleTypeDef* hcan);

void can_device_init(void);
void can_receive_start(void);
void gyro_device_init(void);

void send_gimbal_current(int16_t yaw_iq, int16_t pit_iq, int16_t trigger_iq);
void send_chassis_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

void send_gimbal_motor_ctrl_message(int16_t gimbal_cur[],int16_t fric_cur[]);
void send_chassis_motor_ctrl_message(int16_t chassis_cur[]);
void send_cap_instructions(uint8_t open, uint8_t max, float power);

#endif

