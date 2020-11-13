#ifndef __PC_INFO_H
#define __PC_INFO_H

#include "stm32f4xx_hal.h"
#include "pid.h"     

#define REV_MAX_LEN 5
#define SND_MAX_LEN 5


#define	GIMBAL_DET_ID_r    201 //云台相对转角（从当前位置转的角度）
#define	SHOOT_SPD_ID_r     202 //建议发射速度
#define	SHOOT_ID_r         203 //发射机构控制
#define	BUFF_DONE_ID_r     204 //打击大神符完成（没做）
#define	JUDGE_ID_r         205 //裁判系统判断红蓝方（没做）
#define GIMBAL_POS_ID_r    206 //云台绝对转角（即直接转到这个角度）
#define CHASSIS_ID_r       207 //pc控制底盘

#define	PC_MODE_ID_s       201
#define	JUDGE_ID_s         202
#define	SHOOT_SPD_ID_s     203
#define	GIMBAL_SPEED_ID_s  204
#define	BUFF_DONE_ID_s     205
#define GIMBAL_POS_ID_s    206
#define POSITION_ID_s      207 //尝试从裁判系统获取位置

#define SHOOT_OFF     1
#define SHOOT_FRIC_ON 2
#define SHOOT_ON      4


#define POSITION_MODE 1
#define SPEED_MODE    2

typedef struct
{
	uint8_t param_a;
	uint8_t param_b;
	float   max;
	float   min;
	float   value;
}pc_number_t;

typedef struct
{
	uint8_t mode;   
  float   pit_ref;      
  float   yaw_ref;	
}gimbal_ctrl_t;

typedef struct
{
	float  x_speed;   //左右速度，右正左负
  float  y_speed;	  //前后速度，前正后负
	float  w_speed;   //顺时针为正
}chassis_ctrl_t;    //哨兵只有x

typedef struct
{
	uint16_t shoot_speed;
	uint8_t  shoot_mode;
}shoot_ctrl_t;

typedef struct
{
	pc_number_t   yaw_speed;
	pc_number_t   pit_speed;
	pc_number_t   yaw_angle;
	pc_number_t   pit_angle;
	pc_number_t   position_x;
	pc_number_t   position_y;
	
}pc_send_data_t;

typedef struct
{
	uint8_t pc_gimbal_flag;
	uint8_t pc_chassis_flag;
	uint8_t pc_shoot_flag;
	
	pc_number_t   yaw_change_angle;
	pc_number_t   pit_change_angle;
	pc_number_t   yaw_target_angle;
	pc_number_t   pit_target_angle;
	pc_number_t   chassis_x_speed;
	pc_number_t   chassis_y_speed;
	pc_number_t   chassis_w_speed;
	
	gimbal_ctrl_t gimbal_control_data;
	shoot_ctrl_t shoot_contrl_data;
	chassis_ctrl_t chassis_control_data;
}pc_recieve_data_t;

extern uint8_t  pc_buf[];
extern uint8_t  pc_send_buf[];
extern uint8_t  debug_buf[];
extern pc_send_data_t   pc_send_mesg;
extern pc_recieve_data_t pc_recv_mesg;
extern int start_time;

void pc_param2value(pc_number_t *num); //用于由ab参数系转换至实际值
void pc_value2param(pc_number_t *num); //用于将实际值转换至ab参数系
void pc_number_renew_value(pc_number_t *num, uint8_t param_a, uint8_t param_b);
void pc_number_renew_param(pc_number_t *num, float value);
void pc_param_init(void);

void pc_data_unpack(void);
void pc_data_pack(uint8_t id);
void send_pc_data(void);
void clear_pc_data(void);
void pc_info_task(void const *argu);

void debug_pid(pid_t *pid_debug);

#endif

