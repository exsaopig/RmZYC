#include "pc_info.h"
#include "judgement_info.h"
#include "bsp_uart.h"
#include "sys_config.h"
#include "gimbal_task.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "shoot_task.h"

UBaseType_t pc_info_stack_surplus;
uint8_t  pc_buf[PC_BUFF_LEN];
uint8_t  debug_buf[PC_BUFF_LEN];
uint8_t  pc_send_buf[SND_MAX_LEN];

int start_time=0;

pc_send_data_t   pc_send_mesg;
pc_recieve_data_t pc_recv_mesg; 

void pc_param2value(pc_number_t *num)
{
	num->value = (num->param_a * 200.0f + num->param_b)/40000.0f * (num->max - num->min) + num->min;
}
void pc_value2param(pc_number_t *num)
{
	//val = min(maxv,max(minv,val)); 
	int p = (int)((num->value-num->min)/(num->max-num->min)*40000.0f); 
	num->param_a = p / 200; 
	num->param_b = p % 200; 
}
void pc_number_renew_value(pc_number_t *num, uint8_t param_a, uint8_t param_b)
{
	num->param_a = param_a;
	num->param_b = param_b;
	pc_param2value(num);
}
void pc_number_renew_param(pc_number_t *num, float value)
{
	num->value = value;
	pc_value2param(num);
}
void pc_param_init(void)
{
	pc_recv_mesg.shoot_contrl_data.shoot_mode = 0;
	pc_recv_mesg.shoot_contrl_data.shoot_speed = 0;
	pc_recv_mesg.yaw_change_angle.max = 60;
	pc_recv_mesg.yaw_change_angle.min = -60;
	pc_recv_mesg.pit_change_angle.max = 30;
	pc_recv_mesg.pit_change_angle.min = -30;
	
	pc_recv_mesg.yaw_target_angle.max = 80; //哨兵500
	pc_recv_mesg.yaw_target_angle.min = -80; //哨兵-500
	pc_recv_mesg.pit_target_angle.max = 50; //哨兵20
	pc_recv_mesg.pit_target_angle.min = -50; //哨兵-60
	
	pc_recv_mesg.chassis_x_speed.max = 1;
	pc_recv_mesg.chassis_x_speed.min = -1;
	pc_recv_mesg.chassis_y_speed.max = 1;
	pc_recv_mesg.chassis_y_speed.min = -1;
	pc_recv_mesg.chassis_w_speed.max = 1;
	pc_recv_mesg.chassis_w_speed.min = -1;
	
	pc_send_mesg.yaw_speed.max = 2*3.1415926f;
	pc_send_mesg.yaw_speed.min = -2*3.1415926f;
	pc_send_mesg.pit_speed.max = 2*3.1415926f;
	pc_send_mesg.pit_speed.min = -2*3.1415926f;
	pc_send_mesg.pit_angle.max = 50; //哨兵同上
	pc_send_mesg.pit_angle.min = -50;
	pc_send_mesg.yaw_angle.max = 80;
	pc_send_mesg.yaw_angle.min = -80;
	pc_send_mesg.position_x.max = 30;
	pc_send_mesg.position_x.min = -30;
	pc_send_mesg.position_y.max = 30;
	pc_send_mesg.position_y.min = -30;
	
	start_time = HAL_GetTick();
	
}
void pc_data_unpack(void)
{
	GREEN_Toggle;
	int i =0;
	for(i=0;i<PC_BUFF_LEN;i++)
	{
		if(pc_buf[i]>200)
		{
			switch(pc_buf[i])
			{
				case GIMBAL_DET_ID_r:
				{
					pc_recv_mesg.pc_gimbal_flag = 1;
					pc_number_renew_value(&pc_recv_mesg.yaw_change_angle,pc_buf[i+1],pc_buf[i+2]);
					//pc_recv_mesg.gimbal_control_data.yaw_ref = gimbal.pid.yaw_angle_fdb + pc_recv_mesg.yaw_change_angle.value;
					//pc_recv_mesg.gimbal_control_data.yaw_ref = gimbal.sensor.yaw_relative_angle + pc_recv_mesg.yaw_change_angle.value;
					pc_number_renew_value(&pc_recv_mesg.pit_change_angle,pc_buf[i+3],pc_buf[i+4]);
					//pc_recv_mesg.gimbal_control_data.pit_ref = gimbal.pid.pit_angle_fdb + pc_recv_mesg.pit_change_angle.value;
					//pc_recv_mesg.gimbal_control_data.pit_ref = gimbal.sensor.pit_relative_angle + pc_recv_mesg.pit_change_angle.value;
					break;
				}
				case SHOOT_SPD_ID_r:
				{
					pc_recv_mesg.pc_shoot_flag = 1;
					pc_recv_mesg.shoot_contrl_data.shoot_speed = pc_buf[i+1];
					break;
				}
				case SHOOT_ID_r:
				{
					pc_recv_mesg.pc_shoot_flag = 1;
					pc_recv_mesg.shoot_contrl_data.shoot_mode  = pc_buf[i+1];
					break;
				}
				case BUFF_DONE_ID_r:
				{
					break;
				}
				case JUDGE_ID_r:
				{
					break;
				}
				case GIMBAL_POS_ID_r:
				{
					pc_recv_mesg.pc_gimbal_flag = 1;
					pc_number_renew_value(&pc_recv_mesg.yaw_target_angle,pc_buf[i+1],pc_buf[i+2]);
					pc_number_renew_value(&pc_recv_mesg.pit_target_angle,pc_buf[i+3],pc_buf[i+4]);	
					pc_recv_mesg.gimbal_control_data.yaw_ref = -pc_recv_mesg.yaw_target_angle.value+gimbal.yaw_buff_offset;
					pc_recv_mesg.gimbal_control_data.pit_ref = pc_recv_mesg.pit_target_angle.value+gimbal.pit_buff_offset;
					break;
				}

				case CHASSIS_ID_r:
				{
					pc_recv_mesg.pc_chassis_flag = 1;
					pc_number_renew_value(&pc_recv_mesg.chassis_x_speed, pc_buf[i+1], pc_buf[i+2]);
					pc_number_renew_value(&pc_recv_mesg.chassis_y_speed, pc_buf[i+3], pc_buf[i+4]);
					pc_number_renew_value(&pc_recv_mesg.chassis_w_speed, pc_buf[i+5], pc_buf[i+6]);
					pc_recv_mesg.chassis_control_data.x_speed = pc_recv_mesg.chassis_x_speed.value;
					pc_recv_mesg.chassis_control_data.y_speed = pc_recv_mesg.chassis_y_speed.value;
					pc_recv_mesg.chassis_control_data.w_speed = pc_recv_mesg.chassis_w_speed.value;
				}
				default:
				{
					break;
				}
			}
			pc_buf[i]=0;
		}
	}
}

void clear_pc_data(void)
{
	pc_recv_mesg.pc_gimbal_flag = 0;
	pc_recv_mesg.pc_shoot_flag = 0;
	pc_recv_mesg.pc_chassis_flag = 0;
	pc_recv_mesg.gimbal_control_data.yaw_ref = gimbal.pid.yaw_angle_fdb;
	pc_recv_mesg.gimbal_control_data.pit_ref = gimbal.pid.pit_angle_fdb;
	pc_recv_mesg.shoot_contrl_data.shoot_mode = 0;
	pc_recv_mesg.shoot_contrl_data.shoot_speed = 0;
	pc_recv_mesg.chassis_control_data.x_speed = 0;
	pc_recv_mesg.chassis_control_data.y_speed = 0;
	pc_recv_mesg.chassis_control_data.w_speed = 0;
}
void pc_info_task(void const *argu)
{
	uint32_t mode_wake_time = osKernelSysTick();    //获取当前系统时间，方便延时用
	while(1)
	{
		
		taskENTER_CRITICAL();
    
		pc_number_renew_param(&pc_send_mesg.yaw_speed, gimbal.sensor.yaw_palstance/57.2957f);
		pc_number_renew_param(&pc_send_mesg.pit_speed, gimbal.sensor.pit_palstance/57.2957f);
		
		pc_number_renew_param(&pc_send_mesg.yaw_angle, -(gimbal.sensor.yaw_relative_angle-gimbal.yaw_buff_offset));
		pc_number_renew_param(&pc_send_mesg.pit_angle, (gimbal.sensor.pit_relative_angle-gimbal.pit_buff_offset));
		
		//pc_number_renew_param(&pc_send_mesg.yaw_angle, pc_recv_mesg.yaw_target_angle.value);
		//pc_number_renew_param(&pc_send_mesg.pit_angle, pc_recv_mesg.pit_target_angle.value);
		/*
		int yaw_spd,pit_spd;
		yaw_spd = (gimbal.sensor.yaw_palstance/360.0f+1.0f)*20000.0f;
		pit_spd = (gimbal.sensor.pit_palstance/360.0f+1.0f)*20000.0f;*/

		/*
		if(judge_rece_mesg.real_shoot_data.bullet_spd>0&&judge_rece_mesg.real_shoot_data.bullet_spd!=last_spd)
		{
			pc_send_buf[0] = SHOOT_SPD_ID_s;
			pc_send_buf[1] = (int)judge_rece_mesg.real_shoot_data.bullet_spd;
			last_spd = judge_rece_mesg.real_shoot_data.bullet_spd;
			HAL_UART_Transmit(&huart6, pc_send_buf, 2, 1000);
		}*/
		pc_send_buf[0] = SHOOT_SPD_ID_s;
		if(shoot.fric_wheel_spd>1200)
			pc_send_buf[1] = (int)(shoot.fric_wheel_spd * 0.0539f - 63.7964f);
		else
			pc_send_buf[1] = 0;
		HAL_UART_Transmit(&huart2, pc_send_buf, 2, 2000);
		
		pc_send_buf[0] = GIMBAL_SPEED_ID_s;
		pc_send_buf[1] = pc_send_mesg.yaw_speed.param_a;
		pc_send_buf[2] = pc_send_mesg.yaw_speed.param_b;
		pc_send_buf[3] = pc_send_mesg.pit_speed.param_a;
		pc_send_buf[4] = pc_send_mesg.pit_speed.param_b;
		HAL_UART_Transmit(&huart2, pc_send_buf, 5, 2000);
		
		pc_send_buf[0] = GIMBAL_POS_ID_s;
		pc_send_buf[1] = pc_send_mesg.yaw_angle.param_a;
		pc_send_buf[2] = pc_send_mesg.yaw_angle.param_b;
		pc_send_buf[3] = pc_send_mesg.pit_angle.param_a;
		pc_send_buf[4] = pc_send_mesg.pit_angle.param_b;
		HAL_UART_Transmit(&huart2, pc_send_buf, 5, 2000);
	
//    if((HAL_GetTick() - start_time) > 240000)
//		{
//			pc_send_buf[0] = 205;
//			pc_send_buf[1] = 1;
//			HAL_UART_Transmit(&huart6, pc_send_buf, 2, 2000);
//		}
//		else
//		{
//			pc_send_buf[0] = 205;
//			pc_send_buf[1] = 0;
//			HAL_UART_Transmit(&huart6, pc_send_buf, 2, 2000);
//		}
		
    		
		taskEXIT_CRITICAL();
		pc_info_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
		osDelayUntil(&mode_wake_time, COMM_TASK_PERIOD);      // 延时一段时间继续执行

  }
	
}
//这个函数用于解析PC串口浮点型数字，将缓冲数组清零
void renew_char(char *c,int len)
{
	uint8_t i;
	for(i=0;i<len;i++)
	{
		c[i]='0';
	}
}
//从pc_info的数组里读出pid参数,存到p,i,d里
//注：pc_info是char型，
//即，收到"1.12,1.22,1.230000",p,i,d分别为1.12,1.22,1.23
void debug_pid(pid_t *pid_debug)
{
	int i =0;
	int j=0;
	int k=0;
  uint32_t maxout_rev=0;
  uint32_t intergral_limit_rev=0;
  float kp_rev=0;
  float ki_rev=0;
  float kd_rev=0;
  
	char *end;                                //strtof函数需要的指针
	char data_buffer[10];                     //储存单个浮点数的缓冲区
	float data_f[10];
	uint8_t begin_flag,dot_flag;
	for(i=0;i<PC_BUFF_LEN;i++)
	{
		if(pc_buf[i]=='U')
		{
			begin_flag = 1;
			continue;
		}
		if(pc_buf[i] == 'E')
		{
			data_f[k++]=strtof(data_buffer, &end);
			renew_char(data_buffer,10);
			j = 0;
			dot_flag = 0;
			begin_flag = 0;
			break;
		}
		if(begin_flag)
		{
			if(pc_buf[i] != ',')
			{
				if(pc_buf[i] == '.')
				{dot_flag = 1;}
				data_buffer[j++] = pc_buf[i];
			}
			else
			{
				if(!dot_flag)
				{
					data_buffer[j++] = '.';
					data_buffer[j] = '0';
				}
				data_f[k++]=strtof(data_buffer, &end);
				renew_char(data_buffer,10);
				j = 0;
				dot_flag = 0;
			}
		}
		pc_buf[i] = '0';
	}
	maxout_rev = data_f[0];
	intergral_limit_rev = data_f[1];
	kp_rev = data_f[2];
	ki_rev = data_f[3];
	kd_rev = data_f[4];
	PID_struct_init(pid_debug,POSITION_PID,maxout_rev,intergral_limit_rev,kp_rev,ki_rev,kd_rev);//建立pid	
}


