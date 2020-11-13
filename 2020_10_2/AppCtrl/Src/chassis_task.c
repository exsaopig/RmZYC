#include "chassis_task.h"
#include "bsp_io.h"
#include "bsp_can.h"
#include "gimbal_task.h"
#include "gimbal_info.h"
#include "remote_info.h"
#include "pc_info.h"
#include "keyboard_info.h"
#include "judgement_info.h"
#include "errordetect_task.h"
#include "pid.h"
#include "sys_config.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "cmsis_os.h"
#include "comm_task.h"
#include "sys_config.h"
#include "ramp.h"
#include "iwdg.h"
#include "math.h"

/* chassis twist angle (degree)*/
#define TWIST_ANGLE    30
/* twist period time (ms) */
#define TWIST_PERIOD   1300
/* warning surplus energy */
#define WARNING_ENERGY judge_rece_mesg.robot_state.max_chassis_power

/* chassis task global parameter */
chassis_t chassis;
uint32_t chassis_time_last=0;
int chassis_time_ms=0;
extern TaskHandle_t can_msg_send_task_t;
UBaseType_t chassis_stack_surplus;
uint8_t twistflag = 0;
//static int16_t twist_period = TWIST_PERIOD/CHASSIS_PERIOD;
//static int16_t twist_angle  = TWIST_ANGLE;
const float sinlist[91] = {0,0.0174524,0.0348995,0.052336,0.0697565,0.0871557,0.104528,0.121869,0.139173,0.156434,
					  0.173648,0.190809,0.207912,0.224951,0.241922,0.258819,0.275637,0.292372,0.309017,0.325568,
				 	  0.34202,0.358368,0.374607,0.390731,0.406737,0.422618,0.438371,0.453991,0.469472,0.48481,
					  0.5,0.515038,0.529919,0.544639,0.559193,0.573577,0.587785,0.601815,0.615662,0.62932,0.642788,
					  0.656059,0.669131,0.681998,0.694658,0.707107,0.71934,0.731354,0.743145,0.75471,0.766044,0.777146,
					  0.788011,0.798636,0.809017,0.819152,0.829038,0.838671,0.848048,0.857167,0.866025,0.87462,0.882948,
					  0.891007,0.898794,0.906308,0.913545,0.920505,0.927184,0.93358,0.939693,0.945519,0.951057,0.956305,
					  0.961262,0.965926,0.970296,0.97437,0.978148,0.981627,0.984808,0.987688,0.990268,0.992546,0.994522,
					  0.996195,0.997564,0.99863,0.999391,0.999848,1};
float abs_sum4(float a,float b,float c, float d)
{
	float tmp=0;
	tmp += a>=0?a:(-a);
	tmp += b>=0?b:(-b);
	tmp += c>=0?c:(-c);
	tmp += d>=0?d:(-d);
	return tmp;
}

uint8_t chassis_is_controllable(void)
{
  if (chassis.ctrl_mode == CHASSIS_RELAX 
   || g_err.list[REMOTE_CTRL_OFFLINE].err_exist)
    return 0;
  else
    return 1;
}

void chassis_task(void const *argu)
{
	
	
	chassis_time_ms = HAL_GetTick() - chassis_time_last;
  chassis_time_last = HAL_GetTick();
	HAL_IWDG_Refresh(&hiwdg);
  switch (chassis.ctrl_mode)
  {
    case DODGE_MODE:           //扭屁股
    {
//			taskENTER_CRITICAL();				//进入临界区
      chassis_twist_handler();
//			taskEXIT_CRITICAL();				//退出临界区
    }break;
    
    case AUTO_FOLLOW_GIMBAL:  //全自动且跟随云台（全自动未添加）
    {
      chassis.position_ref = 0;
      chassis.vw = -pid_calc(&pid_chassis_angle, gimbal.sensor.yaw_relative_angle, chassis.position_ref); 
			
    }break;
    
    case AUTO_SEPARATE_GIMBAL: //全自动且分离云台（全自动未添加）
    {
      chassis.position_ref = 0;
			pc_control_chassis();
    }break;
    
    case CHASSIS_STOP:         //停止
    {
      chassis_stop_handler();
    }break;

    case MANUAL_SEPARATE_GIMBAL: //手动控制且与云台分离
    {
      separate_gimbal_handler();
    }break;
    
    case MANUAL_FOLLOW_GIMBAL:   //手动控制且跟随云台
    {
      follow_gimbal_handler();
    }break;

    default:
    {
      chassis_stop_handler();
    }break;
  }
//	if(judge_rece_mesg.chassis_power_info.chassis_current != 0.0f)
//		update_current();
  mecanum_calc(chassis.vx, chassis.vy, chassis.vw, chassis.wheel_spd_ref); //麦克纳姆轮速度解算
  for (int i = 0; i <4; i++)
  {
    chassis.current_ref[i] = pid_calc(&pid_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);
  }
	
	chassis.current_sum_ref = abs_sum4(chassis.current_ref[0],chassis.current_ref[1],
	                                   chassis.current_ref[2],chassis.current_ref[3])/1000.0f;
	
//	if(judge_rece_mesg.chassis_power_info.chassis_current != 0.0f)
//	{
//		power_limit_handler();
//		for(int i = 0;i <4; i++)
//		{
//			chassis.current[i]     = pid_calc(&pid_current[i], chassis.current_fbd[i], chassis.current_ref[i]);
//		}
//  }
//	else
//	{
		for(int i = 0;i <4; i++)
		{
			chassis.current[i]     = chassis.current_ref[i];
		}
//	}
  if (!chassis_is_controllable())
  {
    memset(chassis.current, 0, sizeof(chassis.current));
  }
  memcpy(global_current.chassis_current, chassis.current, sizeof(chassis.current)); //将chssis中层控制结构输出赋值到直接发给can的底层输出
	
	osSignalSet(can_msg_send_task_t, CHASSIS_MOTOR_MSG_SEND);
	
  chassis_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	
}

void chassis_stop_handler(void)
{
  chassis.vy = 0;
  chassis.vx = 0;
  chassis.vw = 0;
}
uint32_t twist_count = 0;
//static void chassis_twist_handler(void)
//{
//	/*
//	此时车身角度应为yaw轴角度
//	*/
//	if(twist_count<60)
//  chassis.position_ref = twist_angle*mod_sin[twist_count];
//	if(twist_count<119&&twist_count>59)
//	chassis.position_ref = -twist_angle*mod_sin[119-twist_count];
//  if(twist_count==120)
//	twist_count=0;
//	twist_count++;
//  //chassis.position_ref = twist_angle*sin(2*PI/twist_period*twist_count);
//  chassis.vw = -pid_calc(&pid_chassis_angle, gimbal.sensor.yaw_relative_angle, chassis.position_ref);
//	//chassis.vw = 0;
//}
static void chassis_twist_handler(void)
{
	/*
	此时车身角度应为yaw轴角度
	*/
//  static int16_t twist_period = TWIST_PERIOD/CHASSIS_PERIOD;
//  static int16_t twist_angle  = TWIST_ANGLE;
//  twist_count++;
//  chassis.position_ref = twist_angle*sin(2*PI/twist_period*twist_count);
//  chassis.vw = -pid_calc(&pid_chassis_angle, gimbal.sensor.yaw_relative_angle, chassis.position_ref);
	static float tempvx,tempvy;
	tempvy = rm.vy * CHASSIS_RC_MOVE_RATIO_Y + km.vy * CHASSIS_KB_MOVE_RATIO_Y; //rm为遥控器控制量，km为键鼠控制量，以云台为基准计算vy
	tempvx = rm.vx * CHASSIS_RC_MOVE_RATIO_X + km.vx * CHASSIS_KB_MOVE_RATIO_X; //以云台为基准计算vx
//	ang = PI * gimbal.sensor.yaw_relative_angle / 180;  //计算yaw轴相对角度，运动方向出问题改正负
	chassis.vx = cosx(gimbal.sensor.yaw_relative_angle) * tempvx - sinx(gimbal.sensor.yaw_relative_angle) * tempvy; //以底盘为基准计算vx 乘旋转矩阵
	chassis.vy = sinx(gimbal.sensor.yaw_relative_angle) * tempvx + cosx(gimbal.sensor.yaw_relative_angle) * tempvy; //以底盘为基准计算vy
	chassis.vw = 200;                                   //叠加vw
}
void separate_gimbal_handler(void)
{
  chassis.vy = rm.vy * CHASSIS_RC_MOVE_RATIO_Y + km.vy * CHASSIS_KB_MOVE_RATIO_Y;
  chassis.vx = rm.vx * CHASSIS_RC_MOVE_RATIO_X + km.vx * CHASSIS_KB_MOVE_RATIO_X;
  chassis.vw = rm.vw * CHASSIS_RC_MOVE_RATIO_R;
}
void follow_gimbal_handler(void)
{
  chassis.position_ref = 0;
  
  chassis.vy = rm.vy * CHASSIS_RC_MOVE_RATIO_Y + km.vy * CHASSIS_KB_MOVE_RATIO_Y; //rm为遥控器控制量，km为键鼠控制量
  chassis.vx = rm.vx * CHASSIS_RC_MOVE_RATIO_X + km.vx * CHASSIS_KB_MOVE_RATIO_X;

  if (chassis.follow_gimbal) //底盘是否跟随云台的最终决定量为chssis.follow_gimbal，跟随云台即pid控制底盘与yaw夹角为0
	{
		if(twistflag == 1) chassis.position_ref = gimbal.sensor.yaw_relative_angle * (1 - ramp_calc(&twist_ramp));
		if(chassis.position_ref == 0) twistflag = 0;
    chassis.vw = -pid_calc(&pid_chassis_angle, gimbal.sensor.yaw_relative_angle, chassis.position_ref);
	}
  else
    chassis.vw = 0;
}
void pc_control_chassis(void)        //207协议控制chassis(哨兵同时适用)
{
	/*
	                       前
                       	 ↑
	                       ↑	
	              左 ← ← 64 00 → → 右       前两个字节控制左右移动
                       	 ↓                后两个字节控制前后移动
	                       ↓                最后两个字节控制底盘角度
	                       后
	
	
	*/
	 
	chassis.vy = -pc_recv_mesg.chassis_control_data.x_speed*CHASSIS_RC_MAX_SPEED_X;  //PC传输的XY与实际标定相反
	chassis.vx = pc_recv_mesg.chassis_control_data.y_speed*CHASSIS_RC_MAX_SPEED_Y;
	chassis.vw = pc_recv_mesg.chassis_control_data.w_speed*CHASSIS_RC_MOVE_RATIO_R;
	

  //chassis.vx = 	-pc_recv_mesg.chassis_control_data.x_speed*CHASSIS_RC_MAX_SPEED_X;   //哨兵通信协议
}


/**
  * @brief mecanum chassis velocity decomposition
  * @param input : ↑=+vx(mm/s)  ←=+vy(mm/s)  ccw=+vw(deg/s)
  *        output: every wheel speed(rpm)
  * @note  1=FR 2=FL 3=BL 4=BR
  */
int rotation_center_gimbal = 0;
void mecanum_calc(float vx, float vy, float vw, int16_t speed[])
{
  static float rotate_ratio_fr;
  static float rotate_ratio_fl;
  static float rotate_ratio_bl;
  static float rotate_ratio_br;
  static float wheel_rpm_ratio;
  
  if(chassis.ctrl_mode == DODGE_MODE)
  {
    chassis.rotate_x_offset = GIMBAL_X_OFFSET;  //车身中心和云台距离
    chassis.rotate_y_offset = 0;
  }
  else
  {
    chassis.rotate_x_offset = 0;
    chassis.rotate_y_offset = 0;
  }
  
  rotate_ratio_fr = ((WHEELBASE + WHEELTRACK)/2.0f - chassis.rotate_x_offset + chassis.rotate_y_offset)/RADIAN_COEF;
  rotate_ratio_fl = ((WHEELBASE + WHEELTRACK)/2.0f - chassis.rotate_x_offset - chassis.rotate_y_offset)/RADIAN_COEF;
  rotate_ratio_bl = ((WHEELBASE + WHEELTRACK)/2.0f + chassis.rotate_x_offset - chassis.rotate_y_offset)/RADIAN_COEF;
  rotate_ratio_br = ((WHEELBASE + WHEELTRACK)/2.0f + chassis.rotate_x_offset + chassis.rotate_y_offset)/RADIAN_COEF;

  wheel_rpm_ratio = 60.0f/(PERIMETER * CHASSIS_DECELE_RATIO);
  
  
  VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
  VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
  VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s
  
  int16_t wheel_rpm[4];
  float   max = 0;

	wheel_rpm[MOTOR_A] = -( vx - vy - vw * rotate_ratio_fr) * wheel_rpm_ratio; //right 
  wheel_rpm[MOTOR_B] = -(-vx - vy - vw * rotate_ratio_fl) * wheel_rpm_ratio; //
  wheel_rpm[MOTOR_C] = -( vx + vy - vw * rotate_ratio_bl) * wheel_rpm_ratio;
  wheel_rpm[MOTOR_D] = -(-vx + vy - vw * rotate_ratio_br) * wheel_rpm_ratio;
  //find max item
  for (uint8_t i = 0; i < 4; i++)
  {
    if (abs(wheel_rpm[i]) > max)
      max = abs(wheel_rpm[i]);
  }
  //equal proportion
  if (max > MAX_WHEEL_RPM)
  {
    float rate = MAX_WHEEL_RPM / max;
    for (uint8_t i = 0; i < 4; i++)
      wheel_rpm[i] *= rate;
  }
  memcpy(speed, wheel_rpm, 4*sizeof(int16_t));
}



/**
  * @brief  nitialize chassis motor pid parameter
  * @usage  before chassis loop use this function
  */
void chassis_param_init(void)
{
  memset(&chassis, 0, sizeof(chassis_t)); //chassis控制结构体归零
  
  chassis.ctrl_mode      = CHASSIS_STOP;  //底盘控制模式为停止
  chassis.last_ctrl_mode = CHASSIS_RELAX; //底盘上次模式为放松态

	//pid参数初始化
#ifdef CHASSIS_EC60
  for (int k = 0; k < 4; k++)
  {
    PID_struct_init(&pid_spd[k], POSITION_PID, 10000, 2500, 25, 1.2, 0);
  }
#else
  for (int k = 0; k < 4; k++)
  {
    PID_struct_init(&pid_spd[k], POSITION_PID, 15000, 500, 6.5f, 0.1, 0);
		PID_struct_init(&pid_current[k], POSITION_PID, 10000, 1000, 1.23f, 0, 0);
  }
#endif
#if (ROBOT == 1)
  PID_struct_init(&pid_chassis_angle, POSITION_PID, MAX_CHASSIS_VR_SPEED, 50, 9.0f, 0.0f, 3.0f);
#else
	PID_struct_init(&pid_chassis_angle, POSITION_PID, MAX_CHASSIS_VR_SPEED, 50, 5.0f, 0.00f, 150.0f);
#endif
  //14 50
  //memset(&pc_recv_mesg.structure_data, 0, sizeof(pc_recv_mesg.structure_data));
}

//另一个功率限制，来自一个热爱MATLAB的老人
float i_last[4];
float current_last[4];
float i_last_sum;
float m=0.0f,m_last=0.0f,m_last_last=0.0f;
float chassis_volt,chassis_current,chassis_current_last;
void update_current(void)
{
	chassis_volt = judge_rece_mesg.chassis_power_info.chassis_volt / 1000.0f;
	chassis_current = judge_rece_mesg.chassis_power_info.chassis_current / 1000.0f;
	chassis_current_last = judge_rece_mesg.chassis_power_info.chassis_current_last / 1000.0f;  //mA,mV转A,V
	int i=0;
	for(i=0;i<4;i++)
	{
		i_last[i] = (float)(moto_chassis[i].given_current_last)/819.2f*
			          (float)(moto_chassis[i].speed_rpm_last)/19.0f;
		if(chassis_volt<10.0f)
		{
			i_last[i] = i_last[i]/23.0f;
		}
		else
		{
			i_last[i] = i_last[i]/chassis_volt;
		}
		i_last[i] = i_last[i]>0?i_last[i]:(-i_last[i]);
	}
	i_last_sum = i_last[0]+i_last[1]+i_last[2]+i_last[3];
	if(i_last_sum<0.5f)
	{
		m = m_last;
	}
	else
	{
		m = (chassis_current_last-0.23f)/i_last_sum;
	}
	m = 0.2f*m+0.3f*m_last+0.5f*m_last_last;
	m = m>0? m:(-m);
	m_last_last = m_last;
	m_last = m;
	
	for(i=0;i<4;i++)
	{
		current_last[i] = chassis.current_fbd[i];
		chassis.current_fbd[i] = m/15.5648f*(float)moto_chassis[i].given_current*(float)moto_chassis[i].speed_rpm;
		if(chassis_volt<10.0f)
		{
			chassis.current_fbd[i] = chassis.current_fbd[i]/23.0f;
		}
		else
		{
			chassis.current_fbd[i] = chassis.current_fbd[i]/chassis_volt;
		} 
		chassis.current_fbd[i] = 0.3f*current_last[i]+0.7f*chassis.current_fbd[i];
	}
	chassis.current_sum = abs_sum4(chassis.current_fbd[0],chassis.current_fbd[1],
	                               chassis.current_fbd[2],chassis.current_fbd[3])
	                               /1000.0f+0.75f;
}
void power_limit_handler(void)
{
	int i=0;
	if(judge_rece_mesg.chassis_power_info.chassis_power>WARNING_ENERGY)
	{
		//printf("gethere1\r\n");
		if(chassis.current_sum<1.0f||chassis_volt<10.0f)
		{
			for(i=0;i<4;i++)
			{
			  chassis.current_ref[i] = chassis.current_ref[i]*0.5f;
			}
		}
		else
		{
			for(i=0;i<4;i++)
			{
				chassis.current_ref[i] = chassis.current_ref[i]*(WARNING_ENERGY*0.25f)
																 /chassis.current_sum/chassis_volt;
			}
		}
	}
	else if(chassis_volt*chassis.current_sum>WARNING_ENERGY)
	{
		//printf("gethere2\r\n");
		if(chassis.current_sum<1.0f||chassis_volt<10.0f)
		{
			for(i=0;i<4;i++)
			{
			  chassis.current_ref[i] = chassis.current_ref[i]*0.5f;
			}
		}
		else
		{
			for(i=0;i<4;i++)
			{
				
				chassis.current_ref[i] = chassis.current_ref[i]*(WARNING_ENERGY*0.5f)
																 /chassis.current_sum/chassis_volt;
			}
		}
	}
	else if(chassis_volt*chassis.current_sum_ref>WARNING_ENERGY)
	{
		if(chassis.current_sum<1.0f||chassis_volt<10.0f)
		{
			for(i=0;i<4;i++)
			{
			  chassis.current_ref[i] = chassis.current_ref[i]*0.5f;
			}
		}
		else
		{
			for(i=0;i<4;i++)
			{
				
				chassis.current_ref[i] = chassis.current_ref[i]*(WARNING_ENERGY*0.75f)
																 /chassis.current_sum/chassis_volt;
			}
		}
	}
	
}

float sinx(float num){
	if(num >= 0){
		if(num > 90)
			return sinlist[180 - (int)(num)];
		else
			return sinlist[(int)(num)];
			
	}
	else{
		if(num < -90)
			return -sinlist[180 + (int)(num)];
		else 
			return -sinlist[-(int)(num)];
	}
}

float cosx(float num){
	if(num < - 90){
		return -sinx(- num - 90);
	}
	else{
		return (sinx(90 - num));		
	}
}

