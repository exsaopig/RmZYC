#include "gimbal_task.h"
#include "stdlib.h"
#include "string.h"
#include "pid.h"
#include "ramp.h"
#include "math.h"
#include "usart.h"
#include "bsp_can.h"
#include "chassis_task.h"
#include "shoot_task.h"
#include "errordetect_task.h"
#include "gimbal_info.h"
#include "calibrate_info.h"
#include "remote_info.h"
#include "keyboard_info.h"
#include "pc_info.h"
#include "judgement_info.h"
#include "cmsis_os.h"
#include "comm_task.h"
#include "sys_config.h"

/* gimbal patrol angle (degree)*/
#define PATROL_ANGLE     40
/* patrol period time (ms) */
#define PATROL_PERIOD    1500
/* gimbal back center time (ms) */
#define BACK_CENTER_TIME 2500

gimbal_t gimbal;
UBaseType_t gimbal_stack_surplus;
extern TaskHandle_t shoot_task_t;
extern TaskHandle_t can_msg_send_task_t;
/* control ramp parameter */
static ramp_t     yaw_ramp = RAMP_GEN_DAFAULT;
static ramp_t     pit_ramp = RAMP_GEN_DAFAULT;

uint32_t gimbal_time_last=0;
int gimbal_time_ms=0;
uint32_t patrol_count=0;
float track_pit=0.0,track_yaw=0.0;
uint8_t track_once=0;

void gimbal_self_check(void)
{
  if ( !read_gimbal_offset(&(gimbal.pit_center_offset), &(gimbal.yaw_center_offset)) )
  {
    /* gimbal has not been calibrated */
		//校准程序还没写，所以这里还不会执行
#ifdef CALIBRATE
    no_cali_data_handler();
#endif
  }
}
uint8_t gimbal_is_controllable(void)
{
  if (gimbal.ctrl_mode == GIMBAL_RELAX
   || g_err.list[REMOTE_CTRL_OFFLINE].err_exist
   || g_err.list[GIMBAL_YAW_OFFLINE].err_exist
   || g_err.list[GIMBAL_PIT_OFFLINE].err_exist)
    return 0;
  else
    return 1;
}
void gimbal_task(void const *argu)
{
	taskENTER_CRITICAL();				//进入临界区
	
	gimbal_time_ms = HAL_GetTick() - gimbal_time_last;
  gimbal_time_last = HAL_GetTick();
   
/***********************test***************************************/


//	  taskENTER_CRITICAL();
//    printf("gimbal_time run \r\n");
//		taskEXIT_CRITICAL(); 
	
	
	
	
	//模式控制，所有云台位置当前值和指定值改动应当在下面发生
	if(gimbal.ctrl_mode != GIMBAL_TRACK_ARMOR)
	{
		track_pit = 0;
		track_yaw = 0;
		track_once = 0;
	}
	switch (gimbal.ctrl_mode)
  {
    case GIMBAL_INIT:           //初始回正模式
      init_mode_handler();
    break;
    
    case GIMBAL_NO_ARTI_INPUT:  //没有输入的情况
      no_action_handler();
    break;

    case GIMBAL_FOLLOW_ZGYRO:   //云台跟随板载陀螺仪
      closed_loop_handler();
    break;

    case GIMBAL_TRACK_ARMOR:    //辅助瞄准
      track_aimor_handler();
    break;

    case GIMBAL_PATROL_MODE:    //哨兵巡逻模式
      gimbal_patrol_handler();
    break;
		
		case GIMBAL_SHOOT_BUFF:     //打大神符
		  shoot_buff_handler();
		break;

    case GIMBAL_POSITION_MODE:  //PC控制云台位置（编码器位置）
      pc_position_ctrl_handler();
    break;

    default:
    break;
  }
	
	if(cover_state)    gimbal.pid.pit_angle_ref = 0;                         //如果弹仓盖打开，pitch轴回正
	
	
//	gimbal.pid.pit_angle_ref=0;
//	gimbal.pid.yaw_angle_ref=0;
////	gimbal.pid.pit_angle_fdb=gimbal.sensor.pit_relative_angle;
//	gimbal.pid.yaw_angle_fdb=gimbal.sensor.yaw_relative_angle;
  pid_calc(&pid_yaw, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_ref);  //yaw位置环计算输出速度
  pid_calc(&pid_pit, gimbal.pid.pit_angle_fdb, gimbal.pid.pit_angle_ref);  //pitch位置环计算输出速度
  gimbal.pid.yaw_spd_ref = pid_yaw.out;                                    //yaw速度指定值设置为位置环pid输出
  gimbal.pid.pit_spd_ref = pid_pit.out;                                    //pitch速度指定值设置为位置环pid输出
	
  gimbal.pid.yaw_spd_fdb = gimbal.sensor.yaw_palstance;                    //yaw角速度当前值更新,来自imu
  gimbal.pid.pit_spd_fdb = gimbal.sensor.pit_palstance;                    //pitch角速度当前值更新，来自imu
	
  pid_calc(&pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);  //yaw速度环计算输出电流
  pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);  //pitch速度环计算输出电流

  /* 检测云台是否可控
     可控标准： 云台没有离线，遥控系统没有离线，云台模式不是relax
     检测离线还没有写，这里暂时一直可以进	*/
  if (gimbal_is_controllable())
	{
    global_current.gimbal_current[0] = -YAW_MOTO_POSITIVE_DIR*pid_yaw_spd.out; //yaw输出，前面为安装方向控制
    global_current.gimbal_current[1] = -PIT_MOTO_POSITIVE_DIR*pid_pit_spd.out; //pitch输出，前面为安装方向控制
    global_current.gimbal_current[2] = pid_trigger_spd.out;                   //拨弹电机输出
		global_current.fric_current[0] = pid_leftfric_spd.out;
		global_current.fric_current[1] = pid_rightfric_spd.out;
  }
  else
  {
    memset(global_current.gimbal_current, 0, sizeof(global_current.gimbal_current));
    gimbal.ctrl_mode = GIMBAL_RELAX;
    pid_trigger.iout = 0;
  }
	
	taskEXIT_CRITICAL();				//退出临界区
	
	osSignalSet(can_msg_send_task_t, GIMBAL_MOTOR_MSG_SEND);
  osSignalSet(shoot_task_t, SHOT_TASK_EXE_SIGNAL);

  gimbal_stack_surplus = uxTaskGetStackHighWaterMark(NULL);

}

void init_mode_handler(void)
{
	/* 更新pitch当前值，按照斜坡函数设定pitch指定值 */
  gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;                             
  gimbal.pid.pit_angle_ref = gimbal.sensor.pit_relative_angle * (1 - ramp_calc(&pit_ramp));
	/* 更新yaw当前值，在pitch抬起结束前保持不动 */
  gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_relative_angle;
  gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_relative_angle;
  if(gimbal.pid.pit_angle_fdb >= -3.0f && gimbal.pid.pit_angle_fdb <= 3.0f)
  {
    /* pitch回正后yaw开始回正，过程与pitch相同 */
    gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_relative_angle * ( 1 - ramp_calc(&yaw_ramp));
    if (gimbal.pid.yaw_angle_fdb >= -2.5f && gimbal.pid.yaw_angle_fdb <= 2.5f)
    {
      /* yaw轴回正后设置云台工作模式为没有输入态 */
      gimbal.ctrl_mode = GIMBAL_NO_ARTI_INPUT;
			/* 设定陀螺仪当前角度为偏差值，之后更新yaw轴角度减去这个偏差，保证每一次yaw轴受控移动后指向的方向为0度 */
      gimbal.yaw_offset_angle = gimbal.sensor.gyro_angle;
      gimbal.pid.pit_angle_ref = 0;
      gimbal.pid.yaw_angle_ref = 0;
    }
  }
}

void no_action_handler(void)
{
  if (gimbal.input.no_action_flag == 1) //完成上次指令
  {
    if ((HAL_GetTick() - gimbal.input.no_action_time) < 1500)
    {
      closed_loop_handler();
    }
    else //完成上次输入指令后超过1500ms yaw回正
    {
      gimbal.input.no_action_flag = 2; 
      gimbal.pid.yaw_angle_ref = 0;
    }
  }
  
  if (gimbal.input.no_action_flag == 2) //没有指令输入且已完成yaw回正，yaw轴以编码器控制，搬车时yaw保持在中心
  {
    chassis.follow_gimbal = 0;
    gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
    gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_relative_angle;
  }
}

void closed_loop_handler(void)
{ 
  static float chassis_angle_tmp = 0;
  static float limit_angle_range = 2;
  
  gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
	gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_relative_angle;
	gimbal.pid.yaw_angle_fdb = gimbal.sensor.gyro_angle - gimbal.yaw_offset_angle;
	
  /* chassis angle relative to gim.pid.yaw_angle_fdb */
  chassis_angle_tmp = gimbal.pid.yaw_angle_fdb - gimbal.sensor.yaw_relative_angle;\

  /* limit gimbal yaw axis angle */
  if ((gimbal.sensor.yaw_relative_angle >= YAW_ANGLE_MIN - limit_angle_range) && \
      (gimbal.sensor.yaw_relative_angle <= YAW_ANGLE_MAX + limit_angle_range))
  {
    gimbal.pid.yaw_angle_ref += rm.yaw_v * GIMBAL_RC_MOVE_RATIO_YAW + km.yaw_v * GIMBAL_PC_MOVE_RATIO_YAW;
    VAL_LIMIT(gimbal.pid.yaw_angle_ref, chassis_angle_tmp + YAW_ANGLE_MIN, chassis_angle_tmp + YAW_ANGLE_MAX);
  }
	else
	{
		gimbal.pid.yaw_angle_ref -= gimbal.pid.yaw_angle_ref>0?1.0f:-1.0f;
	}
  /* limit gimbal pitch axis angle */
  if ((gimbal.sensor.pit_relative_angle >= PIT_ANGLE_MIN - limit_angle_range) && \
      (gimbal.sensor.pit_relative_angle <= PIT_ANGLE_MAX + limit_angle_range))
  {
    gimbal.pid.pit_angle_ref += rm.pit_v * GIMBAL_RC_MOVE_RATIO_PIT + km.pit_v * GIMBAL_PC_MOVE_RATIO_PIT;
    VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
  }
	else
	{
		gimbal.pid.pit_angle_ref -= gimbal.pid.pit_angle_ref>0?1.0f:-1.0f;
	}

}

void pc_position_ctrl_handler(void)
{
  static float chassis_angle_tmp = 0;
  chassis_angle_tmp = gimbal.pid.yaw_angle_fdb - gimbal.sensor.yaw_relative_angle;
  
  gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
  gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_relative_angle;
	if(pc_recv_mesg.pc_gimbal_flag) //防止还没接收到信息时动
	{
    gimbal.pid.pit_angle_ref = pc_recv_mesg.gimbal_control_data.pit_ref;
		track_pit = pc_recv_mesg.gimbal_control_data.pit_ref;
    gimbal.pid.yaw_angle_ref = -pc_recv_mesg.gimbal_control_data.yaw_ref;
		track_yaw = pc_recv_mesg.gimbal_control_data.yaw_ref;
		pc_recv_mesg.pc_gimbal_flag = 0;
  }
  VAL_LIMIT(gimbal.pid.yaw_angle_ref, chassis_angle_tmp + YAW_ANGLE_MIN, chassis_angle_tmp + YAW_ANGLE_MAX);
  VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
}

static void track_aimor_handler(void)
{
  pc_position_ctrl_handler();
	if(track_once)
	{
		gimbal.pid.yaw_angle_ref += -rc.mouse.x*0.005f * GIMBAL_PC_MOVE_RATIO_YAW;
		gimbal.pid.pit_angle_ref += -rc.mouse.y*0.005f * GIMBAL_PC_MOVE_RATIO_PIT;
	
		VAL_LIMIT(gimbal.pid.pit_angle_ref, track_pit-5, track_pit+5);
		VAL_LIMIT(gimbal.pid.yaw_angle_ref, track_yaw-5, track_yaw+5);
	}
	else
	{
		pc_position_ctrl_handler();
		track_once = 1;
	}
	VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
	VAL_LIMIT(gimbal.pid.yaw_angle_ref, YAW_ANGLE_MIN, YAW_ANGLE_MAX);
	gimbal.yaw_offset_angle = gimbal.sensor.gyro_angle;
}
void shoot_buff_handler(void)
{
	pc_position_ctrl_handler();
}
static void gimbal_patrol_handler(void)
{
  static int16_t patrol_period = PATROL_PERIOD/GIMBAL_PERIOD;
  static int16_t patrol_angle  = PATROL_ANGLE;
  
  gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
  gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_relative_angle;
  
  patrol_count++;
  gimbal.pid.yaw_angle_ref = patrol_angle*sin(2*PI/patrol_period*patrol_count);
  gimbal.pid.pit_angle_ref = patrol_angle*sin(2*PI/patrol_period*patrol_count);
	
	pc_position_ctrl_handler();
	
	VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
	VAL_LIMIT(gimbal.pid.yaw_angle_ref, YAW_ANGLE_MIN, YAW_ANGLE_MAX);
}

/**
  * @brief initialize gimbal pid parameter
  *
  */
void gimbal_param_init(void)
{
  memset(&gimbal, 0, sizeof(gimbal_t));                           //云台所有参数初始化为0
  gimbal.ctrl_mode      = GIMBAL_INIT;                            //云台模式初始化为回正模式，即缓慢回到0,0
  gimbal.last_ctrl_mode = GIMBAL_RELAX;                           //设定原来云台模式为放松态
	gimbal_back_param();                                            //云台斜坡函数初始化，即缓慢回正的实现
	gimbal.ecd_offset_angle = gimbal.sensor.yaw_relative_angle;     //这个一直没用到
  gimbal.input.ac_mode        = NO_ACTION;                        //云台输入为没有信号输入，即等待下一步指令
  gimbal.input.action_angle   = 5.0f;
	gimbal.input.no_action_flag = 2;

	//pid参数初始化
  /* pitch axis motor pid parameter */
  PID_struct_init(&pid_pit, POSITION_PID, 2000, 350.0,
                  16.4f, 0.01f, 0.0f); //30  2000 0  300  
  PID_struct_init(&pid_pit_spd, POSITION_PID, 7000, 3000,
                  18.4f, 0.01f,0.0f); //33 1.8 0

  /* yaw axis motor pid parameter */
  PID_struct_init(&pid_yaw, POSITION_PID, 2000, 200,
                  20.0f, 0.01f, 0.00f); //18.4f, 0.001f,100.0f30.0f, 0.001f,150.0f
//  PID_struct_init(&pid_yaw_spd, POSITION_PID, 7000, 3000,
//                  16.6f, 0.001f, 0.0f);
	PID_struct_init(&pid_yaw_spd, POSITION_PID, 7000, 3000,
                  50.0f, 0.00f, 0.0f);
  
  /* bullet trigger motor pid parameter */
  PID_struct_init(&pid_trigger, POSITION_PID, 10000, 2000,
                  15, 0, 10);//15 0 10 //大小会影响卡蛋函数
  PID_struct_init(&pid_trigger_spd, POSITION_PID, 7000, 3000,
                  7.0, 0.1, 5);//1.5 0.1 

  PID_struct_init(&pid_leftfric_spd,POSITION_PID,7000,3000,
	                7.0, 0.1, 0);//左摩擦轮速度环
	
  PID_struct_init(&pid_rightfric_spd,POSITION_PID,7000,3000,
	                7.0, 0.1, 0);

}

void gimbal_back_param(void)
{ 
  ramp_init(&pit_ramp, BACK_CENTER_TIME/GIMBAL_PERIOD);
  ramp_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_PERIOD);
}





