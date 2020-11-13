#include "modeswitch_task.h"
#include "remote_info.h"
#include "keyboard_info.h"
#include "errordetect_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "shoot_task.h"
#include "sys_config.h"
#include "math.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "sys_config.h"
#include "pc_info.h"
#include "ramp.h"

UBaseType_t mode_stack_surplus;
infantry_mode_e last_global_ctrl_mode;
infantry_mode_e global_ctrl_mode;

extern TaskHandle_t info_get_task_t;
extern osTimerId chassis_timer_id;
extern osTimerId gimbal_timer_id;
//extern int    
//float test1=0.0;
//float test2=0.0;


static void kb_enable_hook(void)
{
	//选择是否开启键鼠控制
  if (rc.sw1 == RC_MI && rc.sw2 == RC_UP)  //左中右上，开启键鼠控制
    km.kb_enable = 1;
  else
    km.kb_enable = 0;
}
void get_main_ctrl_mode(void)
{
    switch (rc.sw2) //右边遥控器按键位置
    {
      case RC_UP:   //右侧遥控器按键在上，且非辅助瞄准和打大神符，则纯手动，否则半自动
      {
				if(km.buff_ctrl||km.track_ctrl)  global_ctrl_mode = SEMI_AUTO_MODE;
        else                             global_ctrl_mode = MANUAL_CTRL_MODE;
      }break;
      
#ifdef AUTO_NAVIGATION
      case RC_MI:   //右侧在中间，半自动
      {
        global_ctrl_mode = SEMI_AUTO_MODE;
      }break;
      
      case RC_DN:   //右侧在下，全自动（用于哨兵）
      {
        global_ctrl_mode = AUTO_CTRL_MODE;
      }break;
#endif
      
      default:
      {
        global_ctrl_mode = SAFETY_MODE;
      }break;
    }
		
  if ((rc.sw1 == RC_DN) && (rc.sw2 == RC_DN))
    global_ctrl_mode = SAFETY_MODE;          //两边都在下防暴走安全模式
  
  kb_enable_hook();
  
}
static action_mode_e remote_is_action(void)
{
	// 遥控器遥杆或鼠标移动在小范围内被认为是没有输入指令，改善用户体验
  if ((abs(rc.ch1) >= 10)
   || (abs(rc.ch2) >= 10)
   || (abs(rc.ch3) >= 10)
   || (abs(rc.ch4) >= 10)
   || (abs(rc.mouse.x) >= 5)
   || (abs(rc.mouse.y) >= 5))
  {
    return IS_ACTION;
  }
  else
  {
    return NO_ACTION;
  }
}
static void gimbal_mode_handler(void)
{
  switch (global_ctrl_mode)
  {
    case MANUAL_CTRL_MODE:  //纯手动
    {
      if (last_global_ctrl_mode == SEMI_AUTO_MODE) //由半自动切回手动，云台不动，底盘回正
        gimbal.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
      if(chassis.last_ctrl_mode == DODGE_MODE && chassis.ctrl_mode != DODGE_MODE)
			{
				gimbal.yaw_offset_angle = gimbal.sensor.gyro_angle;  //从扭屁股退出，云台朝向设置为当前为0度（即陀螺仪角度）
			}
      /* no input control signal gimbal mode handle */
      if (gimbal.input.ac_mode == NO_ACTION)  //如果检测到没有输入指令，即遥控器和鼠标在没有被判定为移动
      {
        if (gimbal.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
        {
          //if (fabs(chassis.vw) <= gim.input.action_angle)
					// yaw轴编码器角度小于action_angle(初始化为5度）时，判定为完成指令，模式设置为没有指令输入
          if (fabs(gimbal.sensor.yaw_relative_angle) <= gimbal.input.action_angle) 
          {
            gimbal.ctrl_mode = GIMBAL_NO_ARTI_INPUT;     
            gimbal.input.no_action_flag = 1;             //完成上次指令
            gimbal.input.no_action_time = HAL_GetTick(); //更新没有输入指令时间
          }
        }
      }
      else  //IS_ACTION mode
      {
        chassis.follow_gimbal = 1;
        if (gimbal.ctrl_mode == GIMBAL_NO_ARTI_INPUT)
        {
          gimbal.ctrl_mode = GIMBAL_FOLLOW_ZGYRO; //云台按陀螺仪角度控制
          gimbal.input.no_action_flag = 0;   //有新指令输入
          
          gimbal.pid.yaw_angle_ref = 0;  //初始化yaw轴指定角度为0
					gimbal.yaw_offset_angle = gimbal.sensor.gyro_angle; //设定当前云台朝向为0度
        }
      }
			//纯手动下通过键鼠可切换到半自动
      if(km.buff_ctrl||km.track_ctrl)  //辅助瞄准和打大神符涉及到pc和操作手协调控制，属于半自动
			{
				global_ctrl_mode = SEMI_AUTO_MODE;
			}
			if(km.twist_ctrl) //扭屁股模式，云台模式始终为遥控器或键鼠控制按陀螺仪角度控制
			{
				gimbal.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
			}
      if (gimbal.last_ctrl_mode == GIMBAL_RELAX) //如果云台上次为放松态，则先初始化缓慢回正
        gimbal.ctrl_mode = GIMBAL_INIT;
    }break;
    
    case SEMI_AUTO_MODE: 
    {
			if(!km.kb_enable){ //如果键鼠不可用，判断遥控器左按键位置，用来调试，注，此时遥控器右中
        switch (rc.sw1)
			  {
        case RC_UP:
			  gimbal.ctrl_mode = GIMBAL_FOLLOW_ZGYRO; //左上，遥控器控制云台，这时应当处于扭屁股
				break;
        
        case RC_MI:
        gimbal.ctrl_mode = GIMBAL_TRACK_ARMOR; //左中，调试辅助瞄准
				if(gimbal.last_ctrl_mode != GIMBAL_TRACK_ARMOR)
				{
					track_pit = gimbal.sensor.pit_relative_angle;
					track_yaw = gimbal.sensor.yaw_relative_angle;
					gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_relative_angle;
					gimbal.pid.pit_angle_ref = gimbal.sensor.pit_relative_angle;
					pc_recv_mesg.gimbal_control_data.yaw_ref = gimbal.sensor.yaw_relative_angle;
					pc_recv_mesg.gimbal_control_data.pit_ref = gimbal.sensor.pit_relative_angle;
					pc_recv_mesg.shoot_contrl_data.shoot_mode = 0;
					pc_recv_mesg.shoot_contrl_data.shoot_speed = 0.0f;
					gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_relative_angle;
					gimbal.pid.pit_angle_ref = gimbal.sensor.pit_relative_angle;
				}
			  break;
        
        case RC_DN:
        gimbal.ctrl_mode = GIMBAL_SHOOT_BUFF;  //左下，调试打大神符
				if(gimbal.last_ctrl_mode != GIMBAL_SHOOT_BUFF)
				{
					gimbal.pit_buff_offset = gimbal.sensor.pit_relative_angle;
					gimbal.yaw_buff_offset = gimbal.sensor.yaw_relative_angle;
					pc_recv_mesg.gimbal_control_data.yaw_ref = gimbal.sensor.yaw_relative_angle;
					pc_recv_mesg.gimbal_control_data.pit_ref = gimbal.sensor.pit_relative_angle;
					pc_recv_mesg.shoot_contrl_data.shoot_mode = 0;
					pc_recv_mesg.shoot_contrl_data.shoot_speed = 0.0f;
					gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_relative_angle;
					gimbal.pid.pit_angle_ref = gimbal.sensor.pit_relative_angle;
					
				}
				break;
				default:
				break;
        }
		  }
			else{           //如果键鼠可用
				if(km.buff_ctrl)        
				{
					gimbal.ctrl_mode = GIMBAL_SHOOT_BUFF;  //键盘F键，打大神符
					if(gimbal.last_ctrl_mode != GIMBAL_SHOOT_BUFF)
					{
						gimbal.pit_buff_offset = gimbal.sensor.pit_relative_angle;
						gimbal.yaw_buff_offset = gimbal.sensor.yaw_relative_angle;
						pc_recv_mesg.gimbal_control_data.yaw_ref = gimbal.sensor.yaw_relative_angle;
						pc_recv_mesg.gimbal_control_data.pit_ref = gimbal.sensor.pit_relative_angle;
						pc_recv_mesg.shoot_contrl_data.shoot_mode = 0;
						pc_recv_mesg.shoot_contrl_data.shoot_speed = 0.0f;
						gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_relative_angle;
						gimbal.pid.pit_angle_ref = gimbal.sensor.pit_relative_angle;
					}
				}
				
				else if(km.track_ctrl)  
				{
					gimbal.ctrl_mode = GIMBAL_TRACK_ARMOR; //鼠标右键长按，辅助瞄准
					if(gimbal.last_ctrl_mode != GIMBAL_TRACK_ARMOR)
					{
						track_pit = gimbal.sensor.pit_relative_angle;
					  track_yaw = gimbal.sensor.yaw_relative_angle;
						gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_relative_angle;
						gimbal.pid.pit_angle_ref = gimbal.sensor.pit_relative_angle;
						pc_recv_mesg.gimbal_control_data.yaw_ref = gimbal.sensor.yaw_relative_angle;
						pc_recv_mesg.gimbal_control_data.pit_ref = gimbal.sensor.pit_relative_angle;
						pc_recv_mesg.shoot_contrl_data.shoot_mode = 0;
						pc_recv_mesg.shoot_contrl_data.shoot_speed = 0.0f;
						gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_relative_angle;
						gimbal.pid.pit_angle_ref = gimbal.sensor.pit_relative_angle;
					}
				}
				// 如果 键鼠可用且半自动条件下非辅瞄与大神符，退出半自动，进入全手动，云台工作模式为等待指令（处理时会让底盘回正）
				else                   {global_ctrl_mode = MANUAL_CTRL_MODE; gimbal.ctrl_mode = GIMBAL_NO_ARTI_INPUT;}
			}
    }break;
    
    case AUTO_CTRL_MODE: //遥控器右下全自动
    {
      switch (rc.sw1)
      {
        case RC_UP:
        case RC_MI:
        {
					gimbal.ctrl_mode = GIMBAL_POSITION_MODE; //pc控制云台位置
        }break;
        
        default:
        {
          gimbal.ctrl_mode = GIMBAL_RELAX;
        }break;
      }
    }break;
    
    default:
    {
      gimbal.ctrl_mode = GIMBAL_RELAX;
    }break;
  }
}
extern uint32_t patrol_count;
void get_gimbal_mode(void)
{
  gimbal.input.ac_mode = remote_is_action(); //判断遥控器或鼠标有没有动，有没有下一步指令
  
  if (gimbal.ctrl_mode != GIMBAL_INIT)
  {
    gimbal_mode_handler(); 
  }

  if (gimbal.ctrl_mode != GIMBAL_PATROL_MODE) //判断是否为巡逻模式
    patrol_count = 0;                         //patrol_count把巡逻角度分成巡逻周期份，控制转速，设成0用于下一次开启巡逻        
  
  /* gimbal back to center */
  if (gimbal.last_ctrl_mode == GIMBAL_RELAX && gimbal.ctrl_mode != GIMBAL_RELAX) //放松态即没劲
  {
    /* set gimbal init mode */
    gimbal.ctrl_mode = GIMBAL_INIT;     //初始化时，云台按照斜坡函数缓慢抬起
    gimbal.ecd_offset_angle = gimbal.sensor.yaw_relative_angle;  //从这个位置开始回原点
    gimbal_back_param();               //斜坡函数
  }
}
static void get_global_last_mode(void)
{
  last_global_ctrl_mode = global_ctrl_mode;
  gimbal.last_ctrl_mode = gimbal.ctrl_mode;
  chassis.last_ctrl_mode = chassis.ctrl_mode;
}


static void chassis_mode_handler(void)
{
  switch (global_ctrl_mode)
  {
    case MANUAL_CTRL_MODE:
    {
      chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;   //手动模式下跟随云台
			if(last_global_ctrl_mode == SEMI_AUTO_MODE) //从半自动切换回来，底盘要回正
			{
				chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
				chassis.follow_gimbal = 1;
			}
			if(chassis.ctrl_mode != DODGE_MODE&&chassis.last_ctrl_mode == DODGE_MODE) //从扭屁股切换回来，底盘要回正
			{
				ramp_init(&twist_ramp,1000/10); //1000ms 底盘任务10ms
				twistflag = 1;   //ramp回正
				chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
				chassis.follow_gimbal = 1;
			}
      /* keyboard trigger chassis twist mode */
      if (km.twist_ctrl)   // 扭屁股为 V键
        chassis.ctrl_mode = DODGE_MODE;
    }break;
    
    case SEMI_AUTO_MODE:
    {
			if(!km.kb_enable){  // 键鼠不可用，遥控器控制到半自动为调试模式，右中
        switch (rc.sw1)
			  {
          case RC_UP:
          chassis.ctrl_mode = DODGE_MODE; //左上调试扭屁股
          break;
        
          case RC_MI:
				  chassis.ctrl_mode = MANUAL_SEPARATE_GIMBAL; //左中调试辅瞄，底盘与云台分离
          break;
        
          case RC_DN:
          chassis.ctrl_mode = MANUAL_SEPARATE_GIMBAL; //左下调试大神符，底盘与云台分离
          break;
        }
			}
			else                // 键鼠可用，实战中触发半自动
			{
				if(km.buff_ctrl||km.track_ctrl) chassis.ctrl_mode = MANUAL_SEPARATE_GIMBAL; //辅瞄或打大神符，底盘云台分离
				else{chassis.follow_gimbal = 1; chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;}  //否则退出半自动，底盘跟随云台且回证                                          
			}
    }break;
    
    case AUTO_CTRL_MODE: //全自动（哨兵暂时不可移动）
    {
      switch (rc.sw1)
      {
        case RC_UP:
        case RC_MI:
        {
          //chassis.ctrl_mode = (chassis_mode_e)pc_recv_mesg.chassis_control_data.ctrl_mode;
        }break;
        
        case RC_DN:
        {
          chassis.ctrl_mode = CHASSIS_STOP;
        }break;
        
      }
    }break;
    
    default:
    {
      chassis.ctrl_mode = CHASSIS_STOP;
    }break;
    
  }
  
}
extern uint32_t twist_count;
void get_chassis_mode(void)
{

  if (chassis.ctrl_mode != DODGE_MODE) //非扭屁股模式，计数清零
    twist_count = 0;                   //这个计数用于将扭屁股角度划分成若干份以控制周期，这里归0便于下次操作
  
  if (gimbal.ctrl_mode == GIMBAL_INIT) //云台回正期间保持底盘不动
  {
    chassis.ctrl_mode = CHASSIS_STOP;
  }
  else
  {
    chassis_mode_handler();
  }
}
void get_shoot_mode(void)
{
  switch (global_ctrl_mode)
  {
    case MANUAL_CTRL_MODE:  //纯手动模式下
    {
			if(last_global_ctrl_mode == SEMI_AUTO_MODE)
			{
				shoot.firemode = SINGLE_SHOOT;
				shoot.fric_wheel_run = 1;
			}
      if (km.kb_enable)     //键鼠可用，键鼠控制发射机构
        shoot.ctrl_mode = KEYBOARD_CTRL_SHOT;
      else                  //否则为遥控器控制发射机构
        shoot.ctrl_mode = REMOTE_CTRL_SHOT;
    }break;
    
    case SEMI_AUTO_MODE:    //半自动模式下
    {		
			if (km.kb_enable)     //键鼠控制可用
			{
				if(km.buff_ctrl)                  {shoot.firemode=AUTO_CTRL_SHOOT;shoot.ctrl_mode = AUTO_CTRL_SHOT;} //打大神符需要自动射击
        else                              shoot.ctrl_mode = KEYBOARD_CTRL_SHOT; //辅瞄手动射击
			}
      else                  //键鼠不可用，遥控器右中进入调试
			{
				if(rc.sw1 == RC_DN)               {shoot.firemode=AUTO_CTRL_SHOOT;shoot.ctrl_mode = AUTO_CTRL_SHOT;} //左下进入大神符调试
        else                              shoot.ctrl_mode = REMOTE_CTRL_SHOT;
			}
    }break;
    
    case AUTO_CTRL_MODE:    //全自动模式（哨兵）
    {
			shoot.firemode=AUTO_CTRL_SHOOT;   //
      shoot.ctrl_mode = AUTO_CTRL_SHOT; //发射机构控制模式为自动
    }break;
    
    default:
    {
      shoot.ctrl_mode = SHOT_DISABLE;
    }break;
    
  }
  if (gimbal.ctrl_mode == GIMBAL_RELAX)
    shoot.ctrl_mode = SHOT_DISABLE;

}

void mode_switch_task(void const *argu)
{
 
 osTimerStart(gimbal_timer_id, GIMBAL_PERIOD);   //开启云台定时器，云台任务按照周期运行，周期5ms
 osTimerStart(chassis_timer_id, CHASSIS_PERIOD); //开启底盘定时器，底盘任务按照周期运行，周期10ms
 
 uint32_t mode_wake_time = osKernelSysTick();    //获取当前系统时间，方便延时用
 while (1)
 {
			taskENTER_CRITICAL();				//进入临界区 
			get_main_ctrl_mode();  //更新全局控制模式，即手动、半自动、全自动
			get_gimbal_mode();     //更新云台控制模式
			get_chassis_mode();    //更新底盘控制模式
			get_shoot_mode();      //更新发射机构控制模式
			
			get_global_last_mode();//更新上次模式
//	    printf("mode_switch_task run \r\n");
	    taskEXIT_CRITICAL();				//退出临界区
	 
		  osSignalSet(info_get_task_t, INFO_GET_EXE_SIGNAL);   //设置一个信号量
	    mode_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
      
	    osDelayUntil(&mode_wake_time, INFO_GET_PERIOD);      // 延时一段时间继续执行
	    //osDelayUntil(&mode_wake_time, 2000);                   // 测试用
 }
}


