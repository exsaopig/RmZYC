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
	//ѡ���Ƿ����������
  if (rc.sw1 == RC_MI && rc.sw2 == RC_UP)  //�������ϣ������������
    km.kb_enable = 1;
  else
    km.kb_enable = 0;
}
void get_main_ctrl_mode(void)
{
    switch (rc.sw2) //�ұ�ң��������λ��
    {
      case RC_UP:   //�Ҳ�ң�����������ϣ��ҷǸ�����׼�ʹ����������ֶ���������Զ�
      {
				if(km.buff_ctrl||km.track_ctrl)  global_ctrl_mode = SEMI_AUTO_MODE;
        else                             global_ctrl_mode = MANUAL_CTRL_MODE;
      }break;
      
#ifdef AUTO_NAVIGATION
      case RC_MI:   //�Ҳ����м䣬���Զ�
      {
        global_ctrl_mode = SEMI_AUTO_MODE;
      }break;
      
      case RC_DN:   //�Ҳ����£�ȫ�Զ��������ڱ���
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
    global_ctrl_mode = SAFETY_MODE;          //���߶����·����߰�ȫģʽ
  
  kb_enable_hook();
  
}
static action_mode_e remote_is_action(void)
{
	// ң����ң�˻�����ƶ���С��Χ�ڱ���Ϊ��û������ָ������û�����
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
    case MANUAL_CTRL_MODE:  //���ֶ�
    {
      if (last_global_ctrl_mode == SEMI_AUTO_MODE) //�ɰ��Զ��л��ֶ�����̨���������̻���
        gimbal.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
      if(chassis.last_ctrl_mode == DODGE_MODE && chassis.ctrl_mode != DODGE_MODE)
			{
				gimbal.yaw_offset_angle = gimbal.sensor.gyro_angle;  //��Ťƨ���˳�����̨��������Ϊ��ǰΪ0�ȣ��������ǽǶȣ�
			}
      /* no input control signal gimbal mode handle */
      if (gimbal.input.ac_mode == NO_ACTION)  //�����⵽û������ָ���ң�����������û�б��ж�Ϊ�ƶ�
      {
        if (gimbal.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
        {
          //if (fabs(chassis.vw) <= gim.input.action_angle)
					// yaw��������Ƕ�С��action_angle(��ʼ��Ϊ5�ȣ�ʱ���ж�Ϊ���ָ�ģʽ����Ϊû��ָ������
          if (fabs(gimbal.sensor.yaw_relative_angle) <= gimbal.input.action_angle) 
          {
            gimbal.ctrl_mode = GIMBAL_NO_ARTI_INPUT;     
            gimbal.input.no_action_flag = 1;             //����ϴ�ָ��
            gimbal.input.no_action_time = HAL_GetTick(); //����û������ָ��ʱ��
          }
        }
      }
      else  //IS_ACTION mode
      {
        chassis.follow_gimbal = 1;
        if (gimbal.ctrl_mode == GIMBAL_NO_ARTI_INPUT)
        {
          gimbal.ctrl_mode = GIMBAL_FOLLOW_ZGYRO; //��̨�������ǽǶȿ���
          gimbal.input.no_action_flag = 0;   //����ָ������
          
          gimbal.pid.yaw_angle_ref = 0;  //��ʼ��yaw��ָ���Ƕ�Ϊ0
					gimbal.yaw_offset_angle = gimbal.sensor.gyro_angle; //�趨��ǰ��̨����Ϊ0��
        }
      }
			//���ֶ���ͨ��������л������Զ�
      if(km.buff_ctrl||km.track_ctrl)  //������׼�ʹ������漰��pc�Ͳ�����Э�����ƣ����ڰ��Զ�
			{
				global_ctrl_mode = SEMI_AUTO_MODE;
			}
			if(km.twist_ctrl) //Ťƨ��ģʽ����̨ģʽʼ��Ϊң�����������ư������ǽǶȿ���
			{
				gimbal.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
			}
      if (gimbal.last_ctrl_mode == GIMBAL_RELAX) //�����̨�ϴ�Ϊ����̬�����ȳ�ʼ����������
        gimbal.ctrl_mode = GIMBAL_INIT;
    }break;
    
    case SEMI_AUTO_MODE: 
    {
			if(!km.kb_enable){ //������󲻿��ã��ж�ң�����󰴼�λ�ã��������ԣ�ע����ʱң��������
        switch (rc.sw1)
			  {
        case RC_UP:
			  gimbal.ctrl_mode = GIMBAL_FOLLOW_ZGYRO; //���ϣ�ң����������̨����ʱӦ������Ťƨ��
				break;
        
        case RC_MI:
        gimbal.ctrl_mode = GIMBAL_TRACK_ARMOR; //���У����Ը�����׼
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
        gimbal.ctrl_mode = GIMBAL_SHOOT_BUFF;  //���£����Դ�����
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
			else{           //����������
				if(km.buff_ctrl)        
				{
					gimbal.ctrl_mode = GIMBAL_SHOOT_BUFF;  //����F����������
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
					gimbal.ctrl_mode = GIMBAL_TRACK_ARMOR; //����Ҽ�������������׼
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
				// ��� ��������Ұ��Զ������·Ǹ������������˳����Զ�������ȫ�ֶ�����̨����ģʽΪ�ȴ�ָ�����ʱ���õ��̻�����
				else                   {global_ctrl_mode = MANUAL_CTRL_MODE; gimbal.ctrl_mode = GIMBAL_NO_ARTI_INPUT;}
			}
    }break;
    
    case AUTO_CTRL_MODE: //ң��������ȫ�Զ�
    {
      switch (rc.sw1)
      {
        case RC_UP:
        case RC_MI:
        {
					gimbal.ctrl_mode = GIMBAL_POSITION_MODE; //pc������̨λ��
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
  gimbal.input.ac_mode = remote_is_action(); //�ж�ң�����������û�ж�����û����һ��ָ��
  
  if (gimbal.ctrl_mode != GIMBAL_INIT)
  {
    gimbal_mode_handler(); 
  }

  if (gimbal.ctrl_mode != GIMBAL_PATROL_MODE) //�ж��Ƿ�ΪѲ��ģʽ
    patrol_count = 0;                         //patrol_count��Ѳ�߽Ƕȷֳ�Ѳ�����ڷݣ�����ת�٣����0������һ�ο���Ѳ��        
  
  /* gimbal back to center */
  if (gimbal.last_ctrl_mode == GIMBAL_RELAX && gimbal.ctrl_mode != GIMBAL_RELAX) //����̬��û��
  {
    /* set gimbal init mode */
    gimbal.ctrl_mode = GIMBAL_INIT;     //��ʼ��ʱ����̨����б�º�������̧��
    gimbal.ecd_offset_angle = gimbal.sensor.yaw_relative_angle;  //�����λ�ÿ�ʼ��ԭ��
    gimbal_back_param();               //б�º���
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
      chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;   //�ֶ�ģʽ�¸�����̨
			if(last_global_ctrl_mode == SEMI_AUTO_MODE) //�Ӱ��Զ��л�����������Ҫ����
			{
				chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
				chassis.follow_gimbal = 1;
			}
			if(chassis.ctrl_mode != DODGE_MODE&&chassis.last_ctrl_mode == DODGE_MODE) //��Ťƨ���л�����������Ҫ����
			{
				ramp_init(&twist_ramp,1000/10); //1000ms ��������10ms
				twistflag = 1;   //ramp����
				chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
				chassis.follow_gimbal = 1;
			}
      /* keyboard trigger chassis twist mode */
      if (km.twist_ctrl)   // Ťƨ��Ϊ V��
        chassis.ctrl_mode = DODGE_MODE;
    }break;
    
    case SEMI_AUTO_MODE:
    {
			if(!km.kb_enable){  // ���󲻿��ã�ң�������Ƶ����Զ�Ϊ����ģʽ������
        switch (rc.sw1)
			  {
          case RC_UP:
          chassis.ctrl_mode = DODGE_MODE; //���ϵ���Ťƨ��
          break;
        
          case RC_MI:
				  chassis.ctrl_mode = MANUAL_SEPARATE_GIMBAL; //���е��Ը��飬��������̨����
          break;
        
          case RC_DN:
          chassis.ctrl_mode = MANUAL_SEPARATE_GIMBAL; //���µ��Դ��������������̨����
          break;
        }
			}
			else                // ������ã�ʵս�д������Զ�
			{
				if(km.buff_ctrl||km.track_ctrl) chassis.ctrl_mode = MANUAL_SEPARATE_GIMBAL; //������������������̨����
				else{chassis.follow_gimbal = 1; chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;}  //�����˳����Զ������̸�����̨�һ�֤                                          
			}
    }break;
    
    case AUTO_CTRL_MODE: //ȫ�Զ����ڱ���ʱ�����ƶ���
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

  if (chassis.ctrl_mode != DODGE_MODE) //��Ťƨ��ģʽ����������
    twist_count = 0;                   //����������ڽ�Ťƨ�ɽǶȻ��ֳ����ɷ��Կ������ڣ������0�����´β���
  
  if (gimbal.ctrl_mode == GIMBAL_INIT) //��̨�����ڼ䱣�ֵ��̲���
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
    case MANUAL_CTRL_MODE:  //���ֶ�ģʽ��
    {
			if(last_global_ctrl_mode == SEMI_AUTO_MODE)
			{
				shoot.firemode = SINGLE_SHOOT;
				shoot.fric_wheel_run = 1;
			}
      if (km.kb_enable)     //������ã�������Ʒ������
        shoot.ctrl_mode = KEYBOARD_CTRL_SHOT;
      else                  //����Ϊң�������Ʒ������
        shoot.ctrl_mode = REMOTE_CTRL_SHOT;
    }break;
    
    case SEMI_AUTO_MODE:    //���Զ�ģʽ��
    {		
			if (km.kb_enable)     //������ƿ���
			{
				if(km.buff_ctrl)                  {shoot.firemode=AUTO_CTRL_SHOOT;shoot.ctrl_mode = AUTO_CTRL_SHOT;} //��������Ҫ�Զ����
        else                              shoot.ctrl_mode = KEYBOARD_CTRL_SHOT; //�����ֶ����
			}
      else                  //���󲻿��ã�ң�������н������
			{
				if(rc.sw1 == RC_DN)               {shoot.firemode=AUTO_CTRL_SHOOT;shoot.ctrl_mode = AUTO_CTRL_SHOT;} //���½�����������
        else                              shoot.ctrl_mode = REMOTE_CTRL_SHOT;
			}
    }break;
    
    case AUTO_CTRL_MODE:    //ȫ�Զ�ģʽ���ڱ���
    {
			shoot.firemode=AUTO_CTRL_SHOOT;   //
      shoot.ctrl_mode = AUTO_CTRL_SHOT; //�����������ģʽΪ�Զ�
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
 
 osTimerStart(gimbal_timer_id, GIMBAL_PERIOD);   //������̨��ʱ������̨�������������У�����5ms
 osTimerStart(chassis_timer_id, CHASSIS_PERIOD); //�������̶�ʱ���������������������У�����10ms
 
 uint32_t mode_wake_time = osKernelSysTick();    //��ȡ��ǰϵͳʱ�䣬������ʱ��
 while (1)
 {
			taskENTER_CRITICAL();				//�����ٽ��� 
			get_main_ctrl_mode();  //����ȫ�ֿ���ģʽ�����ֶ������Զ���ȫ�Զ�
			get_gimbal_mode();     //������̨����ģʽ
			get_chassis_mode();    //���µ��̿���ģʽ
			get_shoot_mode();      //���·����������ģʽ
			
			get_global_last_mode();//�����ϴ�ģʽ
//	    printf("mode_switch_task run \r\n");
	    taskEXIT_CRITICAL();				//�˳��ٽ���
	 
		  osSignalSet(info_get_task_t, INFO_GET_EXE_SIGNAL);   //����һ���ź���
	    mode_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
      
	    osDelayUntil(&mode_wake_time, INFO_GET_PERIOD);      // ��ʱһ��ʱ�����ִ��
	    //osDelayUntil(&mode_wake_time, 2000);                   // ������
 }
}


