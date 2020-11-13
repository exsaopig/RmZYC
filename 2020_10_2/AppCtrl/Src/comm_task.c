#include "comm_task.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "sys_config.h"
#include "judgement_info.h"

UBaseType_t can_send_surplus;

void can_msg_send_task(void const *argu)
{
  osEvent event;
  
  while (1)
  {
    event = osSignalWait(GIMBAL_MOTOR_MSG_SEND | \
                         CHASSIS_MOTOR_MSG_SEND, osWaitForever);
    
    if (event.status == osEventSignal)
    {
      if (event.value.signals & GIMBAL_MOTOR_MSG_SEND)
      {        
       send_gimbal_motor_ctrl_message(global_current.gimbal_current,global_current.fric_current); //给云台 拨弹 摩擦轮电机发送指令 
      }
      
      if (event.value.signals & CHASSIS_MOTOR_MSG_SEND)
      {
       send_chassis_motor_ctrl_message(global_current.chassis_current); //给底盘电机发指令
			 send_cap_instructions(1,judge_rece_mesg.robot_state.max_chassis_power,judge_rece_mesg.chassis_power_info.chassis_power);
      }
    }
    can_send_surplus = uxTaskGetStackHighWaterMark(NULL);
  }
}

