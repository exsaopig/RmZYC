#include "shoot_info.h"
#include "shoot_task.h"
#include "remote_info.h"
#include "keyboard_info.h"

void get_shoot_info(void)
{
  switch (shoot.ctrl_mode)
  {
    case REMOTE_CTRL_SHOT:
    {
			shoot.firemode = SINGLE_SHOOT;
      remote_ctrl_shoot_hook();
    }break;
    
    case KEYBOARD_CTRL_SHOT:
    {
      //shoot.fric_wheel_spd = speed_debug;//DEFAULT_FRIC_WHEEL_SPEED;
      remote_ctrl_shoot_hook();
      keyboard_shoot_hook();
    }break;
    
    case SEMIAUTO_CTRL_SHOT:
    {
      //shoot.fric_wheel_run = pc_recv_mesg.shoot_control_data.fric_wheel_run;
      //shoot.fric_wheel_spd = pc_recv_mesg.shoot_control_data.fric_wheel_spd;
      //shoot.shoot_cmd      = pc_recv_mesg.shoot_control_data.shoot_cmd;
      //shoot.c_shoot_cmd    = pc_recv_mesg.shoot_control_data.c_shoot_cmd;
    }break;
    
    case AUTO_CTRL_SHOT:
    {
			
      //shoot.fric_wheel_run = pc_recv_mesg.shoot_control_data.fric_wheel_run;
      //shoot.fric_wheel_spd = pc_recv_mesg.shoot_control_data.fric_wheel_spd;
      //shoot.shoot_cmd      = pc_recv_mesg.shoot_control_data.shoot_cmd;
      //shoot.c_shoot_cmd    = pc_recv_mesg.shoot_control_data.c_shoot_cmd;
    }break;
    
    default:
    {
      shoot.fric_wheel_run = 0;
      shoot.shoot_cmd      = 0;
      shoot.c_shoot_cmd    = 0;
    }break;
  }
  /* get remote and keyboard friction wheel control information */
    
  /* get remote and keyboard shoot command information */
}
