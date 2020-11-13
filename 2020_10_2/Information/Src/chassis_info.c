#include "chassis_info.h"
#include "chassis_task.h"
#include "keyboard_info.h"
#include "remote_info.h"
#include "keyboard_info.h"
#include "bsp_can.h"

void get_chassis_info(void)
{
  /* get chassis wheel speed */
  for (uint8_t i = 0; i < 4; i++)
  {
    chassis.wheel_spd_fdb[i] = moto_chassis[i].speed_rpm;
  }

  /* get remote and keyboard chassis control information */
  keyboard_chassis_hook();    //键盘控制移动（km结构体更新）
  remote_ctrl_chassis_hook(); //遥控器控制移动信息（rm结构体更新）
  
}


