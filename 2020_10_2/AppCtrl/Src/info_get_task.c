#include "info_get_task.h"
#include "sys_config.h"
#include "chassis_info.h"
#include "gimbal_info.h"
#include "keyboard_info.h"
#include "shoot_info.h"
#include "cmsis_os.h"
#include "remote_info.h"

/* stack usage monitor */
UBaseType_t info_stack_surplus;

/* information get task global parameter */


uint32_t info_time_last;
int info_time_ms;
void info_get_task(void const *argu)
{
  osEvent event;
  
  while (1)
  {
    event = osSignalWait(INFO_GET_EXE_SIGNAL, osWaitForever);
    
    if (event.status == osEventSignal)
    {
      if (event.value.signals & INFO_GET_EXE_SIGNAL)
      {
        info_time_ms = HAL_GetTick() - info_time_last;
        info_time_last = HAL_GetTick();
        
        taskENTER_CRITICAL();
        
        keyboard_global_hook();   //鼠标左右键长按单点区分
        
        get_chassis_info();
        get_gimbal_info();
        get_shoot_info();
        
        get_global_last_info();

        taskEXIT_CRITICAL();

      }
    }
    
    info_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
  }
}
	
	







