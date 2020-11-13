#include "TJU_oled_task.h"
#include "cmsis_os.h"
#include "bsp_oled.h"
#include "remote_info.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "pid.h"
#include "shoot_task.h"
#include "gimbal_task.h"
char a[10]="1234";
float b;
void TJU_oled_task(void const * argument)
{
	uint32_t oled_wake_time = osKernelSysTick();
	for(;;)
	{

//   LCD_P6x8Str(0,0,"yaw.out");
//   LCD_P6x8Str(0,1,"yaw.get");
//   LCD_P6x8Str(0,2,"yaw.set");
//	 LCD_P6x8Str(0,3,"yaw_spd.out");
//	 LCD_P6x8Str(0,4,"yaw_spd.get");
//	 LCD_P6x8Str(0,5,"yaw_spd.set");
//			pageshow();
//		if(PAGE_UP)
//		{LCD_P6x8Str(0,5,"yaw_spd.set");}
//		
//	 LCD_P6x8Str(1,3,"pid_pit");
//	 LCD_P6x8Str(1,4,"pid_pit_spd");
//		b=1.34;

//		oledprintf(1,3,b);
//	 oledprintf_int(1,1,shoot.c_shoot_cmd);
//		LCD_P6x8Str(1,2,"moto_trigger.speed_rpm");
//		oledprintf_int(1,3,moto_trigger.speed_rpm);
//		LCD_P6x8Str(1,4,"trig.spd_ref");
//		oledprintf_int(1,5,trig.spd_ref);
//		LCD_P6x8Str(1,6,"pid_trigger_spd.out");
//		oledprintf_int(1,7,pid_trigger_spd.out);
 
///**********gimbal:moto encoder****************/
//		LCD_P6x8Str(1,0,"moto_yaw.encoder");
//		oledprintf_int(1,1,moto_yaw.encoder);
//		LCD_P6x8Str(1,2,"moto_pitch.encoder");
//		oledprintf_int(1,3,moto_pitch.encoder);
///**********gimbal:relative angle*************/
//		LCD_P6x8Str(1,4,"yaw relative angle");
//		oledprintf(1,5,gimbal.sensor.yaw_relative_angle);
//		LCD_P6x8Str(1,6,"pitch relative angle");
//		oledprintf(1,7,gimbal.sensor.pit_relative_angle);
/********************************************/
//		LCD_P6x8Str(1,0,"pid_yaw_spd.out");
//		oledprintf_int(1,1,pid_yaw_spd.out);
//		LCD_P6x8Str(1,2,"pid_pit_spd.out");
//		oledprintf_int(1,3,pid_pit_spd.out);
//		LCD_P6x8Str(1,4,"pid_trigger_spd.out");
//		oledprintf_int(1,5,pid_trigger_spd.out);		
//		
//		LCD_P6x8Str(12,4,(int)rc.ch4);
//		b=pid_pit.set;
LCD_P6x8StrW(1,6,"**Hello");
LCD_P6x8Str(1,7,"**Hello");
		
	 osDelayUntil(&oled_wake_time, 10);

	}
}


