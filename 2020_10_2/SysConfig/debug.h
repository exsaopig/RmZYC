#ifndef __DEBUG_H
#define __DEBUG_H

#include "gimbal_task.h"
#include "gimbal_info.h"
#include "imu_info.h"
#include "chassis_info.h"
#include "chassis_task.h"
#include "bsp_uart.h"
#include "pid.h"
#include "math.h"
#include "stdlib.h"


//void debug_yaw(void)
//{
//	printf("can_data: %d, degree: %f\r\n", moto_yaw.encoder, gimbal.sensor.yaw_relative_angle);
//}

//void debug_pid_print(pid_t pid)
//{
//	printf("%f,%f,%f,%f,%f,%f\r\n", pid.set, pid.get, pid.out, pid.p, pid.i, pid.d); 
//}
void debug_task(void const *argu);

#endif


