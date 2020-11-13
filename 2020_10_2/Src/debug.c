#include "debug.h"
#include "bsp_uart.h"
#include "sys_config.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "sys_config.h"
#include "pid.h"
#include "judgement_info.h"
#include "pc_info.h"
#include "shoot_task.h"
#include "bsp_can.h"
#include "TJU_imu_task.h"

#define DEBUG_TIME  20  //参数打印周期   单位 ms
#define DEBUG_chassis_TIME  1000  //参数打印周期   单位 ms
#define DEBUG_gimbal_TIME   700  //参数打印周期   单位 ms
UBaseType_t mode_stack_surplus1;

pc_number_t pc_test_yaw;

extern TaskHandle_t info_get_task_t;
extern osTimerId chassis_timer_id;
extern osTimerId gimbal_timer_id;
extern TaskHandle_t shoot_task_t;

void debug_task(void const *argu)
{
//osTimerStart(chassis_timer_id, DEBUG_chassis_TIME);
//osTimerStart(gimbal_timer_id, DEBUG_gimbal_TIME);  
//pc_test_yaw.max = 80;
//pc_test_yaw.min = -80;
//int i=0;
uint32_t debug_wake_time = osKernelSysTick(); 
while (1)
  {
   taskENTER_CRITICAL();				
   //pc_value2param(&pc_test_yaw);
//		if(pc_recv_mesg.pc_gimbal_flag)
//		{
//			
//			if(pc_recv_mesg.pc_shoot_flag)
//				pc_test_yaw.value += 1;
//			else
//				pc_test_yaw.value -= 1;
//			pc_recv_mesg.pc_gimbal_flag = 0;
//		}
//		pc_value2param(&pc_test_yaw);
//		}
//		if(judge_rece_mesg.power_heat_data.chassis_power>70)
//		printf("%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d\r\n",judge_rece_mesg.power_heat_data.chassis_current,chassis.current_sum,chassis.current_sum_ref,
//		                               chassis.current_ref[0],chassis.current_ref[1],chassis.current_ref[2],chassis.current_ref[3],
//		                         chassis.current[0],chassis.current[1],chassis.current[2],chassis.current[3] );
//		printf("%d,%d\r\n",moto_yaw.encoder,moto_pitch.encoder);
//		printf("%f,%f,%f,%f,%d\r\n",gimbal.sensor.pit_relative_angle, gimbal.pid.pit_angle_ref,
//		                            gimbal.pid.pit_spd_fdb,gimbal.pid.pit_spd_ref,global_current.gimbal_current[1]);
		//printf("%f,%f,%f,%f,%d,%d\r\n",pc_recv_mesg.gimbal_control_data.yaw_ref, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_ref,pc_test_yaw.value,pc_test_yaw.param_a,pc_test_yaw.param_b);
		printf("%f,%f,%f\r\n",imu.pit,imu.rol,atti.yaw);
		//printf("%d,%d\r\n",moto_pitch.encoder,moto_yaw.encoder);
//		printf("%f,%f\r\n",gimbal.sensor.pit_relative_angle, gimbal.sensor.yaw_relative_angle);
		//printf("%d,%d,%d,%d\r\n",shoot.fric_wheel_run,shoot.fric_wheel_spd,shoot.firemode,shoot.ctrl_mode);
		//printf("%f,%f,%f\r\n",chassis.current_sum,judge_rece_mesg.power_heat_data.chassis_current,judge_rece_mesg.power_heat_data.chassis_power);
		//printf("%f,%f,%f,%f\r\n",chassis.current_fbd[0],chassis.current_fbd[1],chassis.current_fbd[2],chassis.current_fbd[3]);
//		printf("%f,%f,%f,%f,%f,%f\r\n",pid_spd[0].get,pid_spd[0].set,pid_spd[0].out,
//		                               pid_current[0].get,pid_current[0].set,pid_cu//			printf("%f,%f,%f\r\n",imu_value.imu_angVel.mx,imu_value.imu_angVel.my,imu_value.imu_angVel.mz);
//      printf("%f,%f,%f\r\n",mpu_data.gx / 16.384f,mpu_data.gy / 16.384f,mpu_data.gz / 16.384f);
//    printf("%f,%f,%f\r\n",pid_pit.set,pid_pit.get,pid_pit.out);
//		    printf("%f,%f,%f\r\n",pid_pit_spd.set,pid_pit_spd.get,pid_pit_spd.out);
//		    printf("%f,%f,%f\r\n",pid_yaw_spd.set,pid_yaw_spd.get,pid_yaw_spd.out);
//	printf("%f,%f,%f\r\n",imu_value.angle.mx,imu_value.angle.my,imu_value.angle.mz);
//printf("%d\r\n",gimbal.ctrl_mode);
    taskEXIT_CRITICAL();				
//    
//   // osSignalSet(info_get_task_t, INFO_GET_EXE_SIGNAL);  
//    

    mode_stack_surplus1 = uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(&debug_wake_time, DEBUG_TIME); 
  }


}


