#include "TJU_imu_task.h"
#include "imu_info.h"
#include "bsp_imu_9250.h"
imu_value_t imu_value;

//UBaseType_t imu_stack_surplus;
unsigned char chrTemp[30];
void ShortToChar(short sData,unsigned char cData[])
{
	cData[0]=sData&0xff;
	cData[1]=sData>>8;
}
short CharToShort(unsigned char cData[])
{
	return ((short)cData[1]<<8)|cData[0];
}


void TJU_imu_task(void const * argument)
{
	uint32_t imu_wake_time_2 = osKernelSysTick();
	
 for(;;)
	{
		taskENTER_CRITICAL();
//		TaskDIn;
		IICreadBytes(0x50, AX, 24,&chrTemp[0]);
		imu_value.imu_acce.mx = (float)CharToShort(&chrTemp[0])/32768*16;
		imu_value.imu_acce.my = (float)CharToShort(&chrTemp[2])/32768*16;
		imu_value.imu_acce.mz = (float)CharToShort(&chrTemp[4])/32768*16;
		imu_value.imu_angVel.mx = (float)CharToShort(&chrTemp[6])/32768*2000;
		imu_value.imu_angVel.my = (float)CharToShort(&chrTemp[8])/32768*2000;
		imu_value.imu_angVel.mz = (float)CharToShort(&chrTemp[10])/32768*2000;
		imu_value.magnetic.mx = CharToShort(&chrTemp[12]);
		imu_value.magnetic.my = CharToShort(&chrTemp[14]);
		imu_value.magnetic.mz = CharToShort(&chrTemp[16]);
		imu_value.angle.mx = (float)CharToShort(&chrTemp[18])/32768*180;
		imu_value.angle.my = (float)CharToShort(&chrTemp[20])/32768*180;
		imu_value.angle.mz = (float)CharToShort(&chrTemp[22])/32768*180;
    osDelayUntil(&imu_wake_time_2, 5);

   	taskEXIT_CRITICAL();
//		 imu_stack_surplus=uxTaskGetStackHighWaterMark(NULL);
	}
}

