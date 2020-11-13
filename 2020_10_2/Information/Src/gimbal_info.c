
#include "gimbal_info.h"
#include "imu_info.h"
#include "bsp_can.h"
#include "calibrate_info.h"
#include "remote_info.h"
#include "keyboard_info.h"
#include "gimbal_task.h"
#include "sys_config.h"



uint8_t read_gimbal_offset(int32_t *pit_offset, int32_t *yaw_offset)
{
#ifndef CALIBRATE
	*pit_offset = PITCH_OFFSET;
	*yaw_offset = YAW_OFFSET;
	return 1;
#else
  if (cali_param.gim_cali_data[CALI_GIMBAL_CENTER].calied_done == CALIED_FLAG)
  {
    *pit_offset = cali_param.gim_cali_data[CALI_GIMBAL_CENTER].pitch_offset;
    *yaw_offset = cali_param.gim_cali_data[CALI_GIMBAL_CENTER].yaw_offset;
    
    return 1;
  }
  else
    return 0;
#endif
}
/**
  * @brief     get relative position angle to center
  * @param[in] raw_ecd: gimbal motor encoder raw angle
  * @param[in] center_offset: read gim_cali_data from chip flash
  * @retval    relative angle, unit is degree.
  */
int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
  int16_t tmp = 0;
  if (center_offset >= 4096)
  {
    if (raw_ecd > center_offset - 4096)
      tmp = raw_ecd - center_offset;
    else
      tmp = raw_ecd + 8192 - center_offset;
  }
  else
  {
    if (raw_ecd > center_offset + 4096)
      tmp = raw_ecd - 8192 - center_offset;
    else
      tmp = raw_ecd - center_offset;
  }
  return tmp;
}
void get_gimbal_info(void)
{
  /* transform absolute encoder value to relative angle */
  static float yaw_ecd_ratio = YAW_MOTO_POSITIVE_DIR*YAW_DECELE_RATIO/ENCODER_ANGLE_RATIO;
  static float pit_ecd_ratio = PIT_MOTO_POSITIVE_DIR*PIT_DECELE_RATIO/ENCODER_ANGLE_RATIO;
	//���´ӱ�������ȡ�ĽǶ���Ϣ
  gimbal.sensor.yaw_relative_angle = yaw_ecd_ratio*get_relative_pos(moto_yaw.encoder, gimbal.yaw_center_offset);
  gimbal.sensor.pit_relative_angle = pit_ecd_ratio*get_relative_pos(moto_pitch.encoder, gimbal.pit_center_offset);
  
  /* get gimbal relative palstance */
	// ���´������ǻ�ȡ�Ľ��ٶ���Ϣ
	//�ϸ������������Ҹ�
  //the Z axis(yaw) of gimbal coordinate system corresponds to the IMU Z axis
  gimbal.sensor.yaw_palstance = mpu_data.gz / 16.384f; //unit: dps  
  //the Y axis(pitch) of gimbal coordinate system corresponds to the IMU -X axis
  gimbal.sensor.pit_palstance = -PITCH_SPD_DIR * mpu_data.gx / 16.384f; //unit: dps
	
	// ���´������ǻ�ȡ�ĽǶ���Ϣ
  gimbal.sensor.gyro_angle = atti.yaw;
  /* get remote and keyboard gimbal control information */
  remote_ctrl_gimbal_hook();  //ң����������̨��Ϣ��ң�������ң�˿���rm�ṹ����Ϣ���£�
	keyboard_gimbal_hook();     //���������̨��Ϣ����������ƶ������̰���������Ҽ�����km�ṹ����£�
  
  /* get gimbal calibration command */
  if (gimbal.ctrl_mode == GIMBAL_RELAX)
  {
    //gimbal_cali_msg_hook(pc_recv_mesg.cali_cmd_data.type, last_cali_type);
    //last_cali_type = pc_recv_mesg.cali_cmd_data.type;
  }

}
