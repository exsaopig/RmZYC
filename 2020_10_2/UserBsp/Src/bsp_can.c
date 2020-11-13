#include "bsp_can.h"
#include "chassis_task.h"
#include "sys_config.h"

moto_measure_t moto_pitch;
moto_measure_t moto_yaw;
moto_measure_t moto_trigger;
moto_measure_t moto_chassis[4];
motor_current_t global_current;
moto_measure_t moto_leftfric;
moto_measure_t moto_rightfric;

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
	if(_hcan == &hcan1)
	{
		switch (_hcan->pRxMsg->StdId)
  {
    case CAN_3510_M1_ID:
    case CAN_3510_M2_ID:
    case CAN_3510_M3_ID:
    case CAN_3510_M4_ID:
    {
      static uint8_t i;
      i = _hcan->pRxMsg->StdId - CAN_3510_M1_ID;

      moto_chassis[i].massage_count++ <= 50 ? get_moto_offset(&moto_chassis[i], _hcan) : encoder_data_handler(&moto_chassis[i], _hcan);
    }
    break;
    case CAN_YAW_MOTOR_ID:
    {
      encoder_data_handler(&moto_yaw, _hcan);
    }
    break;
    case CAN_PITCH_MOTOR_ID:
    {
      encoder_data_handler(&moto_pitch, _hcan);
    }
    break;
    case CAN_TRIGGER_MOTOR_ID:
    {
      if (_hcan == &TRIGGER_CAN)
      {
        moto_trigger.massage_count++;
        moto_trigger.massage_count <= 10 ? get_moto_offset(&moto_trigger, _hcan) : encoder_data_handler(&moto_trigger, _hcan);
      }
      else
      {
      }
    }
    break;
#ifdef CHASSIS_GYRO
    case CAN_CHASSIS_ZGYRO_ID:
    {
      chassis.gyro_angle = 0.001f * ((int32_t)(_hcan->pRxMsg->Data[0] << 24) |
                                              (_hcan->pRxMsg->Data[1] << 16) |
                                              (_hcan->pRxMsg->Data[2] << 8) |
                                              (_hcan->pRxMsg->Data[3]));
      
      chassis.gyro_palstance = 0.001f * ((int32_t)(_hcan->pRxMsg->Data[4] << 24) |
                                                  (_hcan->pRxMsg->Data[5] << 16) |
                                                  (_hcan->pRxMsg->Data[6] << 8) |
                                                  (_hcan->pRxMsg->Data[7]));
    }
    break;
#else
#endif
    default:
    {
    }
    break;
  }
  }
	else if(_hcan == &hcan2)
	{
		switch (_hcan->pRxMsg->StdId)
		{
			case CAN2_LEFTFRIC_ID:
			{
				encoder_data_handler(&moto_leftfric,_hcan);
				break;
			}
			case CAN2_RIGHTFRIC_ID:
			{
				encoder_data_handler(&moto_rightfric,_hcan);
				break;
			}
		}
	}
  
  __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
  __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
}
/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(moto_measure_t* moto, CAN_HandleTypeDef* hcan)
{
    moto->encoder        = (uint16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
    moto->offset_encoder = moto->encoder;
}
/**
  * @brief     get motor rpm and calculate motor round_count/total_encoder/total_angle
  * @param     ptr: Pointer to a moto_measure_t structure
  * @attention this function should be called after get_moto_offset() function
  */
void encoder_data_handler(moto_measure_t* moto, CAN_HandleTypeDef* hcan)
{
  moto->last_encoder = moto->encoder;
  moto->encoder      = (uint16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
  
  if (moto->encoder - moto->last_encoder > 4096)
  {
    moto->round_count--;
    moto->encoder_raw_rate = moto->encoder - moto->last_encoder - 8192;
  }
  else if (moto->encoder - moto->last_encoder < -4096)
  {
    moto->round_count++;
    moto->encoder_raw_rate = moto->encoder - moto->last_encoder + 8192;
  }
  else
  {
    moto->encoder_raw_rate = moto->encoder - moto->last_encoder;
  }

  moto->total_encoder = moto->round_count * 8192 + moto->encoder - moto->offset_encoder;
  /* total angle, unit is degree */
  moto->total_angle = moto->total_encoder / ENCODER_ANGLE_RATIO;
  
#ifdef CHASSIS_EC60
  int32_t temp_sum = 0;
  moto->rate_buf[moto->buf_count++] = moto->encoder_raw_rate;
  if (moto->buf_count >= FILTER_BUF)
    moto->buf_count = 0;
  for (uint8_t i = 0; i < FILTER_BUF; i++)
  {
    temp_sum += moto->rate_buf[i];
  }
  moto->filter_rate = (int32_t)(temp_sum/FILTER_BUF);
  moto->speed_rpm   = (int16_t)(moto->filter_rate * 7.324f);
#else
  moto->speed_rpm_last          = moto->speed_rpm;
  moto->speed_rpm               = (int16_t)(hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);
	moto->given_current_last_last = moto->given_current_last;
	moto->given_current_last      = moto->given_current;
  moto->given_current           = (int16_t)(hcan->pRxMsg->Data[4] << 8 | hcan->pRxMsg->Data[5]);
	moto->given_current           = moto->given_current>0?moto->given_current:(-moto->given_current);
	moto->given_current = (int)(0.1f*(float)(moto->given_current)+0.1f*(float)(moto->given_current_last)+0.8f*(moto->given_current_last_last));
#endif

}
/**
  * @brief   can filter initialization
  * @param   CAN_HandleTypeDef
  * @retval  None
  */
void can_device_init(void)
{
  //can1 &can2 use same filter config
  CAN_FilterConfTypeDef  can_filter;
  static CanTxMsgTypeDef Tx1Message;
  static CanRxMsgTypeDef Rx1Message;
  static CanTxMsgTypeDef Tx2Message;
  static CanRxMsgTypeDef Rx2Message;

  can_filter.FilterNumber         = 0;
  can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh         = 0x0000;
  can_filter.FilterIdLow          = 0x0000;
  can_filter.FilterMaskIdHigh     = 0x0000;
  can_filter.FilterMaskIdLow      = 0x0000;
  can_filter.FilterFIFOAssignment = CAN_FilterFIFO0;
  can_filter.BankNumber           = 14;
  can_filter.FilterActivation     = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  //while (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK);
  
  can_filter.FilterNumber         = 14;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter);
  //while (HAL_CAN_ConfigFilter(&hcan2, &can_filter) != HAL_OK);
    
  hcan1.pTxMsg = &Tx1Message;
  hcan1.pRxMsg = &Rx1Message;
  hcan2.pTxMsg = &Tx2Message;
  hcan2.pRxMsg = &Rx2Message;
}

void can_receive_start(void)
{
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0); 
}

/**
  * @brief     reset single axis gyroscope 
  * @attention gyro reset at least wait 2s  
  */
void gyro_device_init(void)
{
  while (ZGYRO_CAN.State == HAL_CAN_STATE_BUSY_TX);
  ZGYRO_CAN.pTxMsg->StdId   = CAN_ZGYRO_RST_ID;
  ZGYRO_CAN.pTxMsg->IDE     = CAN_ID_STD;
  ZGYRO_CAN.pTxMsg->RTR     = CAN_RTR_DATA;
  ZGYRO_CAN.pTxMsg->DLC     = 0x08;
  ZGYRO_CAN.pTxMsg->Data[0] = 0;
  ZGYRO_CAN.pTxMsg->Data[1] = 1;
  ZGYRO_CAN.pTxMsg->Data[2] = 2;
  ZGYRO_CAN.pTxMsg->Data[3] = 3;
  ZGYRO_CAN.pTxMsg->Data[4] = 4;
  ZGYRO_CAN.pTxMsg->Data[5] = 5;
  ZGYRO_CAN.pTxMsg->Data[6] = 6;
  ZGYRO_CAN.pTxMsg->Data[7] = 7;
  HAL_CAN_Transmit(&ZGYRO_CAN, 10);
}

/**
  * @brief  send current which pid calculate to esc. message to calibrate 6025 gimbal motor esc
  * @param  current value corresponding motor(yaw/pitch/trigger)
  */
void send_gimbal_current(int16_t yaw_iq, int16_t pit_iq, int16_t trigger_iq)
{
  GIMBAL_CAN.pTxMsg->StdId   = 0x1ff;
  GIMBAL_CAN.pTxMsg->IDE     = CAN_ID_STD;
  GIMBAL_CAN.pTxMsg->RTR     = CAN_RTR_DATA;
  GIMBAL_CAN.pTxMsg->DLC     = 8;
  /* adding minus due to clockwise rotation of the gimbal motor with positive current */
  GIMBAL_CAN.pTxMsg->Data[0] = -yaw_iq >> 8;
  GIMBAL_CAN.pTxMsg->Data[1] = -yaw_iq;
  /* adding minus due to clockwise rotation of the gimbal motor with positive current */
  GIMBAL_CAN.pTxMsg->Data[2] = -pit_iq >> 8;
  GIMBAL_CAN.pTxMsg->Data[3] = -pit_iq;
  GIMBAL_CAN.pTxMsg->Data[4] = trigger_iq >> 8;
  GIMBAL_CAN.pTxMsg->Data[5] = trigger_iq;
  GIMBAL_CAN.pTxMsg->Data[6] = 0;
  GIMBAL_CAN.pTxMsg->Data[7] = 0;
  HAL_CAN_Transmit(&GIMBAL_CAN, 10);
}
/**
  * @brief  send calculated current to motor
  * @param  3510 motor ESC id
  */
void send_chassis_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  CHASSIS_CAN.pTxMsg->StdId   = 0x200;
  CHASSIS_CAN.pTxMsg->IDE     = CAN_ID_STD;
  CHASSIS_CAN.pTxMsg->RTR     = CAN_RTR_DATA;
  CHASSIS_CAN.pTxMsg->DLC     = 0x08;
  CHASSIS_CAN.pTxMsg->Data[0] = iq1 >> 8;
  CHASSIS_CAN.pTxMsg->Data[1] = iq1;
  CHASSIS_CAN.pTxMsg->Data[2] = iq2 >> 8;
  CHASSIS_CAN.pTxMsg->Data[3] = iq2;
  CHASSIS_CAN.pTxMsg->Data[4] = iq3 >> 8;
  CHASSIS_CAN.pTxMsg->Data[5] = iq3;
  CHASSIS_CAN.pTxMsg->Data[6] = iq4 >> 8;
  CHASSIS_CAN.pTxMsg->Data[7] = iq4;
  HAL_CAN_Transmit(&CHASSIS_CAN, 10);
}

void send_fric_current(int16_t left, int16_t right)
{
  FRIC_CAN.pTxMsg->StdId   = 0x200;
  FRIC_CAN.pTxMsg->IDE     = CAN_ID_STD;
  FRIC_CAN.pTxMsg->RTR     = CAN_RTR_DATA;
  FRIC_CAN.pTxMsg->DLC     = 0x08;
  FRIC_CAN.pTxMsg->Data[0] = left  >> 8;
  FRIC_CAN.pTxMsg->Data[1] = left;
  FRIC_CAN.pTxMsg->Data[2] = right >> 8;
  FRIC_CAN.pTxMsg->Data[3] = right;
  FRIC_CAN.pTxMsg->Data[4] = 0;
  FRIC_CAN.pTxMsg->Data[5] = 0;
  FRIC_CAN.pTxMsg->Data[6] = 0;
  FRIC_CAN.pTxMsg->Data[7] = 0;
  HAL_CAN_Transmit(&FRIC_CAN, 10);
}

void send_cap_instructions(uint8_t open, uint8_t max, float power)
{
  CAP_CAN.pTxMsg->StdId   = CAN_TOCAP_ID;
  CAP_CAN.pTxMsg->IDE     = CAN_ID_STD;
  CAP_CAN.pTxMsg->RTR     = CAN_RTR_DATA;
  CAP_CAN.pTxMsg->DLC     = 0x08;
  CAP_CAN.pTxMsg->Data[0] = open;
  CAP_CAN.pTxMsg->Data[1] = max;
  CAP_CAN.pTxMsg->Data[2] = 0;
  CAP_CAN.pTxMsg->Data[3] = 0;
  CAP_CAN.pTxMsg->Data[4] = *((uint8_t *)(&power));
  CAP_CAN.pTxMsg->Data[5] = *((uint8_t *)(&power) + 1);
  CAP_CAN.pTxMsg->Data[6] = *((uint8_t *)(&power) + 2);
  CAP_CAN.pTxMsg->Data[7] = *((uint8_t *)(&power) + 3);
  HAL_CAN_Transmit(&CAP_CAN, 10);
}


void send_gimbal_motor_ctrl_message(int16_t gimbal_cur[],int16_t fric_cur[])
{
  /* 0: yaw motor current
     1: pitch motor current
     2: trigger motor current*/
  send_gimbal_current(gimbal_cur[0], gimbal_cur[1], gimbal_cur[2]);
	send_fric_current(fric_cur[0],fric_cur[1]);
}
void send_chassis_motor_ctrl_message(int16_t chassis_cur[])
{
  send_chassis_current(chassis_cur[0], chassis_cur[1], 
                       chassis_cur[2], chassis_cur[3]);
}
