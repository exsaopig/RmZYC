#include "judgement_info.h"
#include "shoot_task.h"
#include "protocol.h"
#include "string.h"
#include "pc_info.h"
#include "cmsis_os.h"

uint8_t  judge_buf[JUDGE_FIFO_BUFLEN];
tFrameHeader          FrameHeader;
uint16_t CmdId;
uint32_t judge_time_ms;
uint32_t judge_time_last;
receive_judge_t judge_rece_mesg;
extern shoot_t shoot;

void judgementDataHandler(void)
{
	for (int i=0;i<JUDGE_FIFO_BUFLEN;i++)
		{
			if (judge_buf[i]==DN_REG_ID)
				memcpy(&FrameHeader, &judge_buf[i], sizeof(tFrameHeader));
						if ((FrameHeader.sof == DN_REG_ID)
        && (verify_crc8_check_sum(&judge_buf[i], sizeof(tFrameHeader)))
        && (verify_crc16_check_sum(&judge_buf[i], sizeof(tFrameHeader) + CMD_LEN + FrameHeader.dataLenth + CRC_LEN)))	//进行CRC校验
				{
					memcpy(&CmdId, (&judge_buf[i]+ sizeof(tFrameHeader)), sizeof(CmdId));
					switch (CmdId)
        {
				case EVENT_DATA_ID:
						memcpy(&judge_rece_mesg.event_data, (&judge_buf[i] + sizeof(tFrameHeader) + sizeof(CmdId)), FrameHeader.dataLenth);
				break;
				case PROJECTILE_ID:
						memcpy(&judge_rece_mesg.projectile_data, (&judge_buf[i] + sizeof(tFrameHeader) + sizeof(CmdId)), FrameHeader.dataLenth);
						shoot.remain_bullets += judge_rece_mesg.projectile_data.supply_projectile_num;
				break;
				case ROBOT_STATE_ID:
					memcpy(&judge_rece_mesg.robot_state, (&judge_buf[i] + sizeof(tFrameHeader) + sizeof(CmdId)), FrameHeader.dataLenth);
					break;
				case CHASSIS_INFO_ID:
					judge_rece_mesg.chassis_power_info.chassis_current_last = judge_rece_mesg.chassis_power_info.chassis_current;
					memcpy(&judge_rece_mesg.chassis_power_info, (&judge_buf[i]+ sizeof(tFrameHeader) + sizeof(CmdId)), FrameHeader.dataLenth);
					break;
				case SHOOT_INFO_ID:
					memcpy(&judge_rece_mesg.shoot_data, (&judge_buf[i] + sizeof(tFrameHeader) + sizeof(CmdId)), FrameHeader.dataLenth);
					if(shoot.remain_bullets > 0)  //防止计数溢出
						shoot.remain_bullets --;
//					judge_rece_mesg.shoot_data.bullet_speed;
				break;
        }
				memset(&FrameHeader, 0, sizeof(FrameHeader));
				}
		}
		memset(&judge_buf,0,sizeof(judge_buf));
}

//void judgement_task(void const *argu)
//{
//	uint32_t judgement_wake_time = osKernelSysTick(); 
//	while (1)
//	{
//		judge_time_ms = HAL_GetTick() - judge_time_last;
//		taskENTER_CRITICAL();
//		for (int i=0;i<JUDGE_FIFO_BUFLEN;i++)
//		{
//			if (judge_buf[i]==DN_REG_ID)
//				memcpy(&FrameHeader, &judge_buf[i], sizeof(tFrameHeader));
//						if ((FrameHeader.sof == DN_REG_ID)
//        && (1 == verify_crc8_check_sum(&judge_buf[i], sizeof(tFrameHeader)))
//        && (1 == verify_crc16_check_sum(&judge_buf[i], sizeof(tFrameHeader) + CMD_LEN + FrameHeader.dataLenth + CRC_LEN)))	//进行CRC校验
//				{
//					memcpy(&CmdId, (&judge_buf[i] + sizeof(tFrameHeader)), sizeof(CmdId));
//					switch (CmdId)
//        {
//				case ROBOT_STATE_ID:
//					memcpy(&judge_rece_mesg.robot_state, (&judge_buf[i] + sizeof(tFrameHeader) + sizeof(CmdId)), FrameHeader.dataLenth);
//					shoot.shootlevel=judge_rece_mesg.robot_state.robot_level;
//					break;
//				case CHASSIS_INFO_ID:
//					judge_rece_mesg.chassis_power_info.chassis_current_last=judge_rece_mesg.chassis_power_info.chassis_current;
//					memcpy(&judge_rece_mesg.chassis_power_info, (&judge_buf[i] + sizeof(tFrameHeader) + sizeof(CmdId)), FrameHeader.dataLenth);
//					break;
//				case SHOOT_INFO_ID:
//					memcpy(&judge_rece_mesg.shoot_data, (&judge_buf[i] + sizeof(tFrameHeader) + sizeof(CmdId)), FrameHeader.dataLenth);
//				break;
//				case BUFF_ID:
//					memcpy(&judge_rece_mesg.buff_data, (&judge_buf[i] + sizeof(tFrameHeader) + sizeof(CmdId)), FrameHeader.dataLenth);
//				break;
//        }
//				memset(&FrameHeader, 0, sizeof(FrameHeader));
//				}
//		}
//		//暂时强行把0202id的memcpy进去，前面那个复制比较慢，暂时还不知道原因
//			memcpy(&FrameHeader, &judge_buf, sizeof(tFrameHeader));
//			if ((FrameHeader.sof == DN_REG_ID)
//        && (1 == verify_crc8_check_sum(judge_buf, sizeof(tFrameHeader)))
//        && (1 == verify_crc16_check_sum(judge_buf, sizeof(tFrameHeader) + CMD_LEN + FrameHeader.dataLenth + CRC_LEN)))	//进行CRC校验
//				memcpy(&CmdId, (judge_buf + sizeof(tFrameHeader)), sizeof(CmdId));
//			if (CmdId==CHASSIS_INFO_ID)
//			{
//				judge_rece_mesg.chassis_power_info.chassis_current_last=judge_rece_mesg.chassis_power_info.chassis_current;
//				memcpy(&judge_rece_mesg.chassis_power_info, (judge_buf + sizeof(tFrameHeader) + sizeof(CmdId)), FrameHeader.dataLenth);
//				memset(&FrameHeader, 0, sizeof(FrameHeader));
//			}
//		taskEXIT_CRITICAL();	
//		osDelayUntil(&judgement_wake_time, 5);
//		judge_time_last = HAL_GetTick();
//	}
//}


