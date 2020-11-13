#include "UI_task.h"
#include "bsp_uart.h"
#include "usart.h"
#include "stdio.h"
#include "remote_info.h"
#include "pc_info.h"
#include "pid.h"
#include "judgement_info.h"
#include "sys_config.h"
#include "protocol.h"
#include "cmsis_os.h"

//版本 裁判系统串口协议V1.1

graphic_single_t message_onegraphic;
graphic_five_t message_onebull;
graphic_dele_t message_dele;
float_single_t message_onefloat;

uint8_t UI_bull = 0;
uint8_t seqcount = 0;

extern TaskHandle_t ui_task_t;

void UI_Init(void)
{
	send_bull_init();
}

void ui_task(void const *argu)
{
	uint32_t UI_wake_time = osKernelSysTick();
	while(1)
	{
		switch(UI_bull)
		{
			case 0:
				send_dele(0);
				break;
			case 1:
				send_bull();
				break;
		}
//		send_dele(1);
		renew_float();
		osDelayUntil(&UI_wake_time, UI_TIME);  
	}
}

void send_bull(void)
{
	message_onebull.FrameHead.seq = (seqcount++) & 0xFF;
	
	append_crc8_check_sum(&message_onebull.FrameHead.sof,sizeof(message_onebull.FrameHead));
	append_crc16_check_sum(&message_onebull.FrameHead.sof,sizeof(message_onebull.FrameHead)
	+ CMD_LEN + CRC_LEN + message_onebull.FrameHead.dataLenth);
	
	HAL_UART_Transmit(&JUDGE_HUART,&message_onebull.FrameHead.sof,sizeof(message_onebull),0xffff);
}

void send_bull_init(void)
{
	message_onebull.FrameHead.sof = 0xA5;
	message_onebull.FrameHead.dataLenth = 81;
//	message_onebull.FrameHead.seq = (seqcount++) & 0xFF;                  //帧头填充
	
	message_onebull.CmdId = 0x0301;                                       // 机器人交互命令码
	
	message_onebull.Interactive_header_data.data_cmd_id = 0x0103;
	message_onebull.Interactive_header_data.sender_ID = judge_rece_mesg.robot_state.robot_id;                
	message_onebull.Interactive_header_data.receiver_ID = 0x0100 + judge_rece_mesg.robot_state.robot_id;
	
	message_onebull.Client_graphic_five.grapic_data_struct[0].graphic_name[0] = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[0].graphic_name[1] = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[0].graphic_name[2] = 0;  // 图形名称，不要重复
	message_onebull.Client_graphic_five.grapic_data_struct[0].operate_tpye = 1;
	message_onebull.Client_graphic_five.grapic_data_struct[0].graphic_tpye = 2;
	message_onebull.Client_graphic_five.grapic_data_struct[0].layer = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[0].color = 7;
	message_onebull.Client_graphic_five.grapic_data_struct[0].start_angle = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[0].end_angle = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[0].width = 10;
	message_onebull.Client_graphic_five.grapic_data_struct[0].start_x = 960;
	message_onebull.Client_graphic_five.grapic_data_struct[0].start_y = 440;
	message_onebull.Client_graphic_five.grapic_data_struct[0].radius = 100;
	message_onebull.Client_graphic_five.grapic_data_struct[0].end_x = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[0].end_y = 0;
	
	message_onebull.Client_graphic_five.grapic_data_struct[1].graphic_name[0] = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[1].graphic_name[1] = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[1].graphic_name[2] = 1;  // 图形名称，不要重复
	message_onebull.Client_graphic_five.grapic_data_struct[1].operate_tpye = 1;
	message_onebull.Client_graphic_five.grapic_data_struct[1].graphic_tpye = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[1].layer = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[1].color = 7;
	message_onebull.Client_graphic_five.grapic_data_struct[1].start_angle = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[1].end_angle = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[1].width = 5;
	message_onebull.Client_graphic_five.grapic_data_struct[1].start_x = 810;
	message_onebull.Client_graphic_five.grapic_data_struct[1].start_y = 440;
	message_onebull.Client_graphic_five.grapic_data_struct[1].radius = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[1].end_x = 1110;
	message_onebull.Client_graphic_five.grapic_data_struct[1].end_y = 440;
	
	message_onebull.Client_graphic_five.grapic_data_struct[2].graphic_name[0] = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[2].graphic_name[1] = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[2].graphic_name[2] = 2;  // 图形名称，不要重复
	message_onebull.Client_graphic_five.grapic_data_struct[2].operate_tpye = 1;
	message_onebull.Client_graphic_five.grapic_data_struct[2].graphic_tpye = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[2].layer = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[2].color = 7;
	message_onebull.Client_graphic_five.grapic_data_struct[2].start_angle = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[2].end_angle = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[2].width = 5;
	message_onebull.Client_graphic_five.grapic_data_struct[2].start_x = 960;
	message_onebull.Client_graphic_five.grapic_data_struct[2].start_y = 290;
	message_onebull.Client_graphic_five.grapic_data_struct[2].radius = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[2].end_x = 960;
	message_onebull.Client_graphic_five.grapic_data_struct[2].end_y = 590;
	
	message_onebull.Client_graphic_five.grapic_data_struct[3].operate_tpye = 0;
	message_onebull.Client_graphic_five.grapic_data_struct[4].operate_tpye = 0;
}

void send_circle_single(void)
{
	message_onegraphic.FrameHead.sof = 0xA5;
	message_onegraphic.FrameHead.dataLenth = 21;
	message_onegraphic.FrameHead.seq = (seqcount++) & 0xFF;                  //帧头填充
	
	message_onegraphic.CmdId = 0x0301;                                       // 机器人交互命令码
	
	message_onegraphic.Interactive_header_data.data_cmd_id = 0x0101;
	message_onegraphic.Interactive_header_data.sender_ID = judge_rece_mesg.robot_state.robot_id;                
	message_onegraphic.Interactive_header_data.receiver_ID = 0x0100 + judge_rece_mesg.robot_state.robot_id;     
	
	message_onegraphic.Client_graphic_single.grapic_data_struct.graphic_name[0] = 0;
	message_onegraphic.Client_graphic_single.grapic_data_struct.graphic_name[1] = 0;
	message_onegraphic.Client_graphic_single.grapic_data_struct.graphic_name[2] = 0;  // 图形名称，不要重复
	message_onegraphic.Client_graphic_single.grapic_data_struct.operate_tpye = 1;
	message_onegraphic.Client_graphic_single.grapic_data_struct.graphic_tpye = 2;
	message_onegraphic.Client_graphic_single.grapic_data_struct.layer = 0;
	message_onegraphic.Client_graphic_single.grapic_data_struct.color = 7;
	message_onegraphic.Client_graphic_single.grapic_data_struct.start_angle = 0;
	message_onegraphic.Client_graphic_single.grapic_data_struct.end_angle = 0;
	message_onegraphic.Client_graphic_single.grapic_data_struct.width = 10;
	message_onegraphic.Client_graphic_single.grapic_data_struct.start_x = 960;
	message_onegraphic.Client_graphic_single.grapic_data_struct.start_y = 440;
	message_onegraphic.Client_graphic_single.grapic_data_struct.radius = 100;
	message_onegraphic.Client_graphic_single.grapic_data_struct.end_x = 0;
	message_onegraphic.Client_graphic_single.grapic_data_struct.end_y = 0;
	
	append_crc8_check_sum(&message_onegraphic.FrameHead.sof,sizeof(message_onegraphic.FrameHead));
	append_crc16_check_sum(&message_onegraphic.FrameHead.sof,sizeof(message_onegraphic.FrameHead)
	+ CMD_LEN + CRC_LEN + message_onegraphic.FrameHead.dataLenth);
	
	HAL_UART_Transmit(&JUDGE_HUART,&message_onegraphic.FrameHead.sof,sizeof(message_onegraphic),0xffff);
}

void send_dele(uint8_t lay)
{
	message_dele.FrameHead.sof = 0xA5;
	message_dele.FrameHead.dataLenth = 8;
	message_dele.FrameHead.seq = (seqcount++) & 0xFF;                  //帧头填充
	
	message_dele.CmdId = 0x0301;                                       // 机器人交互命令码
	
	message_dele.Interactive_header_data.data_cmd_id = 0x0100;
	message_dele.Interactive_header_data.sender_ID = judge_rece_mesg.robot_state.robot_id;                
	message_dele.Interactive_header_data.receiver_ID = 0x0100 + judge_rece_mesg.robot_state.robot_id;     
	
	message_dele.Client_Dele.operate_tpye = 1;
	message_dele.Client_Dele.layer = lay;

	
	append_crc8_check_sum(&message_dele.FrameHead.sof,sizeof(message_dele.FrameHead));
	append_crc16_check_sum(&message_dele.FrameHead.sof,sizeof(message_dele.FrameHead)
	+ CMD_LEN + CRC_LEN + message_dele.FrameHead.dataLenth);
	
	HAL_UART_Transmit(&JUDGE_HUART,&message_dele.FrameHead.sof,sizeof(message_dele),0xffff);
}

void renew_float(void)
{
	message_onefloat.FrameHead.sof = 0xA5;
	message_onefloat.FrameHead.dataLenth = 21;
	message_onefloat.FrameHead.seq = (seqcount++) & 0xFF;                  //帧头填充
	
	message_onefloat.CmdId = 0x0301;                                       // 机器人交互命令码
	
	message_onefloat.Interactive_header_data.data_cmd_id = 0x0101;
	message_onefloat.Interactive_header_data.sender_ID = judge_rece_mesg.robot_state.robot_id;                
	message_onefloat.Interactive_header_data.receiver_ID = 0x0100 + judge_rece_mesg.robot_state.robot_id;  
	
	message_onefloat.Client_float_single.graphic_name[0] = 0;
	message_onefloat.Client_float_single.graphic_name[1] = 0;
	message_onefloat.Client_float_single.graphic_name[2] = 4;  // 图形名称，不要重复
	message_onefloat.Client_float_single.operate_tpye = 1;
	message_onefloat.Client_float_single.graphic_tpye = 5;
	message_onefloat.Client_float_single.layer = 1;
	message_onefloat.Client_float_single.color = 0;
	message_onefloat.Client_float_single.start_angle = 50;
	message_onefloat.Client_float_single.end_angle = 1;
	message_onefloat.Client_float_single.width = 5;
	message_onefloat.Client_float_single.start_x = 400;
	message_onefloat.Client_float_single.start_y = 900;
//	message_onefloat.Client_float_single.num = judge_rece_mesg.chassis_power_info.chassis_power;
	message_onefloat.Client_float_single.num = 2.0f;
	append_crc8_check_sum(&message_onefloat.FrameHead.sof,sizeof(message_onefloat.FrameHead));
	append_crc16_check_sum(&message_onefloat.FrameHead.sof,sizeof(message_onefloat.FrameHead)
	+ CMD_LEN + CRC_LEN + message_onefloat.FrameHead.dataLenth);
	
	HAL_UART_Transmit(&JUDGE_HUART,&message_onefloat.FrameHead.sof,sizeof(message_onefloat),0xffff);
}
