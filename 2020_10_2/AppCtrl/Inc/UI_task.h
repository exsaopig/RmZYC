#ifndef _UI_TASK_H
#define _UI_TASK_H
#include "stm32f4xx_hal.h"
#include "judgement_info.h"

//version:裁判系统串口协议V1.1
//发送频率官方推荐不高于2Hz


#define UI_TIME 500

typedef __packed struct
{
uint16_t data_cmd_id;											//数据段的内容ID
uint16_t sender_ID;                       //发送者的ID
uint16_t receiver_ID;                     //接收者的ID
}ext_student_interactive_header_data_t;   //交互数据接收信息

//内容ID data_cmd_id 长度（头结构长度+内容数据段长度）    功能说明
//0x0200~0x02FF      6+n                                  己方机器人间通信
//0x0100             6+2                                  客户端删除图形
//0x0101             6+15                                 客户端绘制一个图形
//0x0102             6+30                                 客户端绘制二个图形
//0x0103             6+75                                 客户端绘制五个图形
//0x0104             6+105                                客户端绘制七个图形
//0x0110             6+45                                 客户端绘制字符图

//发送者的ID sender_ID： 
//1，  英雄(红)； 2，   工程(红)； 3/4/5，       步兵(红)； 6，   空中(红)； 7，   哨兵(红)；9，   雷达站（红）
//101，英雄(蓝)； 102， 工程(蓝)； 103/104/105， 步兵(蓝)； 106， 空中(蓝)； 107， 哨兵(蓝)；109， 雷达站（蓝）

//接收者的ID receiver_ID:
//0x0101，               英雄操作手客户端(红)； 0x0102， 工程操作手客户端(红)；
//0x0103/0x0104/0x0105， 步兵操作手客户端(红)； 0x0106， 空中操作手客户端(红)；
//0x0165，               英雄操作手客户端(蓝)； 0x0166， 工程操作手客户端(蓝)； 
//0x0167/0x0168/0x0169， 步兵操作手客户端(蓝)； 0x016A， 空中操作手客户端(蓝)； 



typedef __packed struct
{
uint8_t graphic_name[3];
uint32_t operate_tpye:3;
uint32_t graphic_tpye:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t radius:10;
uint32_t end_x:11;
uint32_t end_y:11;
}graphic_data_struct_t;                   //图形数据

typedef __packed struct
{
uint8_t graphic_name[3];
uint32_t operate_tpye:3;
uint32_t graphic_tpye:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
float    num;
}float_data_struct_t;                     //浮点数数据

typedef __packed struct
{
graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;      //客户端绘制一个图形

typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[5];
}ext_client_custom_graphic_five_t;        //客户端绘制五个图形

typedef __packed struct
{
uint8_t operate_tpye;         						//图形操作0: 空操作；1: 删除图层；2: 删除所有
uint8_t layer;             							  //图层数： 0~9
}ext_client_custom_graphic_delete_t;      //客户端删除图形


typedef __packed struct
{
	tFrameHeader                           FrameHead;                 //帧头
	uint16_t                               CmdId;                     //命令码 
	ext_student_interactive_header_data_t  Interactive_header_data;   //交互数据接收信息（数据段）
	ext_client_custom_graphic_single_t     Client_graphic_single;     //图形数据（数据段）
 	uint16_t                               CRC16;
}graphic_single_t;                        //单圆结构体

typedef __packed struct
{
	tFrameHeader                           FrameHead;                 //帧头
	uint16_t                               CmdId;                     //命令码 
	ext_student_interactive_header_data_t  Interactive_header_data;   //交互数据接收信息（数据段）
	ext_client_custom_graphic_five_t       Client_graphic_five;       //图形数据（数据段）
 	uint16_t                               CRC16;
}graphic_five_t;                         // 五图结构体

typedef __packed struct
{
	tFrameHeader                           FrameHead;                 //帧头
	uint16_t                               CmdId;                     //命令码 
	ext_student_interactive_header_data_t  Interactive_header_data;   //交互数据接收信息（数据段）
	ext_client_custom_graphic_delete_t     Client_Dele;               //图形数据（数据段）
 	uint16_t                               CRC16;
}graphic_dele_t;                         // 删图层结构体

typedef __packed struct
{
	tFrameHeader                           FrameHead;                 //帧头
	uint16_t                               CmdId;                     //命令码 
	ext_student_interactive_header_data_t  Interactive_header_data;   //交互数据接收信息（数据段）
	float_data_struct_t                    Client_float_single;       //图形数据（数据段）
 	uint16_t                               CRC16;
}float_single_t;                       // 浮点数结构体


extern graphic_single_t message_onegraphic;
extern graphic_five_t message_onebull;
extern graphic_dele_t message_dele;
extern float_single_t message_onefloat;
extern uint8_t UI_bull;

void UI_Init(void);
void send_bull_init(void);
void send_circle_single(void);
void send_bull(void);
void send_dele(uint8_t lay);
void renew_float(void);
void ui_task(void const *argu);


#endif
