#ifndef _UI_TASK_H
#define _UI_TASK_H
#include "stm32f4xx_hal.h"
#include "judgement_info.h"

//version:����ϵͳ����Э��V1.1
//����Ƶ�ʹٷ��Ƽ�������2Hz


#define UI_TIME 500

typedef __packed struct
{
uint16_t data_cmd_id;											//���ݶε�����ID
uint16_t sender_ID;                       //�����ߵ�ID
uint16_t receiver_ID;                     //�����ߵ�ID
}ext_student_interactive_header_data_t;   //�������ݽ�����Ϣ

//����ID data_cmd_id ���ȣ�ͷ�ṹ����+�������ݶγ��ȣ�    ����˵��
//0x0200~0x02FF      6+n                                  ���������˼�ͨ��
//0x0100             6+2                                  �ͻ���ɾ��ͼ��
//0x0101             6+15                                 �ͻ��˻���һ��ͼ��
//0x0102             6+30                                 �ͻ��˻��ƶ���ͼ��
//0x0103             6+75                                 �ͻ��˻������ͼ��
//0x0104             6+105                                �ͻ��˻����߸�ͼ��
//0x0110             6+45                                 �ͻ��˻����ַ�ͼ

//�����ߵ�ID sender_ID�� 
//1��  Ӣ��(��)�� 2��   ����(��)�� 3/4/5��       ����(��)�� 6��   ����(��)�� 7��   �ڱ�(��)��9��   �״�վ���죩
//101��Ӣ��(��)�� 102�� ����(��)�� 103/104/105�� ����(��)�� 106�� ����(��)�� 107�� �ڱ�(��)��109�� �״�վ������

//�����ߵ�ID receiver_ID:
//0x0101��               Ӣ�۲����ֿͻ���(��)�� 0x0102�� ���̲����ֿͻ���(��)��
//0x0103/0x0104/0x0105�� ���������ֿͻ���(��)�� 0x0106�� ���в����ֿͻ���(��)��
//0x0165��               Ӣ�۲����ֿͻ���(��)�� 0x0166�� ���̲����ֿͻ���(��)�� 
//0x0167/0x0168/0x0169�� ���������ֿͻ���(��)�� 0x016A�� ���в����ֿͻ���(��)�� 



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
}graphic_data_struct_t;                   //ͼ������

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
}float_data_struct_t;                     //����������

typedef __packed struct
{
graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;      //�ͻ��˻���һ��ͼ��

typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[5];
}ext_client_custom_graphic_five_t;        //�ͻ��˻������ͼ��

typedef __packed struct
{
uint8_t operate_tpye;         						//ͼ�β���0: �ղ�����1: ɾ��ͼ�㣻2: ɾ������
uint8_t layer;             							  //ͼ������ 0~9
}ext_client_custom_graphic_delete_t;      //�ͻ���ɾ��ͼ��


typedef __packed struct
{
	tFrameHeader                           FrameHead;                 //֡ͷ
	uint16_t                               CmdId;                     //������ 
	ext_student_interactive_header_data_t  Interactive_header_data;   //�������ݽ�����Ϣ�����ݶΣ�
	ext_client_custom_graphic_single_t     Client_graphic_single;     //ͼ�����ݣ����ݶΣ�
 	uint16_t                               CRC16;
}graphic_single_t;                        //��Բ�ṹ��

typedef __packed struct
{
	tFrameHeader                           FrameHead;                 //֡ͷ
	uint16_t                               CmdId;                     //������ 
	ext_student_interactive_header_data_t  Interactive_header_data;   //�������ݽ�����Ϣ�����ݶΣ�
	ext_client_custom_graphic_five_t       Client_graphic_five;       //ͼ�����ݣ����ݶΣ�
 	uint16_t                               CRC16;
}graphic_five_t;                         // ��ͼ�ṹ��

typedef __packed struct
{
	tFrameHeader                           FrameHead;                 //֡ͷ
	uint16_t                               CmdId;                     //������ 
	ext_student_interactive_header_data_t  Interactive_header_data;   //�������ݽ�����Ϣ�����ݶΣ�
	ext_client_custom_graphic_delete_t     Client_Dele;               //ͼ�����ݣ����ݶΣ�
 	uint16_t                               CRC16;
}graphic_dele_t;                         // ɾͼ��ṹ��

typedef __packed struct
{
	tFrameHeader                           FrameHead;                 //֡ͷ
	uint16_t                               CmdId;                     //������ 
	ext_student_interactive_header_data_t  Interactive_header_data;   //�������ݽ�����Ϣ�����ݶΣ�
	float_data_struct_t                    Client_float_single;       //ͼ�����ݣ����ݶΣ�
 	uint16_t                               CRC16;
}float_single_t;                       // �������ṹ��


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
