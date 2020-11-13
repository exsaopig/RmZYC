#include "shoot_task.h"
#include "string.h"
#include "sys_config.h"
#include "bsp_io.h"
#include "pid.h"
#include "bsp_can.h"
#include "remote_info.h"
#include "gimbal_task.h"
#include "pc_info.h"
#include "math.h"
#include "judgement_info.h"
#include "cmsis_os.h"
#include "sys_config.h"

#define WARNING_HEAT 80
#define HEAT_ONE_BULLET 10
#define TRIGGER_EOCODER_A_BULLET 49152

shoot_t   shoot;
trigger_t trig;

int shoot_time_ms;
//�ǵÿ�һ��dir
int triple_shoot_count = 0;  //��ֹ������ģʽ��ת��̫�죬��ûд
int single_shoot_time = 0;   //��ֹ������ͨ��ʱ���̫��,����ͬ
int single_shoot_onetime = 0;
UBaseType_t shoot_stack_surplus;
int32_t left_spd,right_spd;


void shoot_heat_limit(void)
{
//	float v;
  int zidan_count;	
//	v = judge_rece_mesg.shoot_data.bullet_speed;
	if(shoot.firemode == SINGLE_SHOOT)
	{
		zidan_count = 1;
	}
	else if(shoot.firemode == TRIPLE_SHOOT)
	{
		zidan_count = 3;
	}
	else
	{
		zidan_count = 1;
	}
	switch(shoot.shootlevel)
	{
		case 1:
			if((175-judge_rece_mesg.chassis_power_info.shooter_heat0)<zidan_count * HEAT_ONE_BULLET)		//�ȼ�һ�������ޣ�180
			{
				shoot.c_shoot_cmd = 0;
			  shoot.shoot_cmd = 0;
			}
			shoot.trigger_motor_speed = 5000;
			break;
		case 2:
			if((235-judge_rece_mesg.chassis_power_info.shooter_heat0)<zidan_count * HEAT_ONE_BULLET)		//�ȼ����������ޣ�240
			{
				shoot.c_shoot_cmd = 0;
			  shoot.shoot_cmd = 0;
			}
			shoot.trigger_motor_speed = 5000;
			break;
		case 3:
			if((295-judge_rece_mesg.chassis_power_info.shooter_heat0)<zidan_count * HEAT_ONE_BULLET)		//�ȼ����������ޣ�300
			{
				shoot.c_shoot_cmd = 0;
			  shoot.shoot_cmd = 0;
			}
			shoot.trigger_motor_speed = 6000;
			break;
		default:
			if((175-judge_rece_mesg.chassis_power_info.shooter_heat0)<zidan_count * HEAT_ONE_BULLET)
			{
				shoot.c_shoot_cmd = 0;
			  shoot.shoot_cmd = 0;
			}
			shoot.trigger_motor_speed = 2000;
			break;
	}
	if(((judge_rece_mesg.buff_data.power_rune_buff)&0x03) == 1)		//�����˹����ӳɣ���ʱ�˺��ӱ�
	{
		shoot.trigger_motor_speed *= 2; 
	}
}

void shoot_task(void const *argu)
{
	osEvent event;
	while (1)
  {
		event = osSignalWait(SHOT_TASK_EXE_SIGNAL, osWaitForever);
    
    if (event.status == osEventSignal)
    {
			
			if (event.value.signals & SHOT_TASK_EXE_SIGNAL)
      {
				setfric_wheel_spd(); //2020���ӵ��ٶ�Ϊһ�̶�ֵ //����Ħ����ת��  �������ģʽ�����㣬��������ȫ�Զ�����Щ��ģʽ�л�ʱ�ģ� �Լ������������ת��
				shoot_heat_limit();
				fric_wheel_ctrl();   //Ħ���ֿ���
					
				if (!shoot.fric_wheel_run)  // ���Ħ����û��������
				{
					shoot.shoot_cmd   = 0;    //������
					shoot.c_shoot_cmd = 0;    //������
				}
					
				if (shoot.fric_wheel_run)   //Ħ�����Ѵ򿪣������
				{
					if (shoot.shoot_cmd==1)   //����
					{
						//trig.pos_ref = moto_trigger.total_encoder; //���²������������ֵ
						switch(shoot.firemode)
						{
							case(TRIPLE_SHOOT):                      //����漰ģʽʱ������
								trig.pos_ref += 147456 * trig.dir;     //�������ת��λ�ü���������
								break;
							case(SINGLE_SHOOT):                      //����ǵ���
								trig.pos_ref += 49152 * trig.dir;      //��1/6*һȦ
								break;
							case(FULL_AUTOSHOOT):                    //���������²�����ȫ�Զ�����һֱ������
								shoot.shoot_cmd = 0;
								shoot.c_shoot_cmd = 1;
								break;
							case(AUTO_CTRL_SHOOT):                   //�Զ����
							{
								if(pc_recv_mesg.shoot_contrl_data.shoot_mode!=0)    //���յ�pcָ��
								{ 
									single_shoot_time = HAL_GetTick();                //��¼ʱ�䣬������Ƶ
									if(pc_recv_mesg.shoot_contrl_data.shoot_mode == SHOOT_ON)
									{									
										if(single_shoot_time-single_shoot_onetime>100) //100Ϊ������Ƶ
										{
											trig.pos_ref += 49152 * trig.dir;            //����
											single_shoot_onetime = HAL_GetTick();        //��¼ʱ�䣬������Ƶ
										}
									}
									pc_recv_mesg.shoot_contrl_data.shoot_mode = 0;    //����һ����0��ֹһֱ��
								}
								break;
							}
						}
						shoot.shoot_cmd = 2; //����һ��������־λ��2����ֹһֱ��                         
					}
					
								
					if (shoot.c_shoot_cmd==1)                        //��������ģʽ,ֻ���ٶȻ����������һֱת
					{
						trig.pos_ref = moto_trigger.total_encoder-moto_trigger.total_encoder%49152; //�˳�����ʱ�����������Ѳ���λ�ã������Ƴ�ʼλ��
						trig.spd_ref = shoot.trigger_motor_speed * trig.dir;  //ȫ�ٵ�ʱ�򲥵��ٶȣ����Ը��ݵȼ��޸ģ��о����岻��
					}
					else
					{
						//�������λ�û�pid
						pid_calc(&pid_trigger, moto_trigger.total_encoder / 100, trig.pos_ref / 100);
						trig.spd_ref = pid_trigger.out;                                 //�����������ģʽ���ٶȵ�ָ��ֵ����Ϊ����λ�û�pid�����ֵ
					}
					if(shoot.c_shoot_cmd!=1&&shoot.shoot_cmd!=1)
					{
						pid_calc(&pid_trigger, moto_trigger.total_encoder / 100, trig.pos_ref / 100);
						trig.spd_ref = pid_trigger.out; 
					}
					block_bullet_handler();                                           //��������
					empty_judge();
					pid_calc(&pid_trigger_spd, moto_trigger.speed_rpm, trig.spd_ref); //�ٶȻ�pid���������������뷢����gimbal_task
				}
				else
				{
					trig.spd_ref = 0;                                                 //Ħ���ֲ���ʱ�������ܶ�
					pid_calc(&pid_trigger_spd, moto_trigger.speed_rpm, trig.spd_ref); //�ٶȻ�pid���������������뷢����gimbal_task
					//pid_trigger_spd.out = 0;                            //Ħ����û���������ʱ�����������
				}
	
			}	
		}
		shoot_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
}
//��Ҫ��config.h�и�pwm��ʱ���Ŀڣ���Ҫ��bsp_io.h�иļ���Ĭ��io�
//��Ħ���ָ�Ϊcan2���
static void fric_wheel_ctrl(void)
{
  if (shoot.fric_wheel_run)
  {
			turn_on_friction_wheel(shoot.fric_wheel_spd);
      turn_on_laser(); //�ļ���
  }
  else
  {
    turn_off_friction_wheel();
		
    turn_off_laser();
  }
}

void shoot_param_init(void)   //��Ҫ���������г�ʼ��
{
  memset(&shoot, 0, sizeof(shoot_t));      //shoot��0
  
  shoot.ctrl_mode = REMOTE_CTRL_SHOT;      //�����������ģʽΪң��������ģʽ
	shoot.firemode = SINGLE_SHOOT;           //����ģʽΪ����
  shoot.fric_wheel_spd = SPEED_LEVEL_0;    	 //�����ٶ�Ϊ�㼶
  shoot.shootlevel = LEVEL0;               //�����ȼ�Ϊ�㼶
	shoot.trigger_motor_speed = 1000;
	shoot.shoot_bullets = 0;
	shoot.remain_bullets = 0;
  
  trig.dir             = TIGGER_DIR; 
	//����Ŀ�ⶼû��
  trig.feed_bullet_spd = 2000;
  trig.c_shoot_spd     = 4000;
  trig.one_sta         = TRIG_INIT;
}

void setfric_wheel_spd()           
{
	static int fric_wheel_count = 0; //������ͨ��ʱ��������ֹĦ����Ƶ������
	static uint16_t SPEED_FIXED = 0;
//	if(judge_rece_mesg.robot_state.shooter_heat0_speed_limit < ) shoot.shootlevel = LEVEL0;
	if( judge_rece_mesg.robot_state.shooter_heat0_speed_limit < 18) shoot.shootlevel = LEVEL0;
	else if(judge_rece_mesg.robot_state.shooter_heat0_speed_limit >= 18 && judge_rece_mesg.robot_state.shooter_heat0_speed_limit < 22) shoot.shootlevel = LEVEL1;
	else if(judge_rece_mesg.robot_state.shooter_heat0_speed_limit >= 22 && judge_rece_mesg.robot_state.shooter_heat0_speed_limit < 30) shoot.shootlevel = LEVEL2;
	else if(judge_rece_mesg.robot_state.shooter_heat0_speed_limit >= 30 ) shoot.shootlevel = LEVEL3;
	switch(shoot.shootlevel)
	{
		case(LEVEL0):
		{
			SPEED_FIXED = SPEED_LEVEL_0;
		break;
		}
		case(LEVEL1):
		{
			SPEED_FIXED = SPEED_LEVEL_1;
		break;
		}
		case(LEVEL2):
		{
			SPEED_FIXED = SPEED_LEVEL_2;
		break;
		}
		case(LEVEL3):
		{
			SPEED_FIXED = SPEED_LEVEL_3;
		break;
		}
	}
	switch(shoot.firemode)
	{
		case(SINGLE_SHOOT):                            //����
		{																		//2020�굥���ӵ�����Ϊ�̶�ֵ
				/*switch(shoot.shootlevel)
	      {
					case(LEVEL1):
						shoot.fric_wheel_spd = SINGLE_LEVEL1;  //����һ��
						break;
					case(LEVEL2):
						shoot.fric_wheel_spd = SINGLE_LEVEL2;  //�������
						break; 
					case(LEVEL3):
						shoot.fric_wheel_spd = SINGLE_LEVEL3;  //��������
						break;
					default:
						shoot.fric_wheel_spd = SINGLE_LEVEL1;  
						break;
				}*/
				shoot.fric_wheel_spd = SPEED_FIXED;	
		break;
		}
		case(TRIPLE_SHOOT):                            //������
		{
				/*switch(shoot.shootlevel)
	      {
					case(LEVEL1):
						shoot.fric_wheel_spd = TRIPLE_LEVEL1; //һ��������
						break;
					case(LEVEL2):
						shoot.fric_wheel_spd = TRIPLE_LEVEL2; //����������
						break;
					case(LEVEL3):
						shoot.fric_wheel_spd = TRIPLE_LEVEL3; //����������
						break;
					default:
						shoot.fric_wheel_spd = TRIPLE_LEVEL1;
					  break;
				}*/
				shoot.fric_wheel_spd = SPEED_FIXED;
			break;
			}
		case(FULL_AUTOSHOOT):                            //ȫ�Զ���ָ��������סһֱ��
		{
				/*switch(shoot.shootlevel)
	      {
					case(LEVEL1):
						 shoot.fric_wheel_spd = FULL_AUTO_LEVEL1;//һ��ȫ�Զ�
						break;
					case(LEVEL2):
						 shoot.fric_wheel_spd = FULL_AUTO_LEVEL2;//����ȫ�Զ�
						break;
					case(LEVEL3):
						 shoot.fric_wheel_spd = FULL_AUTO_LEVEL3;//����ȫ�Զ�
						break;
					default:
						shoot.fric_wheel_spd = FULL_AUTO_LEVEL1;
					  break;
				}*/
			shoot.fric_wheel_spd = SPEED_FIXED;
		break;
		}
		case(AUTO_CTRL_SHOOT):                             //�Զ������pc���ƣ�
		{
			if(pc_recv_mesg.shoot_contrl_data.shoot_mode!=0) //���յ�pc���Ʒ������ָ��
			{
				if(pc_recv_mesg.shoot_contrl_data.shoot_mode == SHOOT_OFF) //��⵽�ر�Ħ����
				{
					if(fric_wheel_count++>=100)                              //��������ﵽ100��ر�Ħ���֣�����Ħ���ּ�ͣ
				  {
					  shoot.fric_wheel_run = 0;                              
				  }
		   	}
				else if(pc_recv_mesg.shoot_contrl_data.shoot_mode == SHOOT_FRIC_ON) //��Ħ����
				{
					shoot.fric_wheel_run = 1; //��Ħ����
					shoot.shoot_cmd = 0;      //������
					shoot.c_shoot_cmd = 0;    //������
					shoot.fric_wheel_spd = SPEED_FIXED;
					if(pc_recv_mesg.shoot_contrl_data.shoot_speed == 0.0f)
						shoot.fric_wheel_spd = 0;
					fric_wheel_count = 0;
				}
				else if(pc_recv_mesg.shoot_contrl_data.shoot_mode == SHOOT_ON) //��
				{
					shoot.fric_wheel_run = 1;  //��Ħ����
					shoot.shoot_cmd = 1;       //������
					shoot.c_shoot_cmd = 0;     //������
					shoot.fric_wheel_spd = SPEED_FIXED;
					if(pc_recv_mesg.shoot_contrl_data.shoot_speed == 0.0f)
						shoot.fric_wheel_spd = 0;
					fric_wheel_count = 0;
				}
				else
				{
					shoot.fric_wheel_spd = 0;
					shoot.fric_wheel_run = 0;
				}
			}
			else
			{
				shoot.fric_wheel_spd = 0;  //�ر�Ħ����
				shoot.fric_wheel_run = 0;
			}
		}
	}
}


//������⼰����
void block_bullet_handler(void)
{
	static uint32_t stall_count5 = 0;
	static uint32_t stall_count1 = 0;
  static uint32_t stall_count2 = 0;
	static uint32_t stall_count3 = 0;
	static uint32_t stall_count4 = 0;     //��Ӧ����ģʽ��ͬ�ļ���  
  static uint8_t  stall_f = 0;          //��ʾ�Ƿ񿨵�
	static uint32_t stall_inv_count = 0;  //������ת�ļ���
	
	if(shoot.shoot_cmd==0&&shoot.c_shoot_cmd==0)
	{
		stall_count1=0;
		stall_count2=0;
		stall_count3=0;
		stall_count4=0;
		stall_count5=0;
	}
	switch(shoot.firemode)
	{
		case(   SINGLE_SHOOT ):
		stall_count5=0;
		stall_count2=0;
		stall_count3=0;
		stall_count4=0;
			if(fabs(moto_trigger.total_encoder-trig.pos_ref)>2000)
				stall_count1++;
			else
				stall_count1=0;
		//	printf("  %d\r\n",stall_count1);
			break;
		case(   TRIPLE_SHOOT ):
		stall_count5=0;
		stall_count1=0;
		stall_count3=0;
		stall_count4=0;
			if(fabs(moto_trigger.total_encoder-trig.pos_ref)>2000)
				stall_count2++;
			else
				stall_count2=0;
//
		break;
		case(   FULL_AUTOSHOOT ):
		stall_count5=0;
		stall_count2=0;
		stall_count1=0;
		stall_count4=0;
			if(fabs(moto_trigger.speed_rpm)<500)
				stall_count3++;
			else
				stall_count3=0;
		break;
		case(AUTO_CTRL_SHOOT):
		{
		stall_count5=0;
		stall_count2=0;
		stall_count1=0;
		stall_count3=0;
			if(fabs(moto_trigger.total_encoder-trig.pos_ref)>2000)
				stall_count4++;
			else
				stall_count4=0;
			break;
		}
	}
  
  if (stall_count1 >= 100||stall_count2 >= 250||stall_count3 >= 50||stall_count4 > 100||stall_count5>100)         // 
  {
    stall_f = 1;
    stall_count1 = 0;
		stall_count2 = 0;
		stall_count3 = 0;
		stall_count4 = 0;
		stall_count5=0;
  }
  
  if (stall_f == 1)
  {
    stall_inv_count++;
    
    if (stall_inv_count >= 20)  //��תʱ��//0.1s
    {
      stall_f = 0;
      stall_inv_count = 0;
    }
    else
      trig.spd_ref =2000*(-TIGGER_DIR);  //��ת�ٶ�
  }
}

void turn_on_friction_wheel(uint16_t spd)
{
//  LEFT_FRICTION  = spd;
//  RIGHT_FIRCTION = spd;
	  left_spd  = -spd;
	  right_spd = spd;
		pid_calc(&pid_leftfric_spd,moto_leftfric.speed_rpm,left_spd);
		pid_calc(&pid_rightfric_spd,moto_rightfric.speed_rpm,right_spd);
}

void turn_off_friction_wheel(void)
{
//  LEFT_FRICTION  = 0;
//  RIGHT_FIRCTION = 0;
	  left_spd  = 0;
	  right_spd = 0;
		pid_calc(&pid_leftfric_spd,moto_leftfric.speed_rpm,left_spd);
		pid_calc(&pid_rightfric_spd,moto_rightfric.speed_rpm,right_spd);
}



//�ղּ��
void  empty_judge(void)
{	
	static uint8_t fric_flag = 0;
	static uint8_t shoot_flag =0;
	static uint16_t shoot_number_cnt_last;
	static int32_t trig_encoder;
	if(shoot.fric_wheel_run && (fric_flag == 0))
	{
		fric_flag = 1;
		trig_encoder = moto_trigger.total_encoder;
	}
	if(!shoot.fric_wheel_run)
		fric_flag = 0;
	
	if((shoot.c_shoot_cmd == 1)||(shoot.shoot_cmd == 2))
	{
		if(shoot_flag == 0)
		{
			shoot_number_cnt_last = shoot.remain_bullets;
			shoot_flag = 1;
		}
	}	
	
	if(shoot.shoot_cmd == 2)
	{
		if(moto_trigger.total_encoder - trig_encoder >= TRIGGER_EOCODER_A_BULLET*7)
		{
			if(shoot_number_cnt_last == shoot.remain_bullets)
			{	
				shoot.remain_bullets = 0;
			}
			else shoot_flag = 0;
			
			trig_encoder = moto_trigger.total_encoder;
		}
	}
	if(shoot.c_shoot_cmd == 1)
	{
		if(moto_trigger.total_encoder - trig_encoder >= TRIGGER_EOCODER_A_BULLET*12)
		{
			if(shoot_number_cnt_last == shoot.remain_bullets)
			{	
				shoot.remain_bullets = 0;
			}
			else shoot_flag = 0;
			
			trig_encoder = moto_trigger.total_encoder;
		}
	}
}
