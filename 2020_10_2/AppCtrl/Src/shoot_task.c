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
//记得看一下dir
int triple_shoot_count = 0;  //防止三连发模式下转的太快，还没写
int single_shoot_time = 0;   //防止与妙算通信时打的太快,下面同
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
			if((175-judge_rece_mesg.chassis_power_info.shooter_heat0)<zidan_count * HEAT_ONE_BULLET)		//等级一热量上限：180
			{
				shoot.c_shoot_cmd = 0;
			  shoot.shoot_cmd = 0;
			}
			shoot.trigger_motor_speed = 5000;
			break;
		case 2:
			if((235-judge_rece_mesg.chassis_power_info.shooter_heat0)<zidan_count * HEAT_ONE_BULLET)		//等级二热量上限：240
			{
				shoot.c_shoot_cmd = 0;
			  shoot.shoot_cmd = 0;
			}
			shoot.trigger_motor_speed = 5000;
			break;
		case 3:
			if((295-judge_rece_mesg.chassis_power_info.shooter_heat0)<zidan_count * HEAT_ONE_BULLET)		//等级三热量上限：300
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
	if(((judge_rece_mesg.buff_data.power_rune_buff)&0x03) == 1)		//机器人攻击加成，此时伤害加倍
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
				setfric_wheel_spd(); //2020年子弹速度为一固定值 //设置摩擦轮转速  根据射击模式（单点，三连发，全自动，这些在模式切换时改） 以及步兵级别给定转速
				shoot_heat_limit();
				fric_wheel_ctrl();   //摩擦轮控制
					
				if (!shoot.fric_wheel_run)  // 如果摩擦轮没开，不打蛋
				{
					shoot.shoot_cmd   = 0;    //不单发
					shoot.c_shoot_cmd = 0;    //不连发
				}
					
				if (shoot.fric_wheel_run)   //摩擦轮已打开，允许打蛋
				{
					if (shoot.shoot_cmd==1)   //单发
					{
						//trig.pos_ref = moto_trigger.total_encoder; //更新拨弹电机编码器值
						switch(shoot.firemode)
						{
							case(TRIPLE_SHOOT):                      //如果涉及模式时三连发
								trig.pos_ref += 147456 * trig.dir;     //拨弹电机转的位置加三倍单发
								break;
							case(SINGLE_SHOOT):                      //如果是单发
								trig.pos_ref += 49152 * trig.dir;      //加1/6*一圈
								break;
							case(FULL_AUTOSHOOT):                    //单发条件下不允许全自动（即一直连发）
								shoot.shoot_cmd = 0;
								shoot.c_shoot_cmd = 1;
								break;
							case(AUTO_CTRL_SHOOT):                   //自动射击
							{
								if(pc_recv_mesg.shoot_contrl_data.shoot_mode!=0)    //接收到pc指令
								{ 
									single_shoot_time = HAL_GetTick();                //记录时间，控制射频
									if(pc_recv_mesg.shoot_contrl_data.shoot_mode == SHOOT_ON)
									{									
										if(single_shoot_time-single_shoot_onetime>100) //100为控制射频
										{
											trig.pos_ref += 49152 * trig.dir;            //单发
											single_shoot_onetime = HAL_GetTick();        //记录时间，控制射频
										}
									}
									pc_recv_mesg.shoot_contrl_data.shoot_mode = 0;    //打完一蛋清0防止一直打
								}
								break;
							}
						}
						shoot.shoot_cmd = 2; //打完一蛋单发标志位归2，防止一直打                         
					}
					
								
					if (shoot.c_shoot_cmd==1)                        //开启连发模式,只用速度环，拨弹电机一直转
					{
						trig.pos_ref = moto_trigger.total_encoder-moto_trigger.total_encoder%49152; //退出连发时拨弹电机在最佳拨弹位置，即近似初始位置
						trig.spd_ref = shoot.trigger_motor_speed * trig.dir;  //全速的时候播弹速度，可以根据等级修改，感觉意义不大
					}
					else
					{
						//拨弹电机位置环pid
						pid_calc(&pid_trigger, moto_trigger.total_encoder / 100, trig.pos_ref / 100);
						trig.spd_ref = pid_trigger.out;                                 //如果不是连发模式，速度的指定值设置为单发位置环pid算出的值
					}
					if(shoot.c_shoot_cmd!=1&&shoot.shoot_cmd!=1)
					{
						pid_calc(&pid_trigger, moto_trigger.total_encoder / 100, trig.pos_ref / 100);
						trig.spd_ref = pid_trigger.out; 
					}
					block_bullet_handler();                                           //卡弹处理
					empty_judge();
					pid_calc(&pid_trigger_spd, moto_trigger.speed_rpm, trig.spd_ref); //速度环pid算输出，具体输出与发送在gimbal_task
				}
				else
				{
					trig.spd_ref = 0;                                                 //摩擦轮不开时拨弹不能动
					pid_calc(&pid_trigger_spd, moto_trigger.speed_rpm, trig.spd_ref); //速度环pid算输出，具体输出与发送在gimbal_task
					//pid_trigger_spd.out = 0;                            //摩擦轮没开不允许打蛋时拨弹电机放松
				}
	
			}	
		}
		shoot_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
}
//需要在config.h中改pwm定时器的口，需要在bsp_io.h中改激光默认io�
//现摩擦轮改为can2输出
static void fric_wheel_ctrl(void)
{
  if (shoot.fric_wheel_run)
  {
			turn_on_friction_wheel(shoot.fric_wheel_spd);
      turn_on_laser(); //改激光
  }
  else
  {
    turn_off_friction_wheel();
		
    turn_off_laser();
  }
}

void shoot_param_init(void)   //需要在主函数中初始化
{
  memset(&shoot, 0, sizeof(shoot_t));      //shoot清0
  
  shoot.ctrl_mode = REMOTE_CTRL_SHOT;      //发射机构控制模式为遥控器控制模式
	shoot.firemode = SINGLE_SHOOT;           //发射模式为单发
  shoot.fric_wheel_spd = SPEED_LEVEL_0;    	 //发射速度为零级
  shoot.shootlevel = LEVEL0;               //步兵等级为零级
	shoot.trigger_motor_speed = 1000;
	shoot.shoot_bullets = 0;
	shoot.remain_bullets = 0;
  
  trig.dir             = TIGGER_DIR; 
	//以下目测都没用
  trig.feed_bullet_spd = 2000;
  trig.c_shoot_spd     = 4000;
  trig.one_sta         = TRIG_INIT;
}

void setfric_wheel_spd()           
{
	static int fric_wheel_count = 0; //与妙算通信时计数，防止摩擦轮频繁开关
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
		case(SINGLE_SHOOT):                            //单点
		{																		//2020年单发子弹热量为固定值
				/*switch(shoot.shootlevel)
	      {
					case(LEVEL1):
						shoot.fric_wheel_spd = SINGLE_LEVEL1;  //单点一级
						break;
					case(LEVEL2):
						shoot.fric_wheel_spd = SINGLE_LEVEL2;  //单点二级
						break; 
					case(LEVEL3):
						shoot.fric_wheel_spd = SINGLE_LEVEL3;  //单点三级
						break;
					default:
						shoot.fric_wheel_spd = SINGLE_LEVEL1;  
						break;
				}*/
				shoot.fric_wheel_spd = SPEED_FIXED;	
		break;
		}
		case(TRIPLE_SHOOT):                            //三连发
		{
				/*switch(shoot.shootlevel)
	      {
					case(LEVEL1):
						shoot.fric_wheel_spd = TRIPLE_LEVEL1; //一级三连发
						break;
					case(LEVEL2):
						shoot.fric_wheel_spd = TRIPLE_LEVEL2; //二级三连发
						break;
					case(LEVEL3):
						shoot.fric_wheel_spd = TRIPLE_LEVEL3; //三级三连发
						break;
					default:
						shoot.fric_wheel_spd = TRIPLE_LEVEL1;
					  break;
				}*/
				shoot.fric_wheel_spd = SPEED_FIXED;
			break;
			}
		case(FULL_AUTOSHOOT):                            //全自动（指鼠标左键按住一直打）
		{
				/*switch(shoot.shootlevel)
	      {
					case(LEVEL1):
						 shoot.fric_wheel_spd = FULL_AUTO_LEVEL1;//一级全自动
						break;
					case(LEVEL2):
						 shoot.fric_wheel_spd = FULL_AUTO_LEVEL2;//二级全自动
						break;
					case(LEVEL3):
						 shoot.fric_wheel_spd = FULL_AUTO_LEVEL3;//三级全自动
						break;
					default:
						shoot.fric_wheel_spd = FULL_AUTO_LEVEL1;
					  break;
				}*/
			shoot.fric_wheel_spd = SPEED_FIXED;
		break;
		}
		case(AUTO_CTRL_SHOOT):                             //自动射击（pc控制）
		{
			if(pc_recv_mesg.shoot_contrl_data.shoot_mode!=0) //接收到pc控制发射机构指令
			{
				if(pc_recv_mesg.shoot_contrl_data.shoot_mode == SHOOT_OFF) //检测到关闭摩擦轮
				{
					if(fric_wheel_count++>=100)                              //如果计数达到100则关闭摩擦轮，放置摩擦轮急停
				  {
					  shoot.fric_wheel_run = 0;                              
				  }
		   	}
				else if(pc_recv_mesg.shoot_contrl_data.shoot_mode == SHOOT_FRIC_ON) //打开摩擦轮
				{
					shoot.fric_wheel_run = 1; //开摩擦轮
					shoot.shoot_cmd = 0;      //不单发
					shoot.c_shoot_cmd = 0;    //不连发
					shoot.fric_wheel_spd = SPEED_FIXED;
					if(pc_recv_mesg.shoot_contrl_data.shoot_speed == 0.0f)
						shoot.fric_wheel_spd = 0;
					fric_wheel_count = 0;
				}
				else if(pc_recv_mesg.shoot_contrl_data.shoot_mode == SHOOT_ON) //打蛋
				{
					shoot.fric_wheel_run = 1;  //开摩擦轮
					shoot.shoot_cmd = 1;       //开单发
					shoot.c_shoot_cmd = 0;     //不连发
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
				shoot.fric_wheel_spd = 0;  //关闭摩擦轮
				shoot.fric_wheel_run = 0;
			}
		}
	}
}


//卡弹检测及处理
void block_bullet_handler(void)
{
	static uint32_t stall_count5 = 0;
	static uint32_t stall_count1 = 0;
  static uint32_t stall_count2 = 0;
	static uint32_t stall_count3 = 0;
	static uint32_t stall_count4 = 0;     //对应四种模式不同的计数  
  static uint8_t  stall_f = 0;          //表示是否卡蛋
	static uint32_t stall_inv_count = 0;  //卡蛋后反转的计数
	
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
    
    if (stall_inv_count >= 20)  //反转时间//0.1s
    {
      stall_f = 0;
      stall_inv_count = 0;
    }
    else
      trig.spd_ref =2000*(-TIGGER_DIR);  //反转速度
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



//空仓检测
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
