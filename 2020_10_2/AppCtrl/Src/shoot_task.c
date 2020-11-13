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
//¼ÇµÃ¿´Ò»ÏÂdir
int triple_shoot_count = 0;  //·ÀÖ¹ÈıÁ¬·¢Ä£Ê½ÏÂ×ªµÄÌ«¿ì£¬»¹Ã»Ğ´
int single_shoot_time = 0;   //·ÀÖ¹ÓëÃîËãÍ¨ĞÅÊ±´òµÄÌ«¿ì,ÏÂÃæÍ¬
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
			if((175-judge_rece_mesg.chassis_power_info.shooter_heat0)<zidan_count * HEAT_ONE_BULLET)		//µÈ¼¶Ò»ÈÈÁ¿ÉÏÏŞ£º180
			{
				shoot.c_shoot_cmd = 0;
			  shoot.shoot_cmd = 0;
			}
			shoot.trigger_motor_speed = 5000;
			break;
		case 2:
			if((235-judge_rece_mesg.chassis_power_info.shooter_heat0)<zidan_count * HEAT_ONE_BULLET)		//µÈ¼¶¶şÈÈÁ¿ÉÏÏŞ£º240
			{
				shoot.c_shoot_cmd = 0;
			  shoot.shoot_cmd = 0;
			}
			shoot.trigger_motor_speed = 5000;
			break;
		case 3:
			if((295-judge_rece_mesg.chassis_power_info.shooter_heat0)<zidan_count * HEAT_ONE_BULLET)		//µÈ¼¶ÈıÈÈÁ¿ÉÏÏŞ£º300
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
	if(((judge_rece_mesg.buff_data.power_rune_buff)&0x03) == 1)		//»úÆ÷ÈË¹¥»÷¼Ó³É£¬´ËÊ±ÉËº¦¼Ó±¶
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
				setfric_wheel_spd(); //2020Äê×Óµ¯ËÙ¶ÈÎªÒ»¹Ì¶¨Öµ //ÉèÖÃÄ¦²ÁÂÖ×ªËÙ  ¸ù¾İÉä»÷Ä£Ê½£¨µ¥µã£¬ÈıÁ¬·¢£¬È«×Ô¶¯£¬ÕâĞ©ÔÚÄ£Ê½ÇĞ»»Ê±¸Ä£© ÒÔ¼°²½±ø¼¶±ğ¸ø¶¨×ªËÙ
				shoot_heat_limit();
				fric_wheel_ctrl();   //Ä¦²ÁÂÖ¿ØÖÆ
					
				if (!shoot.fric_wheel_run)  // Èç¹ûÄ¦²ÁÂÖÃ»¿ª£¬²»´òµ°
				{
					shoot.shoot_cmd   = 0;    //²»µ¥·¢
					shoot.c_shoot_cmd = 0;    //²»Á¬·¢
				}
					
				if (shoot.fric_wheel_run)   //Ä¦²ÁÂÖÒÑ´ò¿ª£¬ÔÊĞí´òµ°
				{
					if (shoot.shoot_cmd==1)   //µ¥·¢
					{
						//trig.pos_ref = moto_trigger.total_encoder; //¸üĞÂ²¦µ¯µç»ú±àÂëÆ÷Öµ
						switch(shoot.firemode)
						{
							case(TRIPLE_SHOOT):                      //Èç¹ûÉæ¼°Ä£Ê½Ê±ÈıÁ¬·¢
								trig.pos_ref += 147456 * trig.dir;     //²¦µ¯µç»ú×ªµÄÎ»ÖÃ¼ÓÈı±¶µ¥·¢
								break;
							case(SINGLE_SHOOT):                      //Èç¹ûÊÇµ¥·¢
								trig.pos_ref += 49152 * trig.dir;      //¼Ó1/6*Ò»È¦
								break;
							case(FULL_AUTOSHOOT):                    //µ¥·¢Ìõ¼şÏÂ²»ÔÊĞíÈ«×Ô¶¯£¨¼´Ò»Ö±Á¬·¢£©
								shoot.shoot_cmd = 0;
								shoot.c_shoot_cmd = 1;
								break;
							case(AUTO_CTRL_SHOOT):                   //×Ô¶¯Éä»÷
							{
								if(pc_recv_mesg.shoot_contrl_data.shoot_mode!=0)    //½ÓÊÕµ½pcÖ¸Áî
								{ 
									single_shoot_time = HAL_GetTick();                //¼ÇÂ¼Ê±¼ä£¬¿ØÖÆÉäÆµ
									if(pc_recv_mesg.shoot_contrl_data.shoot_mode == SHOOT_ON)
									{									
										if(single_shoot_time-single_shoot_onetime>100) //100Îª¿ØÖÆÉäÆµ
										{
											trig.pos_ref += 49152 * trig.dir;            //µ¥·¢
											single_shoot_onetime = HAL_GetTick();        //¼ÇÂ¼Ê±¼ä£¬¿ØÖÆÉäÆµ
										}
									}
									pc_recv_mesg.shoot_contrl_data.shoot_mode = 0;    //´òÍêÒ»µ°Çå0·ÀÖ¹Ò»Ö±´ò
								}
								break;
							}
						}
						shoot.shoot_cmd = 2; //´òÍêÒ»µ°µ¥·¢±êÖ¾Î»¹é2£¬·ÀÖ¹Ò»Ö±´ò                         
					}
					
								
					if (shoot.c_shoot_cmd==1)                        //¿ªÆôÁ¬·¢Ä£Ê½,Ö»ÓÃËÙ¶È»·£¬²¦µ¯µç»úÒ»Ö±×ª
					{
						trig.pos_ref = moto_trigger.total_encoder-moto_trigger.total_encoder%49152; //ÍË³öÁ¬·¢Ê±²¦µ¯µç»úÔÚ×î¼Ñ²¦µ¯Î»ÖÃ£¬¼´½üËÆ³õÊ¼Î»ÖÃ
						trig.spd_ref = shoot.trigger_motor_speed * trig.dir;  //È«ËÙµÄÊ±ºò²¥µ¯ËÙ¶È£¬¿ÉÒÔ¸ù¾İµÈ¼¶ĞŞ¸Ä£¬¸Ğ¾õÒâÒå²»´ó
					}
					else
					{
						//²¦µ¯µç»úÎ»ÖÃ»·pid
						pid_calc(&pid_trigger, moto_trigger.total_encoder / 100, trig.pos_ref / 100);
						trig.spd_ref = pid_trigger.out;                                 //Èç¹û²»ÊÇÁ¬·¢Ä£Ê½£¬ËÙ¶ÈµÄÖ¸¶¨ÖµÉèÖÃÎªµ¥·¢Î»ÖÃ»·pidËã³öµÄÖµ
					}
					if(shoot.c_shoot_cmd!=1&&shoot.shoot_cmd!=1)
					{
						pid_calc(&pid_trigger, moto_trigger.total_encoder / 100, trig.pos_ref / 100);
						trig.spd_ref = pid_trigger.out; 
					}
					block_bullet_handler();                                           //¿¨µ¯´¦Àí
					empty_judge();
					pid_calc(&pid_trigger_spd, moto_trigger.speed_rpm, trig.spd_ref); //ËÙ¶È»·pidËãÊä³ö£¬¾ßÌåÊä³öÓë·¢ËÍÔÚgimbal_task
				}
				else
				{
					trig.spd_ref = 0;                                                 //Ä¦²ÁÂÖ²»¿ªÊ±²¦µ¯²»ÄÜ¶¯
					pid_calc(&pid_trigger_spd, moto_trigger.speed_rpm, trig.spd_ref); //ËÙ¶È»·pidËãÊä³ö£¬¾ßÌåÊä³öÓë·¢ËÍÔÚgimbal_task
					//pid_trigger_spd.out = 0;                            //Ä¦²ÁÂÖÃ»¿ª²»ÔÊĞí´òµ°Ê±²¦µ¯µç»ú·ÅËÉ
				}
	
			}	
		}
		shoot_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
}
//ĞèÒªÔÚconfig.hÖĞ¸Äpwm¶¨Ê±Æ÷µÄ¿Ú£¬ĞèÒªÔÚbsp_io.hÖĞ¸Ä¼¤¹âÄ¬ÈÏio¿
//ÏÖÄ¦²ÁÂÖ¸ÄÎªcan2Êä³ö
static void fric_wheel_ctrl(void)
{
  if (shoot.fric_wheel_run)
  {
			turn_on_friction_wheel(shoot.fric_wheel_spd);
      turn_on_laser(); //¸Ä¼¤¹â
  }
  else
  {
    turn_off_friction_wheel();
		
    turn_off_laser();
  }
}

void shoot_param_init(void)   //ĞèÒªÔÚÖ÷º¯ÊıÖĞ³õÊ¼»¯
{
  memset(&shoot, 0, sizeof(shoot_t));      //shootÇå0
  
  shoot.ctrl_mode = REMOTE_CTRL_SHOT;      //·¢Éä»ú¹¹¿ØÖÆÄ£Ê½ÎªÒ£¿ØÆ÷¿ØÖÆÄ£Ê½
	shoot.firemode = SINGLE_SHOOT;           //·¢ÉäÄ£Ê½Îªµ¥·¢
  shoot.fric_wheel_spd = SPEED_LEVEL_0;    	 //·¢ÉäËÙ¶ÈÎªÁã¼¶
  shoot.shootlevel = LEVEL0;               //²½±øµÈ¼¶ÎªÁã¼¶
	shoot.trigger_motor_speed = 1000;
	shoot.shoot_bullets = 0;
	shoot.remain_bullets = 0;
  
  trig.dir             = TIGGER_DIR; 
	//ÒÔÏÂÄ¿²â¶¼Ã»ÓÃ
  trig.feed_bullet_spd = 2000;
  trig.c_shoot_spd     = 4000;
  trig.one_sta         = TRIG_INIT;
}

void setfric_wheel_spd()           
{
	static int fric_wheel_count = 0; //ÓëÃîËãÍ¨ĞÅÊ±¼ÆÊı£¬·ÀÖ¹Ä¦²ÁÂÖÆµ·±¿ª¹Ø
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
		case(SINGLE_SHOOT):                            //µ¥µã
		{																		//2020Äêµ¥·¢×Óµ¯ÈÈÁ¿Îª¹Ì¶¨Öµ
				/*switch(shoot.shootlevel)
	      {
					case(LEVEL1):
						shoot.fric_wheel_spd = SINGLE_LEVEL1;  //µ¥µãÒ»¼¶
						break;
					case(LEVEL2):
						shoot.fric_wheel_spd = SINGLE_LEVEL2;  //µ¥µã¶ş¼¶
						break; 
					case(LEVEL3):
						shoot.fric_wheel_spd = SINGLE_LEVEL3;  //µ¥µãÈı¼¶
						break;
					default:
						shoot.fric_wheel_spd = SINGLE_LEVEL1;  
						break;
				}*/
				shoot.fric_wheel_spd = SPEED_FIXED;	
		break;
		}
		case(TRIPLE_SHOOT):                            //ÈıÁ¬·¢
		{
				/*switch(shoot.shootlevel)
	      {
					case(LEVEL1):
						shoot.fric_wheel_spd = TRIPLE_LEVEL1; //Ò»¼¶ÈıÁ¬·¢
						break;
					case(LEVEL2):
						shoot.fric_wheel_spd = TRIPLE_LEVEL2; //¶ş¼¶ÈıÁ¬·¢
						break;
					case(LEVEL3):
						shoot.fric_wheel_spd = TRIPLE_LEVEL3; //Èı¼¶ÈıÁ¬·¢
						break;
					default:
						shoot.fric_wheel_spd = TRIPLE_LEVEL1;
					  break;
				}*/
				shoot.fric_wheel_spd = SPEED_FIXED;
			break;
			}
		case(FULL_AUTOSHOOT):                            //È«×Ô¶¯£¨Ö¸Êó±ê×ó¼ü°´×¡Ò»Ö±´ò£©
		{
				/*switch(shoot.shootlevel)
	      {
					case(LEVEL1):
						 shoot.fric_wheel_spd = FULL_AUTO_LEVEL1;//Ò»¼¶È«×Ô¶¯
						break;
					case(LEVEL2):
						 shoot.fric_wheel_spd = FULL_AUTO_LEVEL2;//¶ş¼¶È«×Ô¶¯
						break;
					case(LEVEL3):
						 shoot.fric_wheel_spd = FULL_AUTO_LEVEL3;//Èı¼¶È«×Ô¶¯
						break;
					default:
						shoot.fric_wheel_spd = FULL_AUTO_LEVEL1;
					  break;
				}*/
			shoot.fric_wheel_spd = SPEED_FIXED;
		break;
		}
		case(AUTO_CTRL_SHOOT):                             //×Ô¶¯Éä»÷£¨pc¿ØÖÆ£©
		{
			if(pc_recv_mesg.shoot_contrl_data.shoot_mode!=0) //½ÓÊÕµ½pc¿ØÖÆ·¢Éä»ú¹¹Ö¸Áî
			{
				if(pc_recv_mesg.shoot_contrl_data.shoot_mode == SHOOT_OFF) //¼ì²âµ½¹Ø±ÕÄ¦²ÁÂÖ
				{
					if(fric_wheel_count++>=100)                              //Èç¹û¼ÆÊı´ïµ½100Ôò¹Ø±ÕÄ¦²ÁÂÖ£¬·ÅÖÃÄ¦²ÁÂÖ¼±Í£
				  {
					  shoot.fric_wheel_run = 0;                              
				  }
		   	}
				else if(pc_recv_mesg.shoot_contrl_data.shoot_mode == SHOOT_FRIC_ON) //´ò¿ªÄ¦²ÁÂÖ
				{
					shoot.fric_wheel_run = 1; //¿ªÄ¦²ÁÂÖ
					shoot.shoot_cmd = 0;      //²»µ¥·¢
					shoot.c_shoot_cmd = 0;    //²»Á¬·¢
					shoot.fric_wheel_spd = SPEED_FIXED;
					if(pc_recv_mesg.shoot_contrl_data.shoot_speed == 0.0f)
						shoot.fric_wheel_spd = 0;
					fric_wheel_count = 0;
				}
				else if(pc_recv_mesg.shoot_contrl_data.shoot_mode == SHOOT_ON) //´òµ°
				{
					shoot.fric_wheel_run = 1;  //¿ªÄ¦²ÁÂÖ
					shoot.shoot_cmd = 1;       //¿ªµ¥·¢
					shoot.c_shoot_cmd = 0;     //²»Á¬·¢
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
				shoot.fric_wheel_spd = 0;  //¹Ø±ÕÄ¦²ÁÂÖ
				shoot.fric_wheel_run = 0;
			}
		}
	}
}


//¿¨µ¯¼ì²â¼°´¦Àí
void block_bullet_handler(void)
{
	static uint32_t stall_count5 = 0;
	static uint32_t stall_count1 = 0;
  static uint32_t stall_count2 = 0;
	static uint32_t stall_count3 = 0;
	static uint32_t stall_count4 = 0;     //¶ÔÓ¦ËÄÖÖÄ£Ê½²»Í¬µÄ¼ÆÊı  
  static uint8_t  stall_f = 0;          //±íÊ¾ÊÇ·ñ¿¨µ°
	static uint32_t stall_inv_count = 0;  //¿¨µ°ºó·´×ªµÄ¼ÆÊı
	
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
    
    if (stall_inv_count >= 20)  //·´×ªÊ±¼ä//0.1s
    {
      stall_f = 0;
      stall_inv_count = 0;
    }
    else
      trig.spd_ref =2000*(-TIGGER_DIR);  //·´×ªËÙ¶È
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



//¿Õ²Ö¼ì²â
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
