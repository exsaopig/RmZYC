#ifndef __SYS_H__
#define __SYS_H__

#include "stm32f4xx_hal.h"

//#define CALIBRATE    //是否写了flash校准云台

//以下是区别不同机器人的宏定义
//定义是哪一个机器人
//0：官方步兵
//1：步兵1
//4：步兵4
/* 底盘电机          |
                A---------B
                     |
						         |
			          C---------D
				     官方步兵为   1 0 2 3
				     袁老板步兵1为 0 1 2 3
*/
#define ROBOT 5


/* 任务运行周期 */
#define COMM_TASK_PERIOD 10
#define GIMBAL_PERIOD 4
#define CHASSIS_PERIOD 4
#define IMU_TASK_PERIOD 1
#define INFO_GET_PERIOD 5

#define JUDGE_UART_TX_SIGNAL   ( 1 << 0 )
#define JUDGE_UART_IDLE_SIGNAL ( 1 << 1 )
#define JUDGE_DMA_FULL_SIGNAL  ( 1 << 2 )

#define PC_UART_TX_SIGNAL      ( 1 << 3 )
#define PC_UART_IDLE_SIGNAL    ( 1 << 4 )
#define PC_DMA_FULL_SIGNAL     ( 1 << 5 )

#define GIMBAL_MOTOR_MSG_SEND  ( 1 << 6 )
#define CHASSIS_MOTOR_MSG_SEND ( 1 << 7 )

#define SHOT_TASK_EXE_SIGNAL   ( 1 << 8 )
#define INFO_GET_EXE_SIGNAL    ( 1 << 9 )


#if (ROBOT == 0)
/* 电机部分 */
	#define CHASSIS_EC60 //电机种类是否为EC60
  #define CHASSIS_GYRO //有无车载单轴陀螺仪
	#define MOTOR_A 1  //201
  #define MOTOR_B 0  //202
  #define MOTOR_C 2  //203
  #define MOTOR_D 3  //204
/* 云台部分 */ 
  #ifndef CALIBRATE    //没写校准云台，需手动设定云台在中间时编码器的值
	  #define YAW_OFFSET   2200  //测量方法：关掉定时器4里的输出，在主函数中打印moto_yaw.encoder,取最大值最小值，
    #define PITCH_OFFSET 3000  //测量方法同yaw
  #endif
	
	/* 云台软件限位 */
	#define PIT_ANGLE_MAX        15  
	#define PIT_ANGLE_MIN        -15
	#define YAW_ANGLE_MAX        35
	#define YAW_ANGLE_MIN        -35
	#define TIGGER_DIR   1     //拨弹电机正转拨弹方向
  #define PITCH_SPD_DIR -1   //pitch速度方向，与板子安装方向和云台安装方向有关
	#define YAW_MOTO_POSITIVE_DIR  1.0f //yaw转向
	
	/* 云台安装位置与底盘中心的距离，单位为mm，X为前后 */
  #define GIMBAL_X_OFFSET 150
  #define GIMBAL_Y_OFFSET 0
  
#elif (ROBOT == 1)
/* 电机部分 */
  #define MOTOR_A 0  //201
  #define MOTOR_B 1  //202
  #define MOTOR_C 2  //203
  #define MOTOR_D 3  //204
/* 云台部分 */ 
  #ifndef CALIBRATE
	  #define YAW_OFFSET   2275
    #define PITCH_OFFSET 6991
  #endif
	
	/* 云台软件限位 */ 
  #define PIT_ANGLE_MAX        25
  #define PIT_ANGLE_MIN        -10
  #define YAW_ANGLE_MAX        32
  #define YAW_ANGLE_MIN        -32
	#define TIGGER_DIR   -1
  #define PITCH_SPD_DIR -1
	#define PIT_MOTO_POSITIVE_DIR  1.0f
	#define YAW_MOTO_POSITIVE_DIR  -1.0f
	
	/* 云台安装位置与底盘中心的距离，单位为mm，X为前后 */
  #define GIMBAL_X_OFFSET 30
  #define GIMBAL_Y_OFFSET 0
	
#elif (ROBOT == 4)
/* 电机部分 */
	#define MOTOR_A 0  //201
  #define MOTOR_B 1  //202
  #define MOTOR_C 2  //203
  #define MOTOR_D 3  //204
/* 云台部分 */
  #ifndef CALIBRATE
	  #define YAW_OFFSET   6010
    #define PITCH_OFFSET 5710
    
  #endif
	/* 云台软件限位 */
	#define PIT_ANGLE_MAX        20
	#define PIT_ANGLE_MIN        -13
	#define YAW_ANGLE_MAX        35
	#define YAW_ANGLE_MIN        -35
	
	#define YAW_MOTO_POSITIVE_DIR  -1.0f
	#define TIGGER_DIR   -1
  #define PITCH_SPD_DIR -1
	#define PIT_MOTO_POSITIVE_DIR  1.0f

	/* 云台安装位置与底盘中心的距离，单位为mm，X为前后 */
  #define GIMBAL_X_OFFSET 30
  #define GIMBAL_Y_OFFSET 0
#elif (ROBOT == 5)
/* 电机部分 */
	#define MOTOR_A 3  //201
  #define MOTOR_B 2  //202
  #define MOTOR_C 1  //203
  #define MOTOR_D 0  //204
/* 云台部分 */
  #ifndef CALIBRATE
	  #define YAW_OFFSET   3460
    #define PITCH_OFFSET 6800
    
  #endif
	/* 云台软件限位 */
	#define PIT_ANGLE_MAX        25
	#define PIT_ANGLE_MIN        -35
	#define YAW_ANGLE_MAX        180000
	#define YAW_ANGLE_MIN        -180000
	
	#define YAW_MOTO_POSITIVE_DIR  1.0f
	#define TIGGER_DIR   1
  #define PITCH_SPD_DIR -1
	#define PIT_MOTO_POSITIVE_DIR  -1.0f

	/* 云台安装位置与底盘中心的距离，单位为mm，X为前后 */
  #define GIMBAL_X_OFFSET 0
  #define GIMBAL_Y_OFFSET 0
#endif

//以下是通用宏定义
 
#define PC_BUFF_LEN     50   //PC发的信息缓冲区长度
#define PC_BUFF_MAX     1024 //最大长度，用于清dma，建议不要改小

/********************** 遥控器控制设置 ***************************/
#define RC_RESOLUTION     660.0f //遥控器遥杆上下活动范围值

/******************************* 底盘设置 *********************************/
/* 遥控器遥杆控制底盘 */
/* 左右 (mm/s) */
#define CHASSIS_RC_MAX_SPEED_X  2000.0f //最大速度
#define CHASSIS_RC_MOVE_RATIO_X 1.0f    //遥杆控制左右方向
/* 前后 (mm/s) */
#define CHASSIS_RC_MAX_SPEED_Y  2000.0f //最大速度
#define CHASSIS_RC_MOVE_RATIO_Y 1.0f    //遥杆控制前后方向
/* 旋转 (deg/s) */
/* 仅在开环即不跟随云台条件下起作用 */
#define CHASSIS_RC_MAX_SPEED_R 200.0f   //旋转最大速度
#define CHASSIS_RC_MOVE_RATIO_R 1.0f    //旋转方向

/* 键鼠控制底盘 */
/* 左右 (mm/s) */
#define CHASSIS_KB_MAX_SPEED_X  2000.0f //最大速度 
#define CHASSIS_KB_MOVE_RATIO_X 1.0f    //键鼠控制左右方向
/* 前后 (mm/s) */
#define CHASSIS_KB_MAX_SPEED_Y  2000.0f //最大速度 
#define CHASSIS_KB_MOVE_RATIO_Y 1.0f    //键鼠控制前后方向

/************************** 云台设置 *******************************/
/* remote mode gimbal speed limit */
/* pitch axis speed */
#define GIMBAL_RC_MOVE_RATIO_PIT 1.0f
/* yaw axis speed */
#define GIMBAL_RC_MOVE_RATIO_YAW 1.0f

/* keyboard mode gimbal speed limit */
/* pitch axis speed */
#define GIMBAL_PC_MOVE_RATIO_PIT 1.0f
/* yaw axis speed */
#define GIMBAL_PC_MOVE_RATIO_YAW 1.0f

/**************************发射设置，射速射频********************************/
/* shoot speed
1300 7
1400 10
1500 17
1520 18
1550 20
1570 21
1600 23
1620 24
1650 25
1700 27
*/
/*#define FULL_AUTO_LEVEL1 1420 //maximum value is 2500
#define SINGLE_LEVEL1 5000 //maximum value is 2500
#define TRIPLE_LEVEL1 1450 //maximum value is 2500
#define FULL_AUTO_LEVEL2 1450 //maximum value is 2500
#define SINGLE_LEVEL2 1550 //maximum value is 2500
#define TRIPLE_LEVEL2 1450 //maximum value is 2500
#define FULL_AUTO_LEVEL3 1500 //maximum value is 2500
#define SINGLE_LEVEL3 1570 //maximum value is 2500
#define TRIPLE_LEVEL3 1480 //maximum value is 2500
*/
#define SPEED_LEVEL_0 3650  //15
#define SPEED_LEVEL_1 4500  //18 
#define SPEED_LEVEL_2 5300  //22
#define SPEED_LEVEL_3 7600  //30
/* shoot frequence */
//#define TRIGGER_MOTOR_SPEED      6500 //拨弹电机转速，控制射频

/************************ chassis parameter ****************************/
/* 轮子周长(mm) */
#define PERIMETER  478

/* wheel track distance(mm) */
#define WHEELTRACK 403    //不知道是啥，解算麦轮速度用了，别删，看着办吧
/* wheelbase distance(mm) */
#define WHEELBASE  385    //不知道是啥，解算麦轮速度用了，别删，看着办吧

/* 默认为3508，看是否宏定义了EC60 */

#ifdef CHASSIS_EC60   //EC60的只有官方车不加注释了
  /* chassis motor use EC60 */
  /* the deceleration ratio of chassis motor */
  #define CHASSIS_DECELE_RATIO (1.0f)
  /* single 3508 motor maximum speed, unit is rpm */
  #define MAX_WHEEL_RPM        400   //440rpm = 3500mm/s
  /* chassis maximum translation speed, unit is mm/s */
  #define MAX_CHASSIS_VX_SPEED 3300  //415rpm
  #define MAX_CHASSIS_VY_SPEED 3300
  /* chassis maximum rotation speed, unit is degree/s */
  #define MAX_CHASSIS_VR_SPEED 300
#else     //3508，官方，我不想写注释了，好麻烦
  /* chassis motor use 3508 */
  /* the deceleration ratio of chassis motor */
  #define CHASSIS_DECELE_RATIO (1.0f/19.0f)
  /* single 3508 motor maximum speed, unit is rpm */
  #define MAX_WHEEL_RPM        8500  //8347rpm = 3500mm/s
  /* chassis maximum translation speed, unit is mm/s */
  #define MAX_CHASSIS_VX_SPEED 3300  //8000rpm
  #define MAX_CHASSIS_VY_SPEED 3300
  /* chassis maximum rotation speed, unit is degree/s */
  #define MAX_CHASSIS_VR_SPEED 300   //5000rpm
#endif

/************************** gimbal parameter *****************************/
/* the ratio of motor encoder value translate to degree */
#define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)    //编码器折算到369度
/* the deceleration ratio of pitch axis motor */
#define PIT_DECELE_RATIO       1.0f
/* the deceleration ratio of yaw axis motor */
#define YAW_DECELE_RATIO       1.0f
/* the positive direction of pitch axis motor */

/* the positive direction of yaw axis motor */

/* the positive direction of tirgger motor */
#define TRI_MOTO_POSITIVE_DIR  1.0f




/***********************system interface setting****************************/

/* automatic navigation interface */
#define AUTO_NAVIGATION 

/* can relevant */
#define CHASSIS_CAN       hcan1
#define ZGYRO_CAN         hcan2
#define CHASSIS_ZGYRO_CAN hcan1
#define GIMBAL_CAN        hcan1
#define TRIGGER_CAN       hcan1
#define FRIC_CAN          hcan2
#define CAP_CAN           hcan1
/* uart relevant */
/**
  * @attention
  * close usart DMA receive interrupt, so need add 
  * uart_receive_handler() before HAL_UART_IROHandler() in uart interrupt function
*/
#define DBUS_HUART         huart1 //for dji remote controler reciever
#define DEBUG_HUART        huart6 //for debug
#define JUDGE_HUART        huart3 //connected to judge system
#define COMPUTER_HUART     huart2//connected to manifold/TXone

//#define LEFT_FRICTION        TIM1->CCR1
//#define RIGHT_FIRCTION       TIM1->CCR4
#define COVER_STEER			 TIM2->CCR1

#define DEFAULT_TUNE  300

/* imu temperature control */
#define IMU_PWM_PULSE      TIM3->CCR2
#define DEFAULT_IMU_TEMP   50

/* math relevant */

/* radian coefficient */
#define RADIAN_COEF 57.3f
/* circumference ratio */
#define PI          3.142f

#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

#define SDA_IN();  {GPIOA->MODER&=~(3<<(5*2));GPIOA->MODER|=0<<5*2;}
#define SDA_OUT(); {GPIOA->MODER&=~(3<<(5*2));GPIOA->MODER|=1<<5*2;}
#define IIC_SDA_UP() 	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define IIC_SDA_DOWN()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)
#define IIC_SCL_UP()     HAL_GPIO_WritePin(GPIOI, GPIO_PIN_9, GPIO_PIN_SET)
#define IIC_SCL_DOWN()   HAL_GPIO_WritePin(GPIOI, GPIO_PIN_9, GPIO_PIN_RESET)
#define IIC_SDA_READ()   HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)
#define RED_OFF     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET)
#define RED_ON     	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET)
#define GREEN_ON  	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET)
#define GREEN_OFF   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET)
#define RED_Toggle HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11)
#define GREEN_Toggle HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14)

#define PAGE_UP  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==1
#define ENTER    HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1   //1
#define DOWN     HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12)==1  //4
#define RETURN   HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==1   //3
#define UP       HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==1   //2

#endif
