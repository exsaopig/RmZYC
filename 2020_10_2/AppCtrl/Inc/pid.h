#ifndef __PID_H
#define __PID_H

#include "stm32f4xx_hal.h"

enum
{
  LLAST = 0,
  LAST,
  NOW,
  POSITION_PID,//位置式
  DELTA_PID,   //增量式
};

typedef struct pid_t
{
  float p;
  float i;
  float d;

  float set;
  float get;
  float err[3];

  float pout;
  float iout;
  float dout;
  float out;

  float input_max_err;    //input max err;
  float output_deadband;  //output deadband; 
  
  uint32_t pid_mode;
  uint32_t max_out; //最后输出限幅
  uint32_t integral_limit; //积分输出限幅

  void (*f_param_init)(struct pid_t *pid, 
                       uint32_t      pid_mode,
                       uint32_t      max_output,
                       uint32_t      inte_limit,
                       float         p,
                       float         i,
                       float         d);
  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);
 
} pid_t;

void PID_struct_init(
    pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd);

float pid_calc(pid_t *pid, float fdb, float ref);

extern pid_t pid_pit;
extern pid_t pid_yaw;
extern pid_t pid_pit_spd;
extern pid_t pid_yaw_spd;
extern pid_t pid_spd[4];

extern pid_t pid_chassis_angle;
extern pid_t pid_trigger;
extern pid_t pid_trigger_spd;
extern pid_t pid_imu_tmp;
extern pid_t pid_current[4];
extern pid_t pid_leftfric_spd;
extern pid_t pid_rightfric_spd;
#endif


