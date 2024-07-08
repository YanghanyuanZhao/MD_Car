#include "pid.h"
#include "tim.h"
PID pid_speed,pid_position;

/**********************************
 * 功能：PID结构体参数初始化
 * 输入：无
 * 返回：无
 * *******************************/
void PID_Init(void)//PID参数初始化
{
    pid_speed.err = 0;
    pid_speed.integral = 0;
    pid_speed.maxIntegral = 1000;
    pid_speed.maxOutput = __HAL_TIM_GetAutoreload(&PWM_TIM);
    pid_speed.lastErr = 0;
    pid_speed.output = 0;
    pid_speed.kp = KP_speed;
    pid_speed.ki = KI_speed;
    pid_speed.kd = KD_speed;

    pid_position.err = 0;
    pid_position.integral = 0;
    pid_position.maxIntegral = 80;
    pid_position.maxOutput = __HAL_TIM_GetAutoreload(&PWM_TIM);
    pid_position.lastErr = 0;
    pid_position.output = 0;
    pid_position.kp = KP_position;//这几个宏定义要自己补充
    pid_position.ki = KI_position;
    pid_position.kd = KD_position;
}

/****************************************
 * 作用：速度环PID计算
 * 参数：PID参数结构体地址；目标值；反馈值
 * 返回值：无
 * ****************************************/
float Speed_PID_Realize(PID* pid,float target,float feedback)//一次PID计算
{
    pid->err = target - feedback;
    if(pid->err < 0.3 && pid->err > -0.3) pid->err = 0;//pid死区
    pid->integral += pid->err;
    
    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;

    if(target == 0) pid->integral = 0; // 刹车时清空i


    pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral) + (pid->kd * (pid->err - pid->lastErr));//全量式PID

    //输出限幅
    if(target >= 0)//正转时
    {
        if(pid->output < 0) pid->output = 0;
        else if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
    }
    else if(target < 0)//反转时
    {
        if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
        else if(pid->output > 0) pid->output = 0;
    }

    pid->lastErr = pid->err;
    if(target == 0) pid->output = 0; // 刹车时直接输出0
    return pid->output;
}

float Location_PID_Realize(PID* pid,float target,float feedback)//一次PID计算
{
     if(pid->err < 0.5 && pid->err > -0.5) pid->err = 0;//pid死区
    pid->err = target - feedback;
    pid->integral += pid->err;

    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;

    pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral) + (pid->kd * (pid->err - pid->lastErr));//全量式PID

    //输出限幅
    if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
    if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;

    pid->lastErr = pid->err;

    return pid->output;
}
