#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "stm32f1xx.h"

//电机1的编码器输入引脚
#define MOTO1_ENCODER1_PORT GPIOA
#define MOTO1_ENCODER1_PIN  GPIO_PIN_0
#define MOTO1_ENCODER2_PORT GPIOA
#define MOTO1_ENCODER2_PIN  GPIO_PIN_1
#define MOTO2_ENCODER1_PORT GPIOA
#define MOTO2_ENCODER1_PIN  GPIO_PIN_8
#define MOTO2_ENCODER2_PORT GPIOA
#define MOTO2_ENCODER2_PIN  GPIO_PIN_9

//定时器号
#define ENCODER_2_TIM htim1
#define ENCODER_1_TIM htim2
#define PWM_TIM     htim3
#define GAP_TIM     htim4

#define MOTOR_SPEED_RERATIO 45u    //电机减速比
#define PULSE_PRE_ROUND 11 //一圈多少个脉冲
#define RADIUS_OF_TYRE 22 //轮胎半径，单位毫米
#define LINE_SPEED_C RADIUS_OF_TYRE * 2 * 3.14
#define RELOADVALUE_1 __HAL_TIM_GetAutoreload(&ENCODER_1_TIM)    //获取自动装载值,本例中为20000
#define COUNTERNUM_1 __HAL_TIM_GetCounter(&ENCODER_1_TIM)        //获取编码器定时器中的计数值
#define RELOADVALUE_2 __HAL_TIM_GetAutoreload(&ENCODER_1_TIM)    //获取自动装载值,本例中为20000
#define COUNTERNUM_2 __HAL_TIM_GetCounter(&ENCODER_1_TIM)        //获取编码器定时器中的计数值
//#define Target_Speed_1 300
//#define Target_Position 0
#define MOTOR1_TIM htim3
#define MOTOR2_TIM htim3
#define  MOTOR1_CHANNEL_FORWARD TIM_CHANNEL_2
#define  MOTOR1_CHANNEL_BACKWARD TIM_CHANNEL_1
#define  MOTOR2_CHANNEL_FORWARD TIM_CHANNEL_3
#define  MOTOR2_CHANNEL_BACKWARD TIM_CHANNEL_4
#define SPEED_RECORD_NUM 20 // 经测试，50Hz个采样值进行滤波的效果比较好

typedef struct _Motor
{
    int32_t lastCount;   //上一次计数值
    int32_t totalCount;  //总计数值
    int16_t overflowNum; //溢出次数
    float speed;         //电机转速
    uint8_t direct;      //旋转方向
}Motor;

extern Motor motor1,motor2;
extern float Target_Speed_1,Target_Speed_2;
extern float speed,position;

float Speed_Low_Filter(float new_Spe,float *speed_Record);
void Motor_Init(void);
void Motor_Control(TIM_HandleTypeDef *htim,float Target_Speed_1,float Target_Position);
void  motor1_run(float velocity);
void  motor2_run(float velocity);
void  motor_foward(float distance,float velocity);
void  motor_turnleft(float angle,float velocity);
void  motor_turnright(float angle,float velocity);
void fix_error(float error,float velocity);

#endif
