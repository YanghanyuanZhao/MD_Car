#include "encoder.h"
#include "tim.h"
#include "pid.h"

float speed_Record[SPEED_RECORD_NUM]={0};
float motor1_Out,motor2_Out;
float Now_Position;
float speed,position;
int ITjishu;
float Target_Speed_1,Target_Speed_2;


void Motor_Init(void)
{
    HAL_TIM_Encoder_Start(&ENCODER_1_TIM, TIM_CHANNEL_ALL);      //开启编码器定时器
    __HAL_TIM_ENABLE_IT(&ENCODER_1_TIM,TIM_IT_UPDATE);           //开启编码器定时器更新中断,防溢出处理
    HAL_TIM_Base_Start_IT(&GAP_TIM);                       //开启100ms定时器中断
    HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_2);            //开启PWM
    HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_1);            //开启PWM
	HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_3);            //开启PWM
    HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_4);            //开启PWM
    __HAL_TIM_SET_COUNTER(&ENCODER_1_TIM, 10000);                //编码器定时器初始值设定为10000
    motor1.lastCount = 0;                                   //结构体内容初始化
    motor1.totalCount = 0;
    motor1.overflowNum = 0;                                  
    motor1.speed = 0;
    motor1.direct = 0;
}

float Speed_Low_Filter(float new_Spe,float *speed_Record)
{
    float sum = 0.0f;
    //test_Speed = new_Spe;
    for(uint8_t i=SPEED_RECORD_NUM-1;i>0;i--)//将现有数据后移一位
    {
        speed_Record[i] = speed_Record[i-1];
        sum += speed_Record[i-1];
    }
    speed_Record[0] = new_Spe;//第一位是新的数据
    sum += new_Spe;
    //test_Speed = sum/SPEED_RECORD_NUM;
    return sum/SPEED_RECORD_NUM;//返回均值
}

void Motor_Get_Speed(TIM_HandleTypeDef *htim)
{
        /**********************************电机测速************************************/
        motor1.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&ENCODER_1_TIM);//如果向上计数（正转），返回值为0，否则返回值为1
        motor1.totalCount = COUNTERNUM_1 + motor1.overflowNum * RELOADVALUE_1;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        
        if(motor1.lastCount - motor1.totalCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motor1.overflowNum++;
            motor1.totalCount = COUNTERNUM_1 + motor1.overflowNum * RELOADVALUE_1;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        else if(motor1.totalCount - motor1.lastCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motor1.overflowNum--;
            motor1.totalCount = COUNTERNUM_1 + motor1.overflowNum * RELOADVALUE_1;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        
        motor1.speed = (float)(motor1.totalCount - motor1.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 3000;//算得每秒多少转,除以4是因为4倍频
        motor1.speed = Speed_Low_Filter(motor1.speed,speed_Record);
        motor1.lastCount = motor1.totalCount; //记录这一次的计数值
}

void Motor_Contorl(TIM_HandleTypeDef *htim,float Target_Speed_1,float Target_Position)
{
    Motor_Get_Speed(htim);//得到电机转速
    Now_Position = (float)(motor1.totalCount-10000);// 得到当前位置 10000编码器脉冲计数的初始值
    Target_Speed_1 = Location_PID_Realize(&pid_position,Target_Position,Now_Position);//位置环 Target_Position是目标位置，自行定义即可
    motor1_Out = Speed_PID_Realize(&pid_speed,Target_Speed_1,motor1.speed);//速度环
    if(motor1_Out >= 0)
    {
        __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD, 1000);
        __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD, 1000-motor1_Out);
    }
    else
    {
        __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD, 1000);
        __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD, 1000+motor1_Out);
    }
}
  
void  motor1_run(float velocity)
{
	if(velocity<0)
	{
		Target_Speed_1 = velocity;
	}
	else if(velocity>0)
	{
		Target_Speed_1 = velocity;
	}
	
	else if(velocity==0)
	{
		Target_Speed_1 = 0;
	}
}

void  motor2_run(/*int direction,*/float velocity)
{
	if(velocity>0)
	{
		Target_Speed_2 = velocity;
	}
	else if(velocity<0)
	{
		Target_Speed_2 = velocity;
	}
	else if(velocity==0)
	{
		Target_Speed_2 = 0;
	}
} 

void  motor_foward(float distance,float velocity)
{
	if(velocity>0)
	{
		motor1_run(velocity);
		motor2_run(velocity);
	}
	if(velocity<0)
	{
		motor1_run(velocity);
		motor2_run(velocity);
	}
	HAL_Delay(56*distance);
	motor1_run(0);
	motor2_run(0);
}

void  motor_turnleft(float angle,float velocity)
{
	motor1_run(0);
	motor2_run(1.2*velocity);
	HAL_Delay(20*angle);
	motor2_run(0);
}

void  motor_turnright(float angle,float velocity)
{
	motor2_run(0);
	motor1_run(1.2*velocity);
	HAL_Delay(20*angle);
	motor1_run(0);
}

void fix_error(float error,float velocity)
{
	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器回调函数，用于计算速度
{	
    //Motor_Control(&htim3,speed,position);
 if(htim->Instance==GAP_TIM.Instance)//间隔定时器中断，是时候计算速度了
    {
        motor1.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&ENCODER_1_TIM);//如果向上计数（正转），返回值为0，否则返回值为1
        motor1.totalCount = COUNTERNUM_1 + motor1.overflowNum * RELOADVALUE_1;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        
        if(motor1.lastCount - motor1.totalCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motor1.overflowNum++;
            motor1.totalCount = COUNTERNUM_1 + motor1.overflowNum * RELOADVALUE_1;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        else if(motor1.totalCount - motor1.lastCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motor1.overflowNum--;
            motor1.totalCount = COUNTERNUM_1 + motor1.overflowNum * RELOADVALUE_1;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        
        motor1.speed = (float)(motor1.totalCount - motor1.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 3000;//算得每秒多少转,除以4是因为4倍频
        /*******************在这里添加滤波函数************************/
        motor1.speed = Speed_Low_Filter(motor1.speed,speed_Record);
        /**********************************************************/
        motor1.lastCount = motor1.totalCount; //记录这一次的计数值
		/***************************PID速度环**********************************/
		motor1_Out = Speed_PID_Realize(&pid_speed,Target_Speed_1,motor1.speed);
        //Target_Speed_1是目标速度，自行定义就好
        if(motor1_Out >= 0)
    	{
        __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD, 10000);
        __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD, 10000-motor1_Out);
    	}
    	else
    	{
        __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD, 10000);
        __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD, 10000+motor1_Out);
    	}
		//printf("speed1 = %f\r\n",motor1.speed);
		///////////////////////////////////////////////////////////////////////////////////////////
		motor2.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&ENCODER_2_TIM);//如果向上计数（正转），返回值为0，否则返回值为1
        motor2.totalCount = COUNTERNUM_2 + motor2.overflowNum * RELOADVALUE_2;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        
        if(motor2.lastCount - motor2.totalCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motor2.overflowNum++;
            motor2.totalCount = COUNTERNUM_2 + motor2.overflowNum * RELOADVALUE_2;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        else if(motor2.totalCount - motor2.lastCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motor2.overflowNum--;
            motor2.totalCount = COUNTERNUM_2 + motor1.overflowNum * RELOADVALUE_2;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        
        motor2.speed = (float)(motor2.totalCount - motor2.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 3000;//算得每秒多少转,除以4是因为4倍频
        /*******************在这里添加滤波函数************************/
        motor2.speed = Speed_Low_Filter(motor2.speed,speed_Record);
        /**********************************************************/
        motor2.lastCount = motor2.totalCount; //记录这一次的计数值
		/***************************PID速度环**********************************/
		motor2_Out = Speed_PID_Realize(&pid_speed,Target_Speed_2,motor2.speed);
        //Target_Speed_2是目标速度，自行定义就好
        if(motor2_Out >= 0)
    	{
        __HAL_TIM_SetCompare(&MOTOR2_TIM, MOTOR2_CHANNEL_FORWARD, 10000);
        __HAL_TIM_SetCompare(&MOTOR2_TIM, MOTOR2_CHANNEL_BACKWARD, 10000-motor2_Out);
    	}
    	else
    	{
        __HAL_TIM_SetCompare(&MOTOR2_TIM, MOTOR2_CHANNEL_BACKWARD, 10000);
        __HAL_TIM_SetCompare(&MOTOR2_TIM, MOTOR2_CHANNEL_FORWARD, 10000+motor2_Out);
    	}
	}
}
