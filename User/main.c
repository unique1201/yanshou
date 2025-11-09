#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "Key.h"
#include "Encoder.h"
#include "Timer.h"
#include "Serial.h"

int32_t speed=0;
float actural1,target1,out1,actural2,target2,out2;
float kp=0.8,ki=0.1,kd=0.05;
float error01,error11,errorint1,error02,error12,errorint2,errorint3;
float out1,out2=0;
int8_t mode=0,keynum=1,state=0;
float angle1, angle2;           // 电机角度（编码器读数）
float target_angle1, target_angle2;
float angle_offset = 0;                  // 角度偏移量

// 编码器相关变量
int32_t encoder1_total = 0, encoder2_total = 0;
int32_t encoder1_last = 0, encoder2_last = 0;

#define ANGLE_SCALE 0.1f  // 角度缩放系数，可根据需要调整

uint8_t anjian(void)
{
    // 直接读取所有按键状态
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0) return 0;
    return 1;
}

void anjian1(void)
{
    static uint8_t count = 0;
    static uint8_t last_state = 0;
    static uint8_t current_state = 0;
    
    count++;
    if(count >= 2)  // 每20ms检测一次（10ms定时器×2）
    {
        count = 0;
        
        last_state = current_state;
        current_state = anjian();  // 读取当前所有按键状态
        
        // 按键释放时触发
        if(current_state == 1 && last_state == 0)
        {
            keynum = last_state;
        }
    }
}

uint8_t keyget(void)
{
	uint8_t t;
	t=keynum;
	keynum=1;
	return t;
}

void botton(void)
{
	if(keyget()==0)
	{
		if(mode==0)
		{
			mode=1;
			errorint3=0;
			target_angle1 = angle1;  // 保持当前位置
            target_angle2 = angle1;  // 电机2跟随电机1
		}
		else
		{
			mode=0;
			target_angle1 = angle1;
            target_angle2 = angle2;
		}
	}
}

void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		anjian1();
		state=1;
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

void setspeed(void)
{
	if(Serial_RxFlag == 1)
	{
		target1=data();
		target2=target1;
	}
}

// 获取编码器相对角度（处理溢出）
float GetEncoderAngle(int32_t current_count, int32_t* last_count, int32_t* total_count)
{
    int32_t delta;
    
    // 计算增量，处理溢出
    delta = current_count - *last_count;
    if(delta > 32767) delta -= 65536;    // 向下溢出
    else if(delta < -32768) delta += 65536; // 向上溢出
    
    // 更新累计值
    *total_count += delta;
    *last_count = current_count;
    
    // 返回相对角度（不是绝对角度）
    return *total_count * ANGLE_SCALE;
}

// 角度归一化到-180到180度范围（相对角度）
float NormalizeRelativeAngle(float angle)
{
    // 对于相对角度，我们保持连续值，不需要归一化到0-360
    // 这样可以避免在0/360边界处跳变
    return angle;
}

// 计算角度误差（相对角度）
float AngleError(float target, float actual)
{
    float error = target - actual;
    
    // 对于相对角度，不需要考虑圆周特性
    // 直接返回误差
    return error;
}

// 位置PID计算
void Position_PID_Calculate(void)
{
    // 电机1位置PID
    error11 = error01;
    error01 = AngleError(target_angle1, angle1);
    errorint1 += error01;
	// 积分限幅
    if(errorint1 > 1000) errorint1 = 1000;
    if(errorint1 < -1000) errorint1 = -1000;
	
	out1 = kp * error01 + ki * errorint1 + kd * (error01 - error11);
	
	if (error01<1)
	{
		out1=0;		
	}

	error12 = error02;
	target_angle2=angle1;
    error02 = AngleError(target_angle2, angle2);
    errorint2 += error02;
	
	// 积分限幅
    if(errorint2 > 1000) errorint2 = 1000;
    if(errorint2 < -1000) errorint2 = -1000;
    
    out2 = kp * error02 + ki * errorint2 + kd * (error02 - error12);
    
    // 输出限幅
    if(out1 > 100) out1 = 100;
    if(out1 < -100) out1 = -100;
    if(out2 > 100) out2 = 100;
    if(out2 < -100) out2 = -100;
	
}

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	Motor_Init();		//直流电机初始化
	Key_Init();			//按键初始化
	Encoder1_Init();
	Encoder2_Init();
	Timer_Init();
	Serial_Init();
	int time=0;
	
	
	while (1)
	{
		if (Serial_GetRxFlag()==1){
			speed=data();
			printf("%d\n",speed);
		}
		botton();
		setspeed();
		
		if (mode==0&&state==1)
		{
			OLED_ShowString(1,1,"mode1");
			state = 0;
			time++;
			if (time%200==0)errorint1=30;
			// 读取编码器角度
			angle1 = GetEncoderAngle(Encoder1_Get(), &encoder1_last, &encoder1_total);
			angle2 = GetEncoderAngle(Encoder2_Get(), &encoder2_last, &encoder2_total);
    
			
			Position_PID_Calculate();
			
			
			Motor_SetSpeed2(out1);
			
			
			Motor_SetSpeed1(0);
			printf("%f\n",out1);
    
			
		}else if(mode==1&&state==1)
		{
			OLED_ShowString(1,1,"mode2");
			Motor_SetSpeed2(0);
			target1=data();
			state=0;
			actural1=Encoder1_Get();
			error11=error01;
			error01=target1-actural1;
			errorint1+=error01;
			out1=kp*error01+ki*errorint1+kd*(error01-error11);
			if(out1>100)out1=100;
			if(out1<-100)out1=-100;
			Motor_SetSpeed1(out1);
			printf("%f\n",actural1);
			
		}
	}
}
