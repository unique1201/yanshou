#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include <stdlib.h>
#include <string.h>

uint8_t KeyNum;
volatile uint32_t system_tick=0;

/*定义变量*/
float Target1=0, Actual1, Out1;	//目标值，实际值，输出值
float Target2=0, Actual2, Out2;	//目标值，实际值，输出值
float Kp1=0.5f, Ki1=0.1f, Kd1=0.02f;					//比例项，积分项，微分项的权重
float Kp2=0.5f, Ki2=0.1, Kd2=0.02;					//比例项，积分项，微分项的权重
float Error0_1, Error1_1, ErrorInt_1;		//本次误差，上次误差，误差积分
float Error0_2, Error1_2, ErrorInt_2;	//本次误差，上次误差，误差积分
uint32_t position1=0,position2=0;
uint8_t receiving_command=0;
int16_t delta1;
int16_t delta2;

typedef enum {
	TASK_SPEED__CONTROL=0,
	TASK_FOLLOW_CONTROL=1
}TASK_Mode_t;

TASK_Mode_t current_task=TASK_SPEED__CONTROL;


void SysTick_Init(void)
{
	SysTick_Config(SystemCoreClock/1000);    //1ms中断
}

void SysTick_Handler(void)
{
	system_tick++;
	Key_Tick();
}
		
int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	Key_Init();			//非阻塞式按键初始化
	Motor_Init();		//电机初始化
	Encoder_Init();		//编码器初始化
	Serial_Init();		//串口初始化，波特率9600
	Timer_Init();		//定时器初始化，定时中断时间1ms
	SysTick_Init();
	
	while (1)
	{
		KeyNum=Key_GetNum();
		if(KeyNum==1)
		{
			current_task=(current_task==TASK_SPEED__CONTROL)?TASK_FOLLOW_CONTROL:TASK_SPEED__CONTROL;
			Target1=0;
			Target2=0;
			ErrorInt_1=0;
			ErrorInt_2=0;
			position1=0;
			position2=0;
		}
		if(current_task==TASK_SPEED__CONTROL){
			Out2=0;
			Motor2_SetPWM(0);
			if(Serial_GetRxFlag()==1){
				if(strstr(Serial_RxPacket,"speed%")!=NULL){
					sscanf(Serial_RxPacket,"speed%%%f",&Target1);
					Serial_Printf("Set_Speed:%d\r\n",(int)Target1);
				}else{
					Serial_SendString("ERROR\r\n");
				}
			}
			OLED_ShowString(1,1,"Speed Control",OLED_8X16);
			Serial_Printf("%f,%f,%f\n",Target1,Actual1,Out1);
			OLED_Update();
		}
		if(current_task==TASK_FOLLOW_CONTROL){
			Out1=0;
			Motor1_SetPWM(0);
			OLED_ShowString(1,1,"Speed Follow",OLED_8X16);
			OLED_ShowNum(4,2,delta2,5,OLED_8X16);

			Serial_Printf("%f,%f,%f\n",Target2,Actual2,Out2);
			OLED_Update();
		}
	}
}

void TIM1_UP_IRQHandler(void)//10ms执行一次
{
	/*定义静态变量（默认初值为0，函数退出后保留值和存储空间）*/
	static uint16_t Count;		

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{	
		Key_Tick();
		Count++;
		if(Count>=10)
		{
		/*获取实际速度值*/
		/*Encoder_Get函数，可以获取两次读取编码器的计次值增量*/
		/*此值正比于速度，所以可以表示速度，但它的单位并不是速度的标准单位*/
		/*此处每隔40ms获取一次计次值增量，电机旋转一周的计次值增量约为408*/
		/*因此如果想转换为标准单位，比如转/秒*/
		/*则可将此句代码改成Actual = Encoder_Get() / 408.0 / 0.04;*/
			delta1 = Encoder1_Get();
			
			delta2 = Encoder2_Get();
			Actual1=delta1;
			Actual2=delta2;
			position1+=delta1;
			position2+=delta2;
			if(current_task==TASK_SPEED__CONTROL){
			Count=0;
			/*获取本次误差和上次误差*/
			Error1_1 = Error0_1;			//获取上次误差
			Error0_1 = Target1 - Actual1;	//获取本次误差，目标值减实际值，即为误差值
			
			/*误差积分（累加）*/
			/*如果Ki不为0，才进行误差积分，这样做的目的是便于调试*/
			/*因为在调试时，我们可能先把Ki设置为0，这时积分项无作用，误差消除不了，误差积分会积累到很大的值*/
			/*后续一旦Ki不为0，那么因为误差积分已经积累到很大的值了，这就导致积分项疯狂输出，不利于调试*/
			if (Ki1 != 0)				//如果Ki不为0
			{
				ErrorInt_1 += Error0_1;		//进行误差积分
			}
			else						//否则
			{
			ErrorInt_1 = 0;			//误差积分直接归0
			}
				/*PID计算*/
			/*使用位置式PID公式，计算得到输出值*/
			Out1 = Kp1 * Error0_1 + Ki1 * ErrorInt_1 + Kd1 * (Error0_1 - Error1_1);
			/*输出限幅*/
			if (Out1 > 100) {Out1 = 100;}		//限制输出值最大为100
			if (Out1 < -100) {Out1 = -100;}	//限制输出值最小为100
			Motor1_SetPWM(Out1);
			}else if(current_task==TASK_FOLLOW_CONTROL){
				Count=0;
				Error1_2=Error0_2;
				Error0_2=position1-position2;
					if (Ki1 != 0)				//如果Ki不为0
				{
					ErrorInt_2 += Error0_2;		//进行误差积分
				}
				else						//否则
				{
				ErrorInt_2 = 0;			//误差积分直接归0
				}
				Out2=Kp2 * Error0_2 + Ki2 * ErrorInt_2 + Kd2 * (Error0_2 - Error1_2);
				if (Out2 > 100) {Out2 = 100;}		//限制输出值最大为100
				if (Out2 < -100) {Out2 = -100;}	//限制输出值最小为100
				Motor2_SetPWM(Out2);			
			}
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		}
	}
}
