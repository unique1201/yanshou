#include "stm32f10x.h"                  // Device header

void LED_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
}
void LED1_ON()
{
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
}
void LED1_OFF()
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
}
void LED2_ON()
{
	GPIO_SetBits(GPIOB,GPIO_Pin_13);
}
void LED2_OFF()
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_13);
}
void LED3_ON()
{
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
}
void LED3_OFF()
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_14);
}
void LED4_ON()
{
	GPIO_SetBits(GPIOB,GPIO_Pin_15);
}
void LED4_OFF()
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_15);
}