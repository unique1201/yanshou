#include "stm32f10x.h"
#include "OLED.h"
#include "Delay.h"
#include "LED.h"
#include <stdio.h>
#include <string.h>

int16_t Num;
int Encoder_Count;
int16_t p = 0, k = 1, o = 0;
int16_t mode = 0;
float kp = 1, ki = 1, kd = 1;

// 按钮状态标志
volatile uint8_t button_up_flag = 0;
volatile uint8_t button_down_flag = 0;
volatile uint8_t button_ok_flag = 0;
volatile uint8_t button_back_flag = 0;
volatile uint32_t last_exti2_time = 0;
volatile uint32_t last_exti4_time = 0;
volatile uint32_t last_exti14_time = 0;

// LED控制相关变量
uint8_t led_speed_level = 0;  // 0,1,2 分别对应500ms,1000ms,200ms
uint8_t led_direction = 0;    // 0:正向, 1:反向
uint16_t led_delay_times[3] = {500, 1000, 200}; // 延时时间数组

// 长按相关变量
volatile uint8_t button_up_pressed = 0;
volatile uint8_t button_down_pressed = 0;
volatile uint32_t button_up_press_time = 0;
volatile uint32_t button_down_press_time = 0;
uint8_t pid_edit_mode = 0;  // PID编辑模式标志


// OK按钮防抖相关变量
uint32_t ok_button_last_time = 0;
uint8_t ok_button_state = 1; // 1:释放, 0:按下
uint8_t ok_button_last_state = 1;

void menu_Init1(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 启用GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_2 | GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void menu_Init2(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 启用GPIOC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void Button_Init(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 启用AFIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    // 配置按钮中断线
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);  // PA4 -> 上按钮
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);  // PA2 -> 下按钮  
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);  // PA0 -> OK按钮
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14); // PC14 -> 返回按钮
    
    // 配置EXTI4 (PA4 - 上按钮)
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // 配置EXTI2 (PA2 - 下按钮)
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_Init(&EXTI_InitStructure);
    
    // 配置EXTI0 (PA0 - OK按钮)
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_Init(&EXTI_InitStructure);
    
    // 配置EXTI14 (PC14 - 返回按钮)
    EXTI_InitStructure.EXTI_Line = EXTI_Line14;
    EXTI_Init(&EXTI_InitStructure);
    
    // 配置NVIC
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    // EXTI4 (PA4)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // EXTI2 (PA2)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);
    
    // EXTI0 (PA0)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_Init(&NVIC_InitStructure);
    
    // EXTI15_10 (PC14)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void Encoder_Button_Init(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 启用AFIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    // 配置编码器中断线 - 使用PB0和PB1
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0); // PB0 -> 编码器A相
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1); // PB1 -> 编码器B相
    
    // 配置EXTI0 (PB0 - 编码器A相)
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // 配置EXTI1 (PB1 - 编码器B相)
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_Init(&EXTI_InitStructure);
    
    // 配置NVIC - PB0和PB1使用EXTI0和EXTI1中断
    // 注意：EXTI0已经在Button_Init中配置过，这里不需要重复配置
    
    // EXTI1 (PB1)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 启用GPIOB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // 配置PB0和PB1为上拉输入
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 初始化编码器计数
    Encoder_Count = 0;
}

// 定时器初始化 - 用于长按检测
void Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 启用TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // 定时器配置：1ms中断
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;  // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // 预分频器，72MHz/72 = 1MHz，1MHz/1000 = 1kHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    // 启用TIM2更新中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    
    // 配置TIM2中断
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 启动定时器
    TIM_Cmd(TIM2, ENABLE);
}

// 统一的PID参数显示函数
void Update_PID_Display(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "PID");
    
    // 显示编辑模式标志
    if (pid_edit_mode) {
        OLED_ShowString(1, 12, "EDIT");
    }
    
    // 显示kp
    OLED_ShowString(2, 1, k == 1 ? ">kp:" : "kp :");
    OLED_ShowNum(2, 6, (int16_t)kp, 2);              // 整数部分
    OLED_ShowChar(2, 8, '.');
    OLED_ShowNum(2, 9, (int16_t)(kp * 10) % 10, 1);  // 小数部分
    
    // 显示ki
    OLED_ShowString(3, 1, k == 2 ? ">ki:" : "ki :");
    OLED_ShowNum(3, 6, (int16_t)ki, 2);
    OLED_ShowChar(3, 8, '.');
    OLED_ShowNum(3, 9, (int16_t)(ki * 10) % 10, 1);
    
    // 显示kd
    OLED_ShowString(4, 1, k == 3 ? ">kd:" : "kd :");
    OLED_ShowNum(4, 6, (int16_t)kd, 2);
    OLED_ShowChar(4, 8, '.');
    OLED_ShowNum(4, 9, (int16_t)(kd * 10) % 10, 1);
}

// LED控制显示函数
void Update_LED_Display(void)
{
    OLED_Clear();
    
    // 显示标题，如果处于编辑模式则显示"E"
    if (mode == 5) {
        OLED_ShowString(1, 1, "LED Control E");
    } else {
        OLED_ShowString(1, 1, "LED Control");
    }
    
    // 显示LED_speed
    OLED_ShowString(2, 1, o == 0 ? ">LED_speed" : "LED_speed");
    OLED_ShowNum(2, 12, led_speed_level, 1);
    
    // 显示LED_dir
    OLED_ShowString(3, 1, o == 1 ? ">LED_dir" : "LED_dir");
    OLED_ShowNum(3, 12, led_direction, 1);
}


static uint32_t system_time = 0;

// 流水灯函数

void LED_Running_Light(void)
{
    static uint32_t last_led_time = 0;
    static uint8_t current_led = 0;
    
    // 检查是否到达延时时间
    if (system_time - last_led_time < led_delay_times[led_speed_level]) {
        return;
    }
    last_led_time = system_time;
	
    
    // 关闭所有LED
    LED1_OFF();
    LED2_OFF();
    LED3_OFF();
    LED4_OFF();
    
    // 根据方向点亮对应的LED
    if (led_direction == 0) {
        // 正向流动
        switch (current_led) {
            case 0: LED1_ON(); break;
            case 1: LED2_ON(); break;
            case 2: LED3_ON(); break;
            case 3: LED4_ON(); break;
        }
        current_led = (current_led + 1) % 4;
    } else {
        // 反向流动
        switch (current_led) {
            case 0: LED4_ON(); break;
            case 1: LED3_ON(); break;
            case 2: LED2_ON(); break;
            case 3: LED1_ON(); break;
        }
        current_led = (current_led + 1) % 4;
    }
}

// PID参数调整函数
void Adjust_PID_Parameter(float increment)
{
    if (k == 1) {
        kp += increment;
        if (kp < 0) kp = 0;
        if (kp > 99.9f) kp = 99.9f;
    }
    else if (k == 2) {
        ki += increment;
        if (ki < 0) ki = 0;
        if (ki > 99.9f) ki = 99.9f;
    }
    else if (k == 3) {
        kd += increment;
        if (kd < 0) kd = 0;
        if (kd > 99.9f) kd = 99.9f;
    }
    Update_PID_Display();
}

// 长按连续调参处理
void Handle_Long_Press_Adjust(void)
{
    static uint32_t last_adjust_time = 0;
    
    if (pid_edit_mode && mode == 1) {
        // 检查上键长按
        if (button_up_pressed && (system_time - button_up_press_time > 1000)) {
            if (system_time - last_adjust_time >= 100) { // 每100ms调整一次
                Adjust_PID_Parameter(0.1f);
                last_adjust_time = system_time;
            }
        }
        
        // 检查下键长按
        if (button_down_pressed && (system_time - button_down_press_time > 1000)) {
            if (system_time - last_adjust_time >= 100) { // 每100ms调整一次
                Adjust_PID_Parameter(-0.1f);
                last_adjust_time = system_time;
            }
        }
    }
}

// 改进的OK按钮检测函数
void Check_OK_Button(void)
{
    static uint8_t debounce_count = 0;
    uint8_t current_state = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
    
    // 状态变化检测
    if (current_state != ok_button_last_state) {
        debounce_count = 0;
        ok_button_last_state = current_state;
        return;
    }
    
    // 防抖计数
    if (debounce_count < 10) { // 10ms防抖
        debounce_count++;
        return;
    }
    
    // 防抖完成，检测状态变化
    if (current_state != ok_button_state) {
        ok_button_state = current_state;
        
        // 检测下降沿（按钮按下）
        if (ok_button_state == 0) {
            // 设置OK按钮标志，在主循环中处理
            button_ok_flag = 1;
        }
    }
}

int oppo=0;
// 注意：PA0和PB0共享EXTI0中断线，我们需要在中断处理中区分它们
void EXTI0_IRQHandler(void)
{
    // 检查是否是PB0编码器触发的中断
    if (EXTI_GetITStatus(EXTI_Line0) == SET)
    {
        if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) == 0 && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == 1 && pid_edit_mode)
		{
			Encoder_Count--;
			// 计算增量（每次变化0.1）
        float increment = Encoder_Count * 0.1f;
        
        // 根据当前选择的参数更新
		if (mode == 1)
		{
        if (k == 1)
        {
            kp += increment;
            if (kp < 0) kp = 0;
            if (kp > 99.9f) kp = 99.9f;
        }
        else if (k == 2)
        {
            ki += increment;
            if (ki < 0) ki = 0;
            if (ki > 99.9f) ki = 99.9f;
        }
        else if (k == 3)
        {
            kd += increment;
            if (kd < 0) kd = 0;
            if (kd > 99.9f) kd = 99.9f;
        }
		
        
        // 清零编码器计数
        Encoder_Count = 0;
		
        
        Update_PID_Display();
		}
		}
        
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

// PB1编码器B相中断处理
void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1) == SET)
    {
        if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == 0 && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) == 1 && pid_edit_mode)
		{
			Encoder_Count++;
			// 计算增量（每次变化0.1）
        float increment = Encoder_Count * 0.1f;
        
        // 根据当前选择的参数更新
		if (mode == 1)
		{
        if (k == 1)
        {
            kp += increment;
            if (kp < 0) kp = 0;
            if (kp > 99.9f) kp = 99.9f;
        }
        else if (k == 2)
        {
            ki += increment;
            if (ki < 0) ki = 0;
            if (ki > 99.9f) ki = 99.9f;
        }
        else if (k == 3)
        {
            kd += increment;
            if (kd < 0) kd = 0;
            if (kd > 99.9f) kd = 99.9f;
        }
		
        // 清零编码器计数
        Encoder_Count = 0;
		
        
        Update_PID_Display();
		}
		}
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

// PA2 - 下按钮
void EXTI2_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line2) == SET)
    {
        button_down_flag = 1;
        button_down_pressed = 1;
        button_down_press_time = system_time;
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

// PA4 - 上按钮
void EXTI4_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line4) == SET)
    {
        button_up_flag = 1;
        button_up_pressed = 1;
        button_up_press_time = system_time;
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

// PC14 - 返回按钮（在EXTI15_10中断中）
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line14) == SET)
    {
        button_back_flag = 1;
        EXTI_ClearITPendingBit(EXTI_Line14);
    }
}

// 定时器2中断处理函数 - 用于系统计时和长按检测
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        system_time++;
        
        // 检查按钮释放
        if (button_up_pressed && GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == 1) {
            button_up_pressed = 0;
        }
        if (button_down_pressed && GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == 1) {
            button_down_pressed = 0;
        }
        
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

// 按钮处理函数
void Button_Process(void)
{
    static uint32_t last_button_time = 0;
    
    // 防抖处理
    if (system_time - last_button_time < 100) { // 20ms防抖
        return;
    }
    last_button_time = system_time;
    
    if (button_up_flag) {
        button_up_flag = 0;
        
        if (mode == 0) {
            // 主菜单模式 - 向上选择
            p = (p + 3) % 4;
            OLED_Clear();
            OLED_ShowString(1, 1, p == 0 ? ">LED Control" : "LED Control");
            OLED_ShowString(2, 1, p == 1 ? ">PID" : "PID");
            OLED_ShowString(3, 1, p == 2 ? ">Image" : "Image");
            OLED_ShowString(4, 1, p == 3 ? ">Angle" : "Angle");
        }
        else if (mode == 1) {
            if (pid_edit_mode) {
                // PID编辑模式 - 短按步进调整
				EXTI_InitTypeDef EXTI_InitStructure;
				EXTI_InitStructure.EXTI_Line = EXTI_Line4;
				EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
				EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
				EXTI_InitStructure.EXTI_LineCmd = ENABLE;
				EXTI_Init(&EXTI_InitStructure);
                Adjust_PID_Parameter(0.1f);
            } else {
                // PID参数设置模式 - 向上选择
                k = (k + 1) % 3 + 1;
                Update_PID_Display();
            }
        }
        else if (mode == 4) {
            // LED控制模式 - 向上选择
            o = (o + 1) % 2;
            Update_LED_Display();
        }
        else if (mode == 5) {
            // LED编辑模式 - 调节参数
            if (o == 0) {
                // 调节LED_speed：三档循环 0->1->2->0
                led_speed_level = (led_speed_level + 1) % 3;
                Update_LED_Display();
            }
            else if (o == 1) {
                // 调节LED_dir：切换方向 0<->1
                led_direction = 1 - led_direction;
                Update_LED_Display();
            }
        }
    }
    
    if (button_down_flag) {
        button_down_flag = 0;
		Delay_ms(10);
        
        if (mode == 0) {
            // 主菜单模式 - 向下选择
            p = (p + 1) % 4;
            OLED_Clear();
            OLED_ShowString(1, 1, p == 0 ? ">LED Control" : "LED Control");
            OLED_ShowString(2, 1, p == 1 ? ">PID" : "PID");
            OLED_ShowString(3, 1, p == 2 ? ">Image" : "Image");
            OLED_ShowString(4, 1, p == 3 ? ">Angle" : "Angle");
        }
        else if (mode == 1) {
            if (pid_edit_mode) {
                // PID编辑模式 - 短按步进调整
				EXTI_InitTypeDef EXTI_InitStructure;
				EXTI_InitStructure.EXTI_Line = EXTI_Line4;
				EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
				EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
				EXTI_InitStructure.EXTI_LineCmd = ENABLE;
				EXTI_Init(&EXTI_InitStructure);
                Adjust_PID_Parameter(-0.1f);
            } else {
                // PID参数设置模式 - 向下选择
                k = k % 3 + 1;
                Update_PID_Display();
            }
        }
        else if (mode == 4) {
            // LED控制模式 - 向下选择
            o = (o + 1) % 2;
            Update_LED_Display();
        }
        else if (mode == 5) {
            // LED编辑模式 - 调节参数
            if (o == 0) {
                // 调节LED_speed：三档循环 2->1->0->2
                led_speed_level = (led_speed_level + 2) % 3;
                Update_LED_Display();
            }
            else if (o == 1) {
                // 调节LED_dir：切换方向 0<->1
                led_direction = 1 - led_direction;
                Update_LED_Display();
            }
        }
    }
    
    if (button_ok_flag) {
        button_ok_flag = 0;
        
        if (mode == 0) {
            if (p == 0) {
                // LED Control模式
                mode = 4;
                o = 0;
                Update_LED_Display();
            }
            else if (p == 1) {
                mode = 1;
                k = 1;
                pid_edit_mode = 0; // 进入时默认非编辑模式
                Update_PID_Display();
            }
            else if (p == 2) {
                mode = 2;
                OLED_Clear();
                OLED_ShowString(1, 1, "Image");
                OLED_ShowString(2, 1, ">Image");
            }
            else if (p == 3) {
                mode = 3;
                OLED_Clear();
                OLED_ShowString(1, 1, "Angle");
                OLED_ShowString(2, 1, ">Angle");
            }
        }
        else if (mode == 1) {
            // PID模式 - 切换编辑模式
            pid_edit_mode = !pid_edit_mode;
            Update_PID_Display();
        }
        else if (mode == 4) {
            // 进入编辑模式
            mode = 5;
            Update_LED_Display();
        }
        else if (mode == 5) {
            // 退出编辑模式
            mode = 4;
            Update_LED_Display();
        }
    }
    
    if (button_back_flag) {
        button_back_flag = 0;
        
        // 返回上一级菜单
        if (mode != 0) {
            mode = 0;
            p = 0;
            pid_edit_mode = 0; // 退出时关闭编辑模式
            OLED_Clear();
            OLED_ShowString(1, 1, ">LED Control");
            OLED_ShowString(2, 1, "PID");
            OLED_ShowString(3, 1, "Image");
            OLED_ShowString(4, 1, "Angle");
        }
    }
}

// 主函数
int main(void)
{
    // 系统初始化
    OLED_Init();
    LED_Init();
    menu_Init1();
    menu_Init2();
    Button_Init();
    Encoder_Init();
    Encoder_Button_Init();
    Timer_Init();  // 初始化定时器
    
    // 显示初始菜单
    OLED_Clear();
    OLED_ShowString(1, 1, ">LED Control");
    OLED_ShowString(2, 1, "PID");
    OLED_ShowString(3, 1, "Image");
    OLED_ShowString(4, 1, "Angle");
    
    // 测试LED
    LED1_ON(); LED2_ON(); LED3_ON(); LED4_ON();
    Delay_ms(200);
    LED1_OFF(); LED2_OFF(); LED3_OFF(); LED4_OFF();
    Delay_ms(200);
    
    while (1)
    {
        // 处理按钮事件
        Button_Process();
        
        // 处理长按连续调参
        Handle_Long_Press_Adjust();
        
        // 运行流水灯效果
        LED_Running_Light();
        
        // 改进的OK按钮检测 - 在主循环中定期调用
        Check_OK_Button();
        
        // 短暂延时，避免过于频繁的检测
        Delay_ms(1); // 1ms延时，让系统有时间处理其他任务
    }
}
