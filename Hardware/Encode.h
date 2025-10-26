#ifndef __ENCODER_H
#define __ENCODER_H

void Encoder_Init();
int16_t Encoder_Get();
extern int16_t Num;
extern int16_t p;
void menu_Init1();
void menu_Init2();
void Button_Init(void);
void Button_Process(void);

#endif
