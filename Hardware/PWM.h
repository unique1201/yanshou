#ifndef __PWM_H
#define __PWM_H

void PWM_Init(void);
void PWM_SetCompare1(uint16_t Compare);//电机1PWM
void PWM_SetCompare2(uint16_t Compare);//电机2PWM

#endif
