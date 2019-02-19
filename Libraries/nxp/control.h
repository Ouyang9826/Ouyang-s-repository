#ifndef _control_h_
#define _control_h_

#include "headfile.h"

extern uint8 S3010_Flag;
extern int16 S3010_duty;
extern int16 PID_dir;
extern int16 ideal_value_DC;

extern int16 ideal_speed_L;
extern int16 ideal_speed_R;
extern int16 ideal_speed;
extern int16 PID_Duty_L; 
extern int16 PID_Duty_R; 
extern uint8 Force_Stop_flag;

#define S3010_midduty                   850   //这3个是恒定的
#define S3010_minduty                   760
#define S3010_maxduty                   940

#define Motor_Max_Duty                  8000
#define Motor_Min_Duty                  -8000

#define Speed_Set                       20

#define K1 gpio_get(C5)                 //读取按键
#define K2 gpio_get(C4)
#define K3 gpio_get(C3)
#define K4 gpio_get(C2)
#define K5 gpio_get(C1)
#define K6 gpio_get(C0)

#define Switch1 gpio_get(E11)           //读取开关状态 0：ON   1：OFF
#define Switch2 gpio_get(E10)
#define Switch3 gpio_get(E9)
#define Switch4 gpio_get(E8)
#define Switch5 gpio_get(E7)
#define Switch6 gpio_get(E6)
#define Switch7 gpio_get(E5)
#define Switch8 gpio_get(E4)

void OLS(signed short int *x, signed short int *y, uint8 n);
void Encoder_Init(void);
void S3010_Init();
void S3010_Control(uint16 x);
void S3010_Control_CA();
void S3010_Control_DC(int16 true_value_DC);
void BEEP_Init();
void BEEP(uint8 x);
void Key_Init();
void Key_Control();
void Switch_Init();
void ADC_Init();
void ADC_Display();
void Motor_Init();
void Motor_Control_L(int16 x);
void Motor_Control_R(int16 x);
void Speed_Control_L(int16 true_value_L);
void Speed_Control_R(int16 true_value_R);
void Differ_Contorl();

void Key_test();
void adc_test();
void servo_test();
void motor_test(uint8 x);
void bianma_test();
void Motor_test(uint8 x);

void Force_Stop();


#endif