/*********************************************************************************************************************
 * @company	   	�������Ŷ�
 * @author     		whg
 * @brief      		ͼƬ����
 * @version    		v1.0
 * @Software 		IAR 8.2
 * @Target core		MK60DN512ZVLQ10
 * @date       		2019-01-19
 * @note		�Ե�вɼ������ݽ��д������ݼѼѵĳ�������޸�
 * @note                ��б�ţ�     0       1       2        
 * @note                ��Ӧλ�ã�    ���    �к�    �Һ�
 ********************************************************************************************************************/
#ifndef _DATA_PROCESS_H_
#define _DATA_PROCESS_H_

   
#include "headfile.h"
 
#define ADC_0_MAX AD_MAX[0]
#define ADC_1_MAX AD_MAX[1]
#define ADC_2_MAX AD_MAX[2]

#define ADC_0_MIN 0
#define ADC_1_MIN 10
#define ADC_2_MIN 0

extern uint16 AD_MAX[3];
extern uint16 ADC_ave[3];
extern uint16 ADC_one_and_inverse[3];

extern int16 Road_err;

extern uint8 island_flag;
extern uint8 island_direction;

float InvSqrt(float x);
void get_data(void);
void normalize(void);
void Road_Cal();

#endif
