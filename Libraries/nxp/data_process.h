/*********************************************************************************************************************
 * @company	   	根号三团队
 * @author     		whg
 * @brief      		图片处理
 * @version    		v1.0
 * @Software 		IAR 8.2
 * @Target core		MK60DN512ZVLQ10
 * @date       		2019-01-19
 * @note		对电感采集的数据进行处理，根据佳佳的程序进行修改
 * @note                电感编号：     0       1       2        
 * @note                对应位置：    左横    中横    右横
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
