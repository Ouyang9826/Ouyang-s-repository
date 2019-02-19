/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		isr
 * @company	   		成都逐飞科技有限公司
 * @author     		Go For It(1325536866)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.17
 * @Target core		MK66FX
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-09-19
 ********************************************************************************************************************/



#ifndef _isr_h
#define _isr_h

#include "headfile.h"
void PORTA_IRQHandler(void);
void PORTB_IRQHandler(void);

void PIT0_IRQHandler(void);

void FTM1_IRQHandler(void);           //用来测速 
void FTM2_IRQHandler(void);


/******************电机测速用到的外部变量********************/
extern int16 cesu_left_count;//左侧电机计数
extern int16 cesu_right_count;//右侧电机计数
extern int16 Left_Speed_now;//左侧测速结果
extern int16 Right_Speed_now;//左侧测速结果
extern int16 Speed_now;
extern int16 Road_HD;

extern int8 BEEP_Count;//蜂鸣器响时间计数

#endif
