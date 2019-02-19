/*********************************************************************************************************************
 * @company	   	根号三团队
 * @author     		Ouyang
 * @brief      		图片处理
 * @version    		v1.0
 * @Software 		IAR 8.2
 * @Target core		MK60DN512ZVLQ10
 * @date       		2019-01-15
 * @note		
 ********************************************************************************************************************/
#ifndef _IMAGE_MANIPULATION_H_
#define _IMAGE_MANIPULATION_H_

#include "headfile.h"
   
#define image_row 60               
#define image_col 94
   
extern uint8 flag_stop;                                       //停止舵机打角标志位
   
extern uint8 image_little[image_row][image_col]; 

extern uint8 add_line[image_row];                             //用于补线的数组

extern uint8 image_bin[image_row][image_col];                 //二值化图片 
extern char image_midline[image_row];                         //中线数组
extern uint8 edge_right[image_row],edge_left[image_row];      //定义右边界与左边界，用于寻找起始行中线

extern uint8 start_row ;                          //定义起始行   待定义
extern uint8 start_col;                           //定义起始中点 待定义
extern uint8 end_row;                             //定义终点行
extern uint8 end_col;                             //定义终点列


extern uint8 R_LostCount;                         //右边界丢失数
extern uint8 L_LostCount;                         //左边界丢失数
extern uint8 R_CorrectCount;                      //右边界正确数
extern uint8 L_CorrectCount;                      //左边界正确数

extern uint8 R_Lost_Line[2][2];                   //储存边缘丢线的数据,二维数组
extern uint8 L_Lost_Line[2][2];

extern uint8 DivLine;                             //有效行
extern uint8 StraightLine;                        //有效直道行数

extern uint8 Roundabout_L_flag;                   //环岛标志位
extern uint8 Roundabout_R_flag;

extern uint8 Roundabout_L_G_flag;                 //环岛标志位
extern uint8 Roundabout_L_O_flag;
extern uint8 Roundabout_R_G_flag;
extern uint8 Roundabout_R_O_flag;

extern signed short int OLS_X[image_row];       //用于最小二乘法
extern signed short int OLS_Y[image_row];       //用于最小二乘法

extern float OLS_K;                              //斜率
extern float OLS_R;                              //截距

extern float Straight_L_K;                       //直道左边界斜率
extern float Straight_R_K;                       //直道右边界斜率

extern float Costant_K;                          //预设前瞻
extern float Costant_R; 
 
extern float AngleError_Now;                     //图像差值

extern signed short int search_hang;            //用于找边界

extern uint8 state_flag;                           //用于debug
extern uint8 debug;               //用于debug


void get_img_bin(uint8 *p1,uint8 threshold);
void Get_Midline();
void image_reduce_size();
void Get_addline();
void add_line_init();
void Get_Straight_K();

#endif