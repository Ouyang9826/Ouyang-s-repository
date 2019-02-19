/*********************************************************************************************************************
 * @company	   	�������Ŷ�
 * @author     		Ouyang
 * @brief      		ͼƬ����
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
   
extern uint8 flag_stop;                                       //ֹͣ�����Ǳ�־λ
   
extern uint8 image_little[image_row][image_col]; 

extern uint8 add_line[image_row];                             //���ڲ��ߵ�����

extern uint8 image_bin[image_row][image_col];                 //��ֵ��ͼƬ 
extern char image_midline[image_row];                         //��������
extern uint8 edge_right[image_row],edge_left[image_row];      //�����ұ߽�����߽磬����Ѱ����ʼ������

extern uint8 start_row ;                          //������ʼ��   ������
extern uint8 start_col;                           //������ʼ�е� ������
extern uint8 end_row;                             //�����յ���
extern uint8 end_col;                             //�����յ���


extern uint8 R_LostCount;                         //�ұ߽綪ʧ��
extern uint8 L_LostCount;                         //��߽綪ʧ��
extern uint8 R_CorrectCount;                      //�ұ߽���ȷ��
extern uint8 L_CorrectCount;                      //��߽���ȷ��

extern uint8 R_Lost_Line[2][2];                   //�����Ե���ߵ�����,��ά����
extern uint8 L_Lost_Line[2][2];

extern uint8 DivLine;                             //��Ч��
extern uint8 StraightLine;                        //��Чֱ������

extern uint8 Roundabout_L_flag;                   //������־λ
extern uint8 Roundabout_R_flag;

extern uint8 Roundabout_L_G_flag;                 //������־λ
extern uint8 Roundabout_L_O_flag;
extern uint8 Roundabout_R_G_flag;
extern uint8 Roundabout_R_O_flag;

extern signed short int OLS_X[image_row];       //������С���˷�
extern signed short int OLS_Y[image_row];       //������С���˷�

extern float OLS_K;                              //б��
extern float OLS_R;                              //�ؾ�

extern float Straight_L_K;                       //ֱ����߽�б��
extern float Straight_R_K;                       //ֱ���ұ߽�б��

extern float Costant_K;                          //Ԥ��ǰհ
extern float Costant_R; 
 
extern float AngleError_Now;                     //ͼ���ֵ

extern signed short int search_hang;            //�����ұ߽�

extern uint8 state_flag;                           //����debug
extern uint8 debug;               //����debug


void get_img_bin(uint8 *p1,uint8 threshold);
void Get_Midline();
void image_reduce_size();
void Get_addline();
void add_line_init();
void Get_Straight_K();

#endif