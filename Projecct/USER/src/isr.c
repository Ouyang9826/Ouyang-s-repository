/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2016,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		isr.c
 * @brief      		�жϺ�����
 * @company	   	�ɶ���ɿƼ����޹�˾
 * @author     		Go For It(1325536866)
 * @version    		v1.0
 * @Software 		IAR 7.7 or MDK 5.17
 * @Target core		MK60DN512VLL10
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2016-02-25
 ********************************************************************************************************************/



#include "isr.h"

int16 cesu_left_count=0;//���������
int16 cesu_right_count=0;//�Ҳ�������
int16 Left_Speed_now=0;//�����ٽ��
int16 Right_Speed_now=0;//�����ٽ��
int16 Speed_now=0;//���ٽ��

int16 Road_HD=0;

int8 PIT0_Count_10ms = 0;//pit0 10ms�жϼ���
int8 PIT0_Count_20ms = 0;//pit0 20ms�жϼ���
int8 BEEP_Count = 0;//��������ʱ�����

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PROTA�ж�ִ�к���
//  @return     void   
//  @since      v1.0
//  Sample usage:               ��A�������жϹ����ҷ����жϵ�ʱ����Զ�ִ�иú���
//-------------------------------------------------------------------------------------------------------------------
void PORTA_IRQHandler(void)
{
    //����жϱ�־��һ�ַ���ֱ�Ӳ����Ĵ�����ÿһλ��Ӧһ������
	PORTA->ISFR = 0xffffffff;
	//ʹ�����Ǳ�д�ĺ궨����������жϵ�����
	//PORTA_FLAG_CLR(A1);
}

void PORTC_IRQHandler(void)
{
    //����жϱ�־��һ�ַ���ֱ�Ӳ����Ĵ�����ÿһλ��Ӧһ������
	PORTC->ISFR = 0xffffffff;
	//ʹ�����Ǳ�д�ĺ궨����������жϵ�����
	//PORTC_FLAG_CLR(C1);
    VSYNC();
}

void DMA0_IRQHandler(void)
{
	DMA_IRQ_CLEAN(DMA_CH0);
        row_finished();
	
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      
//  @return     void   
//  @since      v1.0
//  Sample usage:                
//-------------------------------------------------------------------------------------------------------------------
//PIT0�жϷ�������������oled��Ļ����ʾadc�ɼ�ֵ
void PIT0_IRQHandler(void)
{
  PIT_FlAG_CLR(pit0);//����жϱ�־
    
  PIT0_Count_10ms++;
  //PIT0_Count_20ms++;
  
  if(PIT0_Count_10ms==2)
  {
    PIT0_Count_10ms=0;
    
    get_data();//������ɼ�
    normalize();//��һ�����ֵ����
    Road_Cal();
    
    //Force_Stop();//����ͣ��
  }
  
  //if(PIT0_Count_20ms==10)
  
  if(BEEP_Count !=0)
  {
    BEEP_Count--;
  }
  else
  {
    gpio_set(E26,0);
  }

  S3010_Control_DC(Road_err);//�������
  //�ٶȼ���
  if(gpio_get(A8))//���ֲ���
    Left_Speed_now = cesu_left_count;
  else
    Left_Speed_now = -cesu_left_count;
  if(gpio_get(A17))//���ֲ���
    Right_Speed_now = -cesu_right_count;
  else
    Right_Speed_now = cesu_right_count;
  
  Speed_now = (Left_Speed_now + Right_Speed_now)/2;
  
  if(Road_HD != 0)
  {
    Road_HD -= Speed_now;
  }
  
  cesu_left_count = 0;//�������
  cesu_right_count = 0;//�������
  
  //Differ_Contorl();//���ٴ���
  
//  Speed_Control_L(Left_Speed_now);
//  Speed_Control_R(Right_Speed_now);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      
//  @return     void   
//  @since      v1.0
//  Sample usage:                
//-------------------------------------------------------------------------------------------------------------------
//FTM1���벶׽�жϷ����������ڸ���������
void FTM1_IRQHandler(void)
{
  uint8 s = FTM1_INPUT_FLAG_READ;        //��ȡ��׽�ͱȽ�״̬  All CHnF bits can be checked using only one read of STATUS.
  uint8 CHn;
  
  FTM1_INPUT_FLAG_READ = 0x00;           //���жϱ�־λ
  
  CHn = 0;                      //CH0ͨ������ֵΪ0����CH1ͨ����ֵΪ1
  if( s & (1 << CHn) )
  {
    //FTM_IRQ_DIS(FTM1, CHn);   //��ֹ���벶׽�жϣ�����ֹҲûӰ��
    /*     �û�����       */
//    if( gpio_get(A8 ) )//Left_Direction_Pin))         //������ת���������ʱ��ת����װ��ʽ������������͵�ƽ
//    { cesu_left_count++;        //���ַ�תcount--
//    }
//    else
//    { cesu_left_count--;        //������תcount++
//
//    }
    cesu_left_count++;
  }
}
void FTM2_IRQHandler(void)            //�����ҵ������
{
  uint8 s = FTM2_INPUT_FLAG_READ;        //��ȡ��׽�ͱȽ�״̬  All CHnF bits can be checked using only one read of STATUS.
  uint8 CHn;
  
  FTM2_INPUT_FLAG_READ = 0x00;           //���жϱ�־λ
  
  CHn = 0;                      //CH0ͨ������ֵΪ0����CH1ͨ����ֵΪ1
  if( s & (1 << CHn) )
  {
    //FTM_IRQ_DIS(FTM2, CHn);   //��ֹ���벶׽�жϣ�����ֹҲûӰ��
    /*     �û�����       */
//    if( gpio_get(A17) )//ת���������෴     FTM2_CH1_PIN   //������ת���������ʱ��ת����װ��ʽ������������͵�ƽ
//    {
//     cesu_right_count--;        //���ַ�תcount--
//    }
//    else
//    { 
//      cesu_right_count++;        //������תcount++
//    }
    cesu_right_count++;
  }
}




//-------------------------------------------------------------------------------------------------------------------
//  @brief      UART3�ж�ִ�к���
//  @return     void   
//  @since      v1.0
//  Sample usage:               ��UART3�����жϹ����ҷ����жϵ�ʱ����Զ�ִ�иú���
//-------------------------------------------------------------------------------------------------------------------
void UART3_RX_TX_IRQHandler(void)
{
    if(UART3->S1 & UART_S1_RDRF_MASK)                                     //�������ݼĴ�����
    {
        //�û���Ҫ�����������
        mt9v032_cof_uart_interrupt();
    }
    if(UART3->S1 & UART_S1_TDRE_MASK )                                    //�������ݼĴ�����
    {
        //�û���Ҫ����������

    }
}

void UART4_RX_TX_IRQHandler(void)
{
    if(UART4->S1 & UART_S1_RDRF_MASK)                                     //�������ݼĴ�����
    {
        
    }
    if(UART4->S1 & UART_S1_TDRE_MASK )                                    //�������ݼĴ�����
    {
        //�û���Ҫ����������

    }
}


/*
�жϺ������ƣ��������ö�Ӧ���ܵ��жϺ���
Sample usage:��ǰ������DMA0�жϣ�Ȼ��͵�����ȥ���ĸ���DMA0���жϺ������ƣ��ҵ���дһ�������Ƶĺ�������
void DMA0_IRQHandler(void)
{
    ;
}
�ǵý����жϺ������־λ


DMA0_IRQHandler  
DMA1_IRQHandler  
DMA2_IRQHandler  
DMA3_IRQHandler  
DMA4_IRQHandler  
DMA5_IRQHandler  
DMA6_IRQHandler  
DMA7_IRQHandler  
DMA8_IRQHandler  
DMA9_IRQHandler  
DMA10_IRQHandler 
DMA11_IRQHandler 
DMA12_IRQHandler 
DMA13_IRQHandler 
DMA14_IRQHandler 
DMA15_IRQHandler 
DMA_Error_IRQHandler      
MCM_IRQHandler            
FTFL_IRQHandler           
Read_Collision_IRQHandler 
LVD_LVW_IRQHandler        
LLW_IRQHandler            
Watchdog_IRQHandler       
RNG_IRQHandler            
I2C0_IRQHandler           
I2C1_IRQHandler           
SPI0_IRQHandler           
SPI1_IRQHandler           
SPI2_IRQHandler           
CAN0_ORed_Message_buffer_IRQHandler    
CAN0_Bus_Off_IRQHandler                
CAN0_Error_IRQHandler                  
CAN0_Tx_Warning_IRQHandler             
CAN0_Rx_Warning_IRQHandler             
CAN0_Wake_Up_IRQHandler                
I2S0_Tx_IRQHandler                     
I2S0_Rx_IRQHandler                     
CAN1_ORed_Message_buffer_IRQHandler    
CAN1_Bus_Off_IRQHandler                
CAN1_Error_IRQHandler                  
CAN1_Tx_Warning_IRQHandler             
CAN1_Rx_Warning_IRQHandler             
CAN1_Wake_Up_IRQHandler                
Reserved59_IRQHandler                  
UART0_LON_IRQHandler                   
UART0_RX_TX_IRQHandler                 
UART0_ERR_IRQHandler                   
UART1_RX_TX_IRQHandler                 
UART1_ERR_IRQHandler  
UART2_RX_TX_IRQHandler
UART2_ERR_IRQHandler  
UART3_RX_TX_IRQHandler
UART3_ERR_IRQHandler  
UART4_RX_TX_IRQHandler
UART4_ERR_IRQHandler  
UART5_RX_TX_IRQHandler
UART5_ERR_IRQHandler  
ADC0_IRQHandler
ADC1_IRQHandler
CMP0_IRQHandler
CMP1_IRQHandler
CMP2_IRQHandler
FTM0_IRQHandler
FTM1_IRQHandler
FTM2_IRQHandler
CMT_IRQHandler 
RTC_IRQHandler 
RTC_Seconds_IRQHandler  
PIT0_IRQHandler  
PIT1_IRQHandler  
PIT2_IRQHandler  
PIT3_IRQHandler  
PDB0_IRQHandler  
USB0_IRQHandler  
USBDCD_IRQHandler
ENET_1588_Timer_IRQHandler
ENET_Transmit_IRQHandler  
ENET_Receive_IRQHandler
ENET_Error_IRQHandler  
Reserved95_IRQHandler  
SDHC_IRQHandler
DAC0_IRQHandler
DAC1_IRQHandler
TSI0_IRQHandler
MCG_IRQHandler 
LPTimer_IRQHandler 
Reserved102_IRQHandler 
PORTA_IRQHandler 
PORTB_IRQHandler 
PORTC_IRQHandler 
PORTD_IRQHandler 
PORTE_IRQHandler 
Reserved108_IRQHandler
Reserved109_IRQHandler
SWI_IRQHandler 
*/
                


