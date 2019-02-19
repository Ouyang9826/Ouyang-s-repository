/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2016,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		isr.c
 * @brief      		中断函数库
 * @company	   	成都逐飞科技有限公司
 * @author     		Go For It(1325536866)
 * @version    		v1.0
 * @Software 		IAR 7.7 or MDK 5.17
 * @Target core		MK60DN512VLL10
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2016-02-25
 ********************************************************************************************************************/



#include "isr.h"

int16 cesu_left_count=0;//左侧电机计数
int16 cesu_right_count=0;//右侧电机计数
int16 Left_Speed_now=0;//左侧测速结果
int16 Right_Speed_now=0;//左侧测速结果
int16 Speed_now=0;//测速结果

int16 Road_HD=0;

int8 PIT0_Count_10ms = 0;//pit0 10ms中断计数
int8 PIT0_Count_20ms = 0;//pit0 20ms中断计数
int8 BEEP_Count = 0;//蜂鸣器响时间计数

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PROTA中断执行函数
//  @return     void   
//  @since      v1.0
//  Sample usage:               当A口启用中断功能且发生中断的时候会自动执行该函数
//-------------------------------------------------------------------------------------------------------------------
void PORTA_IRQHandler(void)
{
    //清除中断标志第一种方法直接操作寄存器，每一位对应一个引脚
	PORTA->ISFR = 0xffffffff;
	//使用我们编写的宏定义清除发生中断的引脚
	//PORTA_FLAG_CLR(A1);
}

void PORTC_IRQHandler(void)
{
    //清除中断标志第一种方法直接操作寄存器，每一位对应一个引脚
	PORTC->ISFR = 0xffffffff;
	//使用我们编写的宏定义清除发生中断的引脚
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
//PIT0中断服务函数，用于在oled屏幕上显示adc采集值
void PIT0_IRQHandler(void)
{
  PIT_FlAG_CLR(pit0);//清除中断标志
    
  PIT0_Count_10ms++;
  //PIT0_Count_20ms++;
  
  if(PIT0_Count_10ms==2)
  {
    PIT0_Count_10ms=0;
    
    get_data();//电感数采集
    normalize();//归一化与差值处理
    Road_Cal();
    
    //Force_Stop();//丢线停车
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

  S3010_Control_DC(Road_err);//舵机控制
  //速度计算
  if(gpio_get(A8))//左轮测速
    Left_Speed_now = cesu_left_count;
  else
    Left_Speed_now = -cesu_left_count;
  if(gpio_get(A17))//右轮测速
    Right_Speed_now = -cesu_right_count;
  else
    Right_Speed_now = cesu_right_count;
  
  Speed_now = (Left_Speed_now + Right_Speed_now)/2;
  
  if(Road_HD != 0)
  {
    Road_HD -= Speed_now;
  }
  
  cesu_left_count = 0;//清除计数
  cesu_right_count = 0;//清除计数
  
  //Differ_Contorl();//差速处理
  
//  Speed_Control_L(Left_Speed_now);
//  Speed_Control_R(Right_Speed_now);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      
//  @return     void   
//  @since      v1.0
//  Sample usage:                
//-------------------------------------------------------------------------------------------------------------------
//FTM1输入捕捉中断服务函数，用于给左电机测速
void FTM1_IRQHandler(void)
{
  uint8 s = FTM1_INPUT_FLAG_READ;        //读取捕捉和比较状态  All CHnF bits can be checked using only one read of STATUS.
  uint8 CHn;
  
  FTM1_INPUT_FLAG_READ = 0x00;           //清中断标志位
  
  CHn = 0;                      //CH0通道，赋值为0，若CH1通道则赋值为1
  if( s & (1 << CHn) )
  {
    //FTM_IRQ_DIS(FTM1, CHn);   //禁止输入捕捉中断，不禁止也没影响
    /*     用户任务       */
//    if( gpio_get(A8 ) )//Left_Direction_Pin))         //左轮正转则编码器逆时针转（安装方式决定），输出低电平
//    { cesu_left_count++;        //车轮反转count--
//    }
//    else
//    { cesu_left_count--;        //车轮正转count++
//
//    }
    cesu_left_count++;
  }
}
void FTM2_IRQHandler(void)            //用来右电机测速
{
  uint8 s = FTM2_INPUT_FLAG_READ;        //读取捕捉和比较状态  All CHnF bits can be checked using only one read of STATUS.
  uint8 CHn;
  
  FTM2_INPUT_FLAG_READ = 0x00;           //清中断标志位
  
  CHn = 0;                      //CH0通道，赋值为0，若CH1通道则赋值为1
  if( s & (1 << CHn) )
  {
    //FTM_IRQ_DIS(FTM2, CHn);   //禁止输入捕捉中断，不禁止也没影响
    /*     用户任务       */
//    if( gpio_get(A17) )//转向与左轮相反     FTM2_CH1_PIN   //左轮正转则编码器逆时针转（安装方式决定），输出低电平
//    {
//     cesu_right_count--;        //车轮反转count--
//    }
//    else
//    { 
//      cesu_right_count++;        //车轮正转count++
//    }
    cesu_right_count++;
  }
}




//-------------------------------------------------------------------------------------------------------------------
//  @brief      UART3中断执行函数
//  @return     void   
//  @since      v1.0
//  Sample usage:               当UART3启用中断功能且发生中断的时候会自动执行该函数
//-------------------------------------------------------------------------------------------------------------------
void UART3_RX_TX_IRQHandler(void)
{
    if(UART3->S1 & UART_S1_RDRF_MASK)                                     //接收数据寄存器满
    {
        //用户需要处理接收数据
        mt9v032_cof_uart_interrupt();
    }
    if(UART3->S1 & UART_S1_TDRE_MASK )                                    //发送数据寄存器空
    {
        //用户需要处理发送数据

    }
}

void UART4_RX_TX_IRQHandler(void)
{
    if(UART4->S1 & UART_S1_RDRF_MASK)                                     //接收数据寄存器满
    {
        
    }
    if(UART4->S1 & UART_S1_TDRE_MASK )                                    //发送数据寄存器空
    {
        //用户需要处理发送数据

    }
}


/*
中断函数名称，用于设置对应功能的中断函数
Sample usage:当前启用了DMA0中断，然后就到下面去找哪个是DMA0的中断函数名称，找到后写一个该名称的函数即可
void DMA0_IRQHandler(void)
{
    ;
}
记得进入中断后清除标志位


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
                


