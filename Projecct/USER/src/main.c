/********************************************
逐飞科技 总钻风-摄像头  历程
Designed by Fly Sir
软件版本:V1.1
最后更新:2016年5月3日
相关信息参考下列地址：
淘宝店：https://seekfree.taobao.com/
------------------------------------
软件版本： IAR 7.2 or MDK 5.17
目标核心： MK60DN512VLL10
============================================
MT9V032接线定义：
------------------------------------ 
    模块管脚            单片机管脚
    SDA(51的RX)         PTC17
    SCL(51的TX)         PTC16
    场中断(VSY)         PTC6
    像素中断(PCLK)      PTC18
    数据口(D0-D7)       PTC8-PTC15 

    串口  
    波特率 115200 
    数据位 8 
    校验位 无
    停止位 1位
    流控   无
    串口连接注意事项：切勿使用蓝牙等无线串口连接
    RX                  PTD3
    TX                  PTD2
============================================

分辨率是                188*120
摄像头参数设置可以到    SEEKFREE-->h_file-->SEEKFREE_MT9V032.h

总钻风-摄像头测试步骤：
1.下载程序到开发板
2.插上串口线或者USB转TTL
3.接好MT9V032模块接线
4.通电在TFT液晶上即可观看    
*********************************************/  
#include "headfile.h"
#include "math.h"

//大津法   待改进
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)   
{
    #define GrayScale 256
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = width * height;
    uint8 threshold = 0;
    uint8* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }

    //统计灰度级中每个像素在整幅图像中的个数  
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            pixelCount[(int)data[i * width + j]]++;  //将像素值作为计数数组的下标
        }
    }

    //计算每个像素在整幅图像中的比例  
    float maxPro = 0.0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
        if (pixelPro[i] > maxPro)
        {
            maxPro = pixelPro[i];
        }
    }

    //遍历灰度级[0,255]  
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    for (i = 0; i < GrayScale; i++)     // i作为阈值
    {
        w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
        for (j = 0; j < GrayScale; j++)
        {
            if (j <= i)   //背景部分  
            {
                w0 += pixelPro[j];
                u0tmp += j * pixelPro[j];
            }
            else   //前景部分  
            {
                w1 += pixelPro[j];
                u1tmp += j * pixelPro[j];
            }
        }
        u0 = u0tmp / w0;
        u1 = u1tmp / w1;
        u = u0tmp + u1tmp;
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = i;
        }
    }

    return threshold;
}

void OLED_display(void)
{
  OLED_P6x8Str(0,0,"adc_get");
  
  OLED_P6x8Str(64,0,"0: ");
  OLED_Print_Unum2(76,0,ADC_ave[0]);
  
  OLED_P6x8Str(0,1,"1: ");
  OLED_Print_Unum2(12,1,ADC_ave[1]);
  
  OLED_P6x8Str(64,1,"2: ");
  OLED_Print_Unum2(76,1,ADC_ave[2]);
  
  OLED_P6x8Str(0,2,"normal");
  
  OLED_P6x8Str(64,2,"0: ");
  OLED_Print_Unum2(76,2,ADC_one_and_inverse[0]);
  
  OLED_P6x8Str(0,3,"1: ");
  OLED_Print_Unum2(12,3,ADC_one_and_inverse[1]);
  
  OLED_P6x8Str(64,3,"2: ");
  OLED_Print_Unum2(76,3,ADC_one_and_inverse[2]);
  
  OLED_P6x8Str(0,4,"err: ");
  OLED_Print_Num2(24,4,Road_err);
  
  OLED_P6x8Str(64,4,"dut: ");
  OLED_Print_Unum2(88,4,S3010_duty);
  
  OLED_P6x8Str(0,6,"D_L: ");
  OLED_Print_Num2(24,6,PID_Duty_L);
  
  OLED_P6x8Str(64,6,"D_R: ");
  OLED_Print_Num2(88,6,PID_Duty_R);
  
  OLED_P6x8Str(0,7,"s_L: ");
  OLED_Print_Num2(24,7,Left_Speed_now);
  
  OLED_P6x8Str(64,7,"s_R: ");
  OLED_Print_Num2(88,7,Right_Speed_now);
}

void uart_display()
{
  char data[80]={0};
  sprintf(data,"err:%d duty:%d\tduty_L:%d duty_R:%d\tV_L:%d V_R:%d\n",Road_err,S3010_duty,PID_Duty_L,PID_Duty_R,Left_Speed_now,Right_Speed_now);
  uart_putbuff(uart0,data,80);
  //sprintf(data,"duty_L:%d duty_R:%d ",PID_Duty_L,PID_Duty_R);
  //uart_putbuff(uart0,data,30);
  //sprintf(data,"V_L:%d V_R:%d \n",Left_Speed_now,Right_Speed_now);
  //uart_putbuff(uart0,data,80);
  //uart_putchar(uart0,'\n');
}

uint8 image_threshold;        //图像阈值
uint32 use_time;              //计算大津法时间
int main(void)
{
    get_clk();//上电后必须运行一次这个函数，获取各个频率信息，便于后面各个模块的参数设置
//    uart_init (uart0, 9600);                          //初始换串口与电脑通信
//    DisableInterrupts;//关闭中断
   
    gpio_init(A4,GPO,0);      //解决NMI问题   
    
    add_line_init();          //得到补线数组
    BEEP_Init();              //蜂鸣器初始化
    Key_Init();               //按键初始化
    Switch_Init();            //拨码开关初始化
    camera_init();            //摄像头初始化
    
    lcd_init();               //TFT屏幕初始化     谜之BUG 不要改这两个初始化的顺序!!!
    S3010_Init();             //舵机初始化
    
    OLED_Init();              //OLED初始化
    ADC_Init();               //ADC初始化
    pit_init_ms(pit0,2);      //pit初始化
    enable_irq(PIT0_IRQn);    //使能IRQ（中断请求）
    
    Encoder_Init();           //编码器初始化
    Motor_Init();             //电机初始化
    //Motor_Control_R(-2000);
    
    EnableInterrupts;         //开启中断    
    for(;;)
    {
      Key_test();
      if(mt9v032_finish_flag) //完成一幅图片采集
      {
        mt9v032_finish_flag = 0;
            
        image_reduce_size();  //缩小图片尺寸
        
        pit_time_start(pit1); //开始计时
        image_threshold = otsuThreshold(image_little[0],image_col,image_row);  //大津法计算阈值
        use_time = pit_time_get(pit1)/bus_clk_mhz;          //计算大津法程序消耗时间，单位微秒。
        //本例程大津法时间绝对很长，想直接用必然不可行，需自行优化。
        //有人说：你咋不直接优化好呢。我说：你想得美，要不要我直接把车做好给你啊。
        pit_close(pit1);      //停止计时
        
        get_img_bin(image_little[0], image_threshold);//得到二值化图像
        Get_Midline();                         //计算中线
                
        S3010_Control_CA();                 //控制舵机打一次角
       
        if(Switch1==0)//跑起来是建议关闭Switch1,关闭显示，这样整个while循环只要10ms
        {
          //丢线程序测试
          //          OLED_P6x8Str(0,0,"L_S1:");OLED_Print_Num2(5*6,0,L_Lost_Line[0][0]);
          //          OLED_P6x8Str(0,1,"L_E1:");OLED_Print_Num2(5*6,1,L_Lost_Line[0][1]);
          //          OLED_P6x8Str(0,2,"L_S1:");OLED_Print_Num2(5*6,2,L_Lost_Line[1][0]);
          //          OLED_P6x8Str(0,3,"L_E2:");OLED_Print_Num2(5*6,3,L_Lost_Line[1][1]);
          //          OLED_P6x8Str(0,4,"R_S1:");OLED_Print_Num2(5*6,4,R_Lost_Line[0][0]);
          //          OLED_P6x8Str(0,5,"R_E1:");OLED_Print_Num2(5*6,5,R_Lost_Line[0][1]);
          //          OLED_P6x8Str(0,6,"R_S1:");OLED_Print_Num2(5*6,6,R_Lost_Line[1][0]);
          //          OLED_P6x8Str(0,7,"R_E2:");OLED_Print_Num2(5*6,7,R_Lost_Line[1][1]);
          
          
          //          displayimage032(image_little[0]);             //灰度图片显示
          display_img_bin();                            //TFT显示二值化图片
          //          display_midline();                            //显示屏幕中线
          
          //          OLED_P6x8Str(0,0,"L_K:");OLED_Print_Num2(4*6,0,(int)(Straight_L_K*10000));//斜率
          //          OLED_P6x8Str(0,1,"R_K:");OLED_Print_Num2(4*6,1,(int)(Straight_R_K*10000));
          
          OLED_P6x8Str(0,0,"L_K:");OLED_Print_Num2(4*6,0,(int)(Straight_L_K*10000));//斜率
          OLED_P6x8Str(64,0,"R_K:");OLED_Print_Num2(4*6+64,0,(int)(Straight_R_K*10000));
          
          OLED_P6x8Str(0,1,"USE:");OLED_Print_Num2(4*6,1,DivLine);//有效行
          
          OLED_P6x8Str(64,1,"S_d:");OLED_Print_Num2(4*6+64 , 1, S3010_duty);//舵机占空比显示
        }
        
//        displayimage032(image_little[0]);             //灰度图片显示    
        //发送二值化图像至上位机
        //            p = image[0];
        //            uart_putchar(uart2,0x00);uart_putchar(uart2,0xff);uart_putchar(uart2,0x01);uart_putchar(uart2,0x01);//发送命令
        //            for(i=0; i<COL*ROW; i++)
        //            {
        //                if(p[i]>image_threshold)    uart_putchar(uart2,0xff);
        //                else                        uart_putchar(uart2,0x00);
        //            }
        //seekfree_sendimg_032();                             //发送灰度至上位机
        if(Switch2==0)//电磁数据显示
        {
          OLED_display();
          //uart_display();
        }
      }
      
    }
}


