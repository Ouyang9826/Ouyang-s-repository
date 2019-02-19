//舵机控制接口  PTD5
//舵机占空比    7.5-9.5  中间8.5

//电机右          PWM1    A5     FTM0_CH2_PIN
//电机右          PWM2    A4     FTM0_CH1_PIN
//电机左          PWM3    A7     FTM0_CH4_PIN
//电机左          PWM4    A6     FTM0_CH3_PIN

//编码器右                A13    FTM1_CH1_PIN  
//编码器左                A12    FTM1_CH0_PIN 

//BEEP                    E26

#include "control.h"



/*--------------------------------------------------------------------------------
*  @brief      最小二乘法
*  @since      v5.0
*  @note       最小二乘法拟合直线，求斜率、截距
--------------------------------------------------------------------------------*/
void OLS(signed short int *x, signed short int *y, uint8 n)
{
  float x_sum_average = 0;   //数组 X[N] 个元素求和 并求平均值
  float y_sum_average = 0;   //数组 Y[N] 个元素求和 并求平均值
  float x_square_sum = 0;    //数组 X[N] 个个元素的平均值
  float x_multiply_y = 0;    //数组 X[N]和Y[N]对应元素的乘积
  
  uint8 i=0;
  float z=0;
  
  z=0;
  for(i=0;i<n;i++)
  {
    z = z + x[i];
  }
  x_sum_average = z/n;
  
  z=0;
  for(i=0;i<n;i++)
  {
    z = z + y[i];
  }
  y_sum_average = z/n;
  
  z=0;
  for(i=0;i<n;i++)
  {
    z = z + x[i]*x[i];
  }
  x_square_sum = z;
  
  z=0;
  for(i=0;i<n;i++)
  {
    z = z + x[i]*y[i];
  }
  x_multiply_y = z;
  
  
  OLS_K = ( x_multiply_y - n * x_sum_average * y_sum_average)/( x_square_sum - n * x_sum_average*x_sum_average );
  OLS_R = y_sum_average - OLS_K * x_sum_average;
}



/*---------------------------------------------------------------------------------
*  @brief      编码器控制
*  @since      v1.0
*  @note       对编码器进行控制，计算出车的速度
----------------------------------------------------------------------------------*/
void Encoder_Init(void)
{
  //用于接收左侧编码器的旋转方向标志位，编码器顺时针转输出高电平，逆时针转输出低电平
  gpio_init(A8 ,GPI,0);//左编码器方向
  gpio_init(A17,GPI,0);//右编码器方向
  
  ftm_input_init(ftm1,ftm_ch0,FTM_Falling,5000);//输入捕获初始化    左电机测速  下降沿触发    5000HZ
  ftm_input_init(ftm2,ftm_ch0,FTM_Falling,5000);//输入捕获初始化    右电机测速  下降沿触发    5000HZ
  port_init_NoAlt(FTM1_CH0_PIN,PULLUP);//PTA12引脚用于捕获编码器的脉冲，因为要捕获下降沿，所以此处配上拉电阻
  port_init_NoAlt(FTM2_CH0_PIN,PULLUP);//PTA10引脚用于捕获编码器的脉冲，因为要捕获下降沿，所以此处配上拉电阻
}

/*---------------------------------------------------------------------------------
*  @brief      S3010_Control函数
*  @since      v5.0
*  @note       舵机方向控制函数
               直接复制hou@text的，用的舵机一样，图片尺寸差别也不大
----------------------------------------------------------------------------------*/
/*——————舵机打角标志——————*/
uint8 S3010_Flag = 1;
//      0       1       2
//    不打角  摄像头   电磁

/*——————舵机pd参数——————*/
float S3010_P = 6.0;//摄像头参数
float S3010_D = 2.0;

float S3010_DC_P = 2.0;//电磁参数 速度20 //P调响应速度，太小响应慢，太大引起震荡
float S3010_DC_D = 1.0;                  //D说是增加预见性，可以认为调稳定性，太小，这个太小不太好说，太大会导致系统不稳定，严重震荡，而且过弯道的时候自动减速（这样并不好）
//P     1.7
//D     2.5
int16 ideal_value_DC = 0;//电磁的理想偏差

/*——————舵机pid输出————————*/
float S3010_pid = 0.0;//由角度偏差计算出的pid输出量增量，加在中值pwm上，经过限幅之后输出

int16 Err_ec = 0;//电磁pid调节舵机
int16 PID_dir = 0;

int16 S3010_duty = S3010_midduty;//舵机pwm，这个值小于中值时车向右转，大于中值时车向左转


/*——————计算出的偏差——————*/
float AngleError_Former = 0;//上一幅图像的AngleError_Now


float AngleError1 = 0;
float AngleError2 = 0;
float AngleError3 = 0;

void S3010_Init(void)
{
  ftm_pwm_init(ftm3,ftm_ch3,50,S3010_midduty);
}
/*---------------舵机打角------------------*/
void S3010_Control(uint16 x)
{
  if(x > 950 ) x=950;
  if(x < 750 ) x=750;
  ftm_pwm_duty(ftm3 , ftm_ch3 , x);
}


/*——————摄像头控制打角——————*/
void S3010_Control_CA(void)//位置式舵机控制，利用图像识别，直接进行舵机控制
{
  if(S3010_Flag == 1)
  {
    //PID
    S3010_pid = (S3010_P * AngleError_Now)
      +(S3010_D * (AngleError_Now - AngleError_Former)); //计算输出占空比，PD公式（位置式）
 
    //限制打角范围
    if(S3010_pid > 140)
      S3010_pid = 140;
    else if(S3010_pid < -140)
      S3010_pid = -140;
    
    //计算输出量
    S3010_duty = S3010_midduty + (int16)S3010_pid;
    
    //更新偏差存储
    AngleError_Former = AngleError_Now;
    
    //限定输出范围
    if(S3010_duty > S3010_maxduty)
      S3010_duty = S3010_maxduty;
    else if(S3010_duty < S3010_minduty)
      S3010_duty = S3010_minduty;
    
    
    //输出
    ftm_pwm_duty(ftm3 , ftm_ch3 , S3010_duty);
  }
}

/*——————电磁控制打角——————*/
void S3010_Control_DC(int16 true_value_DC)//用于电磁的舵机控制，直接进行控制舵机
{
  if(S3010_Flag == 2)
  {
    Err_ec = 0;
    static int16 S3010_Error[2] = {0};
    
    S3010_Error[0] = ideal_value_DC - true_value_DC;
    
    Err_ec = S3010_Error[0] - S3010_Error[1];
    
    PID_dir = (int16)(S3010_DC_P*S3010_Error[0]
                      +S3010_DC_D*(S3010_Error[0]-S3010_Error[1]));
    
    S3010_Error[1]=S3010_Error[0];
    
    S3010_duty = S3010_midduty - PID_dir;
    //限幅
    if(S3010_duty > S3010_maxduty)
      S3010_duty = S3010_maxduty;
    else if(S3010_duty < S3010_minduty)
      S3010_duty = S3010_minduty;
    
    ftm_pwm_duty(ftm3 , ftm_ch3 , S3010_duty);//执行
  }
}



/*---------------------------------------------------------------------------------
*  @brief      ADC控制
*  @since      v1.0
*  @note       使用ADC_Display时先初始化OLED
----------------------------------------------------------------------------------*/
void ADC_Init()
{
  adc_init(ADC1_SE4a);//E0   C
  adc_init(ADC1_SE5a);//E1   B
  adc_init(ADC1_SE6a);//E2   A
  adc_init(ADC1_SE7a);//E3   D
}

/*---------------------------------------------------------------------------------
*  @brief      电机差速
*  @since      v1.0
*  @note       不知道用那种方法，先从简单的开始吧，简单是指阿克曼转角
*  @note       别的大佬所说的模拟差速完全没有思路，不知道该怎样去控制两个电机
----------------------------------------------------------------------------------*/
uint16 K_a = 1;
float K_f = 0.8;//过弯减速
float K_m = 0.7;//差速大小

//使用时要加上29，但由于测试值不准，这个值可以改一改
uint8 Differ_table[80] = 
{0 ,3 ,6 ,8 ,11 ,14 ,16 ,19 ,22 ,24 ,27 ,30 
,32 ,35 ,38 ,40 ,43 ,46 ,48 ,51 ,54 ,56 ,59 
,62 ,65 ,67 ,70 ,73 ,76 ,79 ,81 ,84 ,87 ,90 
,93 ,95 ,98 ,101 ,104 ,107 ,110 ,113 ,116 ,119 
,122 ,125 ,128 ,131 ,134 ,137 ,140 ,143 ,146 ,149 
,152 ,155 ,159 ,162 ,165 ,168 ,171 ,175 ,178 ,181 
,185 ,188 ,191 ,195 ,198 ,202 ,205 ,209 ,212 ,216 
,219 ,223 ,227 ,230 ,234 ,238 
};//值为(B*tan a)/2L对应的值*1000

void Differ_Contorl()
{
  uint8 *D_t = Differ_table;
  uint8 table_init = 24;
  if(((S3010_duty-S3010_midduty)>20||(S3010_duty-S3010_midduty)<-20)&&Force_Stop_flag!=1)//进行差速
  {
    if(S3010_duty>S3010_midduty)//左转 处理范围为 861-940
    {
      ideal_speed_L =(int16)(K_a * ideal_speed *(K_f - K_m*(*(D_t+(S3010_duty-861))+table_init)/1000.0 ));//左轮减速
      ideal_speed_R =(int16)(K_a * ideal_speed *(K_f + K_m*(*(D_t+(S3010_duty-861))+table_init)/1000.0 ));//右轮加速
    }
    if(S3010_duty<S3010_midduty)//右转 处理范围为 760-839
    {
      ideal_speed_L =(int16)(K_a * ideal_speed *(K_f + K_m*(*(D_t+(839-S3010_duty))+table_init)/1000.0 ));//左轮加速
      ideal_speed_R =(int16)(K_a * ideal_speed *(K_f - K_m*(*(D_t+(839-S3010_duty))+table_init)/1000.0 ));//右轮减速
    }
  }
  else if(Force_Stop_flag!=1)
  {
    ideal_speed_L = Speed_Set;
    ideal_speed_R = Speed_Set;
  }
}



/*---------------------------------------------------------------------------------
*  @brief      电机控制
*  @since      v1.0
*  @note       
----------------------------------------------------------------------------------*/
void Motor_Init()
{
  //PWM1，2控制右电机 PWM3，4控制左电机
  //PWM1，3有占空比，2，4为低左右电机往前转
  //PWM2，4有占空比，1，3为低左右电机往后转
  ftm_pwm_init(ftm0,ftm_ch2,16000,2000);//PWM1
  ftm_pwm_init(ftm0,ftm_ch1,16000,0);//PWM2
  ftm_pwm_init(ftm0,ftm_ch4,16000,2000);//PWM3
  ftm_pwm_init(ftm0,ftm_ch3,16000,0);//PWM4
}

//控制左电机 PWM3,4
//x 用于占空比控制  占空比=x/10 000
//x>0，电机往前转，x<0电机往后转
void Motor_Control_L(int16 x)
{
  if(x>=0)
  {
    ftm_pwm_duty(ftm0,ftm_ch4,x);//PWM3         //前
    ftm_pwm_duty(ftm0,ftm_ch3,0);//PWM4
  }
  else
  {
    ftm_pwm_duty(ftm0,ftm_ch4,0);//PWM3         //后
    ftm_pwm_duty(ftm0,ftm_ch3,-x);//PWM4
  }
}

//控制右电机 PWM1,2
void Motor_Control_R(int16 x)
{
  if(x>=0)
  {
    ftm_pwm_duty(ftm0,ftm_ch2,x);//PWM1         //前
    ftm_pwm_duty(ftm0,ftm_ch1,0);//PWM2
  }
  else
  {
    ftm_pwm_duty(ftm0,ftm_ch2,0);//PWM1         //后
    ftm_pwm_duty(ftm0,ftm_ch1,-x);//PWM2
  }
}


/*——————电机pid参数——————*/
int16 Speed_Control_P = 24;
int16 Speed_Control_I = 4;
int16 Speed_Control_D = 2;
/*—————电机理想速度——————*/
int16 ideal_speed_L = Speed_Set;
int16 ideal_speed_R = Speed_Set;
int16 ideal_speed = Speed_Set;
/*—————电机pid输出——————*/
int16 PID_Duty_L = 0;
int16 PID_Duty_R = 0;

void Speed_Control_L(int16 true_value_L)
{
  int16 increase_L = 0;
  static int16 Speed_Error_L[3]={0};
  
  Speed_Error_L[0] = ideal_speed_L - true_value_L;
  
  increase_L = (int16)(Speed_Control_P*(Speed_Error_L[0]-Speed_Error_L[1])
                       + Speed_Control_I*Speed_Error_L[0]
                       + Speed_Control_D*(Speed_Error_L[0]+Speed_Error_L[2]-2*Speed_Error_L[1]));
  
  PID_Duty_L += increase_L;
  
  Speed_Error_L[2] = Speed_Error_L[1];
  Speed_Error_L[1] = Speed_Error_L[0];
  
  //限幅
  if(PID_Duty_L > Motor_Max_Duty) PID_Duty_L = Motor_Max_Duty;
  if(PID_Duty_L < Motor_Min_Duty) PID_Duty_L = Motor_Min_Duty;
  
  //输出
  Motor_Control_L(PID_Duty_L);
}

void Speed_Control_R(int16 true_value_R)
{
  int16 increase_R = 0;
  static int16 Speed_Error_R[3]={0};
  
  Speed_Error_R[0] = ideal_speed_R - true_value_R;
  
  increase_R = (int16)(Speed_Control_P*(Speed_Error_R[0]-Speed_Error_R[1])
                       + Speed_Control_I*Speed_Error_R[0]
                       + Speed_Control_D*(Speed_Error_R[0]+Speed_Error_R[2]-2*Speed_Error_R[1]));
  
  PID_Duty_R += increase_R;
  
  Speed_Error_R[2] = Speed_Error_R[1];
  Speed_Error_R[1] = Speed_Error_R[0];
  
  //限幅
  if(PID_Duty_R > Motor_Max_Duty) PID_Duty_R = Motor_Max_Duty;
  if(PID_Duty_R < Motor_Min_Duty) PID_Duty_R = Motor_Min_Duty;
  
  //输出
  Motor_Control_R(PID_Duty_R);
}


/*---------------------------------------------------------------------------------
*  @brief      丢线停车
*  @since      v1.0
*  @note       当三个电感值全部为零时，将车停下
----------------------------------------------------------------------------------*/
uint8 Force_Stop_flag = 0;//丢线停车标志位

void Force_Stop()
{
  static uint8 Stop_Num=0;
  if(ADC_one_and_inverse[0]==1 && ADC_one_and_inverse[1]==1 && ADC_one_and_inverse[2]==1)
  {
    Stop_Num++;
    if(Stop_Num==20)
    {
      ideal_speed_R = ideal_speed_L = 0;
      Force_Stop_flag = 1;//丢线停车标志位置一
    }
      
  }
  if(ADC_one_and_inverse[0]>1||ADC_one_and_inverse[1]>1||ADC_one_and_inverse[2]>1)
    Stop_Num = 0;
}

/*---------------------------------------------------------------------------------
*  @brief      蜂鸣器控制
*  @since      v1.0
*  @note       
----------------------------------------------------------------------------------*/
//蜂鸣器初始化
void BEEP_Init()
{
  gpio_init(E26,GPO,0);
}

//t为蜂鸣器响的时间
//BEEP(4); 蜂鸣器短响8ms
void BEEP(uint8 t)
{
  gpio_set(E26,1);
  BEEP_Count = t;
}


/*---------------------------------------------------------------------------------
*  @brief      按键
*  @since      v1.0
*  @note       Key1-6 : PTC5-0
----------------------------------------------------------------------------------*/
void Key_Init()
{
  gpio_init(C5,GPI,0);//K1
  gpio_init(C4,GPI,0);//K2
  gpio_init(C3,GPI,0);//K3
  gpio_init(C2,GPI,0);//K4
  gpio_init(C1,GPI,0);//K5
  gpio_init(C0,GPI,0);//K6
}
void Key_Control()//要现初始化beep
{
  
}


/*---------------------------------------------------------------------------------
*  @brief      拨码开关
*  @since      v1.0
*  @note       由上往下 依次为 E 11，10，9 ，8 ，7 ，6 ， 5， 4 
               在 control.h中已经有 #define SwitchX  gpio_get(E  )
               直接使用  SwitchX == 0 判断开关是否打开     X：1-8
----------------------------------------------------------------------------------*/
void Switch_Init()    //初始化，上拉输入
{
  port_init (E11, ALT1 | PULLUP );
  port_init (E10, ALT1 | PULLUP );
  port_init (E9 , ALT1 | PULLUP );
  port_init (E8 , ALT1 | PULLUP );
  port_init (E7 , ALT1 | PULLUP );
  port_init (E6 , ALT1 | PULLUP );
  port_init (E5 , ALT1 | PULLUP );
  port_init (E4 , ALT1 | PULLUP );
}



/*---------------------------------------------------------------------------------
//以下皆为模块测试程序
//不用理会
---------------------------------------------------------------------------------*/

void Motor_test(uint8 x)//X:0 后退  X：!0前进
{
  if(x!=0)
  {
    gpio_init(A4,GPO,0);                             //前
    gpio_init(A6,GPO,0);
    ftm_pwm_init(ftm0,ftm_ch2,16000,2000);//PWM1
    ftm_pwm_init(ftm0,ftm_ch4,16000,2000);//PWM3
  }
  if(x==0)
  {
    gpio_init(A5,GPO,0);                           //后
    gpio_init(A7,GPO,0);
    ftm_pwm_init(ftm0,ftm_ch1,16000,2000);//PWM2
    ftm_pwm_init(ftm0,ftm_ch3,16000,2000);//PWM4
  }
}

void servo_test()//舵机测试
{
  int i=850;
  ftm_pwm_init(ftm0,ftm_ch5,50,i);
  while(1)
  {
//    i++;
//    systick_delay_ms(10);
//    if(i >= 950)i = 750;
//    ftm_pwm_duty(ftm0,ftm_ch5,i);  
    //OLED_Print_Num(0, 0, i);
  }
}
void motor_test(uint8 x)//电机测试
{
  if(x!=0)
  {
    gpio_init(A4,GPO,0);                             //前
    gpio_init(A6,GPO,0);
    ftm_pwm_init(ftm0,ftm_ch2,20000,0);//PWM1
    ftm_pwm_init(ftm0,ftm_ch4,20000,0);//PWM3
  }
  if(x==0)
  {
    gpio_init(A5,GPO,0);                           //后
    gpio_init(A7,GPO,0);
    ftm_pwm_init(ftm0,ftm_ch1,20000,2000);//PWM2
    ftm_pwm_init(ftm0,ftm_ch3,20000,2000);//PWM4
  }
}

void adc_test()
{
  adc_init(ADC1_SE4a);//E0   C
  adc_init(ADC1_SE5a);//E1   B
  adc_init(ADC1_SE6a);//E2   A
  adc_init(ADC1_SE7a);//E3   D
  
  while(1)
  {
    OLED_Print_Unum1(0, 0, adc_once(ADC1_SE4a,ADC_16bit));
    OLED_Print_Unum1(0, 2, adc_once(ADC1_SE5a,ADC_16bit));
    OLED_Print_Unum1(0, 4, adc_once(ADC1_SE6a,ADC_16bit));
    OLED_Print_Unum1(0, 6, adc_once(ADC1_SE7a,ADC_16bit));
    systick_delay_ms(100);
  }
}

void bianma_test()//编码器测试
{
  gpio_init(A8 ,GPI,0);
  gpio_init(A17,GPI,0);
  FTM1_input_encoder();//输入左编码器输入捕获
  FTM2_input_encoder();//输入右编码器输入捕获
  while(1)
    {
      OLED_Print_Unum1(0, 0, cesu_left_count);
      OLED_Print_Unum1(0, 2, cesu_right_count);
    }
  
}

void Key_test()
{
  if(K1 == 0)Get_addline();
  if(K2 == 0)BEEP(20);
  if(K3 == 0)BEEP(20);
  if(K4 == 0)BEEP(20);
  if(K5 == 0)BEEP(20);
  if(K6 == 0)BEEP(20);
}


