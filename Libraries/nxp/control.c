//������ƽӿ�  PTD5
//���ռ�ձ�    7.5-9.5  �м�8.5

//�����          PWM1    A5     FTM0_CH2_PIN
//�����          PWM2    A4     FTM0_CH1_PIN
//�����          PWM3    A7     FTM0_CH4_PIN
//�����          PWM4    A6     FTM0_CH3_PIN

//��������                A13    FTM1_CH1_PIN  
//��������                A12    FTM1_CH0_PIN 

//BEEP                    E26

#include "control.h"



/*--------------------------------------------------------------------------------
*  @brief      ��С���˷�
*  @since      v5.0
*  @note       ��С���˷����ֱ�ߣ���б�ʡ��ؾ�
--------------------------------------------------------------------------------*/
void OLS(signed short int *x, signed short int *y, uint8 n)
{
  float x_sum_average = 0;   //���� X[N] ��Ԫ����� ����ƽ��ֵ
  float y_sum_average = 0;   //���� Y[N] ��Ԫ����� ����ƽ��ֵ
  float x_square_sum = 0;    //���� X[N] ����Ԫ�ص�ƽ��ֵ
  float x_multiply_y = 0;    //���� X[N]��Y[N]��ӦԪ�صĳ˻�
  
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
*  @brief      ����������
*  @since      v1.0
*  @note       �Ա��������п��ƣ�����������ٶ�
----------------------------------------------------------------------------------*/
void Encoder_Init(void)
{
  //���ڽ���������������ת�����־λ��������˳ʱ��ת����ߵ�ƽ����ʱ��ת����͵�ƽ
  gpio_init(A8 ,GPI,0);//�����������
  gpio_init(A17,GPI,0);//�ұ���������
  
  ftm_input_init(ftm1,ftm_ch0,FTM_Falling,5000);//���벶���ʼ��    ��������  �½��ش���    5000HZ
  ftm_input_init(ftm2,ftm_ch0,FTM_Falling,5000);//���벶���ʼ��    �ҵ������  �½��ش���    5000HZ
  port_init_NoAlt(FTM1_CH0_PIN,PULLUP);//PTA12�������ڲ�������������壬��ΪҪ�����½��أ����Դ˴�����������
  port_init_NoAlt(FTM2_CH0_PIN,PULLUP);//PTA10�������ڲ�������������壬��ΪҪ�����½��أ����Դ˴�����������
}

/*---------------------------------------------------------------------------------
*  @brief      S3010_Control����
*  @since      v5.0
*  @note       ���������ƺ���
               ֱ�Ӹ���hou@text�ģ��õĶ��һ����ͼƬ�ߴ���Ҳ����
----------------------------------------------------------------------------------*/
/*�����������������Ǳ�־������������*/
uint8 S3010_Flag = 1;
//      0       1       2
//    �����  ����ͷ   ���

/*���������������pd����������������*/
float S3010_P = 6.0;//����ͷ����
float S3010_D = 2.0;

float S3010_DC_P = 2.0;//��Ų��� �ٶ�20 //P����Ӧ�ٶȣ�̫С��Ӧ����̫��������
float S3010_DC_D = 1.0;                  //D˵������Ԥ���ԣ�������Ϊ���ȶ��ԣ�̫С�����̫С��̫��˵��̫��ᵼ��ϵͳ���ȶ��������𵴣����ҹ������ʱ���Զ����٣����������ã�
//P     1.7
//D     2.5
int16 ideal_value_DC = 0;//��ŵ�����ƫ��

/*���������������pid�������������������*/
float S3010_pid = 0.0;//�ɽǶ�ƫ��������pid�����������������ֵpwm�ϣ������޷�֮�����

int16 Err_ec = 0;//���pid���ڶ��
int16 PID_dir = 0;

int16 S3010_duty = S3010_midduty;//���pwm�����ֵС����ֵʱ������ת��������ֵʱ������ת


/*�������������������ƫ�����������*/
float AngleError_Former = 0;//��һ��ͼ���AngleError_Now


float AngleError1 = 0;
float AngleError2 = 0;
float AngleError3 = 0;

void S3010_Init(void)
{
  ftm_pwm_init(ftm3,ftm_ch3,50,S3010_midduty);
}
/*---------------������------------------*/
void S3010_Control(uint16 x)
{
  if(x > 950 ) x=950;
  if(x < 750 ) x=750;
  ftm_pwm_duty(ftm3 , ftm_ch3 , x);
}


/*����������������ͷ���ƴ�ǡ�����������*/
void S3010_Control_CA(void)//λ��ʽ������ƣ�����ͼ��ʶ��ֱ�ӽ��ж������
{
  if(S3010_Flag == 1)
  {
    //PID
    S3010_pid = (S3010_P * AngleError_Now)
      +(S3010_D * (AngleError_Now - AngleError_Former)); //�������ռ�ձȣ�PD��ʽ��λ��ʽ��
 
    //���ƴ�Ƿ�Χ
    if(S3010_pid > 140)
      S3010_pid = 140;
    else if(S3010_pid < -140)
      S3010_pid = -140;
    
    //���������
    S3010_duty = S3010_midduty + (int16)S3010_pid;
    
    //����ƫ��洢
    AngleError_Former = AngleError_Now;
    
    //�޶������Χ
    if(S3010_duty > S3010_maxduty)
      S3010_duty = S3010_maxduty;
    else if(S3010_duty < S3010_minduty)
      S3010_duty = S3010_minduty;
    
    
    //���
    ftm_pwm_duty(ftm3 , ftm_ch3 , S3010_duty);
  }
}

/*��������������ſ��ƴ�ǡ�����������*/
void S3010_Control_DC(int16 true_value_DC)//���ڵ�ŵĶ�����ƣ�ֱ�ӽ��п��ƶ��
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
    //�޷�
    if(S3010_duty > S3010_maxduty)
      S3010_duty = S3010_maxduty;
    else if(S3010_duty < S3010_minduty)
      S3010_duty = S3010_minduty;
    
    ftm_pwm_duty(ftm3 , ftm_ch3 , S3010_duty);//ִ��
  }
}



/*---------------------------------------------------------------------------------
*  @brief      ADC����
*  @since      v1.0
*  @note       ʹ��ADC_Displayʱ�ȳ�ʼ��OLED
----------------------------------------------------------------------------------*/
void ADC_Init()
{
  adc_init(ADC1_SE4a);//E0   C
  adc_init(ADC1_SE5a);//E1   B
  adc_init(ADC1_SE6a);//E2   A
  adc_init(ADC1_SE7a);//E3   D
}

/*---------------------------------------------------------------------------------
*  @brief      �������
*  @since      v1.0
*  @note       ��֪�������ַ������ȴӼ򵥵Ŀ�ʼ�ɣ�����ָ������ת��
*  @note       ��Ĵ�����˵��ģ�������ȫû��˼·����֪��������ȥ�����������
----------------------------------------------------------------------------------*/
uint16 K_a = 1;
float K_f = 0.8;//�������
float K_m = 0.7;//���ٴ�С

//ʹ��ʱҪ����29�������ڲ���ֵ��׼�����ֵ���Ը�һ��
uint8 Differ_table[80] = 
{0 ,3 ,6 ,8 ,11 ,14 ,16 ,19 ,22 ,24 ,27 ,30 
,32 ,35 ,38 ,40 ,43 ,46 ,48 ,51 ,54 ,56 ,59 
,62 ,65 ,67 ,70 ,73 ,76 ,79 ,81 ,84 ,87 ,90 
,93 ,95 ,98 ,101 ,104 ,107 ,110 ,113 ,116 ,119 
,122 ,125 ,128 ,131 ,134 ,137 ,140 ,143 ,146 ,149 
,152 ,155 ,159 ,162 ,165 ,168 ,171 ,175 ,178 ,181 
,185 ,188 ,191 ,195 ,198 ,202 ,205 ,209 ,212 ,216 
,219 ,223 ,227 ,230 ,234 ,238 
};//ֵΪ(B*tan a)/2L��Ӧ��ֵ*1000

void Differ_Contorl()
{
  uint8 *D_t = Differ_table;
  uint8 table_init = 24;
  if(((S3010_duty-S3010_midduty)>20||(S3010_duty-S3010_midduty)<-20)&&Force_Stop_flag!=1)//���в���
  {
    if(S3010_duty>S3010_midduty)//��ת ����ΧΪ 861-940
    {
      ideal_speed_L =(int16)(K_a * ideal_speed *(K_f - K_m*(*(D_t+(S3010_duty-861))+table_init)/1000.0 ));//���ּ���
      ideal_speed_R =(int16)(K_a * ideal_speed *(K_f + K_m*(*(D_t+(S3010_duty-861))+table_init)/1000.0 ));//���ּ���
    }
    if(S3010_duty<S3010_midduty)//��ת ����ΧΪ 760-839
    {
      ideal_speed_L =(int16)(K_a * ideal_speed *(K_f + K_m*(*(D_t+(839-S3010_duty))+table_init)/1000.0 ));//���ּ���
      ideal_speed_R =(int16)(K_a * ideal_speed *(K_f - K_m*(*(D_t+(839-S3010_duty))+table_init)/1000.0 ));//���ּ���
    }
  }
  else if(Force_Stop_flag!=1)
  {
    ideal_speed_L = Speed_Set;
    ideal_speed_R = Speed_Set;
  }
}



/*---------------------------------------------------------------------------------
*  @brief      �������
*  @since      v1.0
*  @note       
----------------------------------------------------------------------------------*/
void Motor_Init()
{
  //PWM1��2�����ҵ�� PWM3��4��������
  //PWM1��3��ռ�ձȣ�2��4Ϊ�����ҵ����ǰת
  //PWM2��4��ռ�ձȣ�1��3Ϊ�����ҵ������ת
  ftm_pwm_init(ftm0,ftm_ch2,16000,2000);//PWM1
  ftm_pwm_init(ftm0,ftm_ch1,16000,0);//PWM2
  ftm_pwm_init(ftm0,ftm_ch4,16000,2000);//PWM3
  ftm_pwm_init(ftm0,ftm_ch3,16000,0);//PWM4
}

//�������� PWM3,4
//x ����ռ�ձȿ���  ռ�ձ�=x/10 000
//x>0�������ǰת��x<0�������ת
void Motor_Control_L(int16 x)
{
  if(x>=0)
  {
    ftm_pwm_duty(ftm0,ftm_ch4,x);//PWM3         //ǰ
    ftm_pwm_duty(ftm0,ftm_ch3,0);//PWM4
  }
  else
  {
    ftm_pwm_duty(ftm0,ftm_ch4,0);//PWM3         //��
    ftm_pwm_duty(ftm0,ftm_ch3,-x);//PWM4
  }
}

//�����ҵ�� PWM1,2
void Motor_Control_R(int16 x)
{
  if(x>=0)
  {
    ftm_pwm_duty(ftm0,ftm_ch2,x);//PWM1         //ǰ
    ftm_pwm_duty(ftm0,ftm_ch1,0);//PWM2
  }
  else
  {
    ftm_pwm_duty(ftm0,ftm_ch2,0);//PWM1         //��
    ftm_pwm_duty(ftm0,ftm_ch1,-x);//PWM2
  }
}


/*���������������pid����������������*/
int16 Speed_Control_P = 24;
int16 Speed_Control_I = 4;
int16 Speed_Control_D = 2;
/*������������������ٶȡ�����������*/
int16 ideal_speed_L = Speed_Set;
int16 ideal_speed_R = Speed_Set;
int16 ideal_speed = Speed_Set;
/*�������������pid���������������*/
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
  
  //�޷�
  if(PID_Duty_L > Motor_Max_Duty) PID_Duty_L = Motor_Max_Duty;
  if(PID_Duty_L < Motor_Min_Duty) PID_Duty_L = Motor_Min_Duty;
  
  //���
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
  
  //�޷�
  if(PID_Duty_R > Motor_Max_Duty) PID_Duty_R = Motor_Max_Duty;
  if(PID_Duty_R < Motor_Min_Duty) PID_Duty_R = Motor_Min_Duty;
  
  //���
  Motor_Control_R(PID_Duty_R);
}


/*---------------------------------------------------------------------------------
*  @brief      ����ͣ��
*  @since      v1.0
*  @note       ���������ֵȫ��Ϊ��ʱ������ͣ��
----------------------------------------------------------------------------------*/
uint8 Force_Stop_flag = 0;//����ͣ����־λ

void Force_Stop()
{
  static uint8 Stop_Num=0;
  if(ADC_one_and_inverse[0]==1 && ADC_one_and_inverse[1]==1 && ADC_one_and_inverse[2]==1)
  {
    Stop_Num++;
    if(Stop_Num==20)
    {
      ideal_speed_R = ideal_speed_L = 0;
      Force_Stop_flag = 1;//����ͣ����־λ��һ
    }
      
  }
  if(ADC_one_and_inverse[0]>1||ADC_one_and_inverse[1]>1||ADC_one_and_inverse[2]>1)
    Stop_Num = 0;
}

/*---------------------------------------------------------------------------------
*  @brief      ����������
*  @since      v1.0
*  @note       
----------------------------------------------------------------------------------*/
//��������ʼ��
void BEEP_Init()
{
  gpio_init(E26,GPO,0);
}

//tΪ���������ʱ��
//BEEP(4); ����������8ms
void BEEP(uint8 t)
{
  gpio_set(E26,1);
  BEEP_Count = t;
}


/*---------------------------------------------------------------------------------
*  @brief      ����
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
void Key_Control()//Ҫ�ֳ�ʼ��beep
{
  
}


/*---------------------------------------------------------------------------------
*  @brief      ���뿪��
*  @since      v1.0
*  @note       �������� ����Ϊ E 11��10��9 ��8 ��7 ��6 �� 5�� 4 
               �� control.h���Ѿ��� #define SwitchX  gpio_get(E  )
               ֱ��ʹ��  SwitchX == 0 �жϿ����Ƿ��     X��1-8
----------------------------------------------------------------------------------*/
void Switch_Init()    //��ʼ������������
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
//���½�Ϊģ����Գ���
//�������
---------------------------------------------------------------------------------*/

void Motor_test(uint8 x)//X:0 ����  X��!0ǰ��
{
  if(x!=0)
  {
    gpio_init(A4,GPO,0);                             //ǰ
    gpio_init(A6,GPO,0);
    ftm_pwm_init(ftm0,ftm_ch2,16000,2000);//PWM1
    ftm_pwm_init(ftm0,ftm_ch4,16000,2000);//PWM3
  }
  if(x==0)
  {
    gpio_init(A5,GPO,0);                           //��
    gpio_init(A7,GPO,0);
    ftm_pwm_init(ftm0,ftm_ch1,16000,2000);//PWM2
    ftm_pwm_init(ftm0,ftm_ch3,16000,2000);//PWM4
  }
}

void servo_test()//�������
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
void motor_test(uint8 x)//�������
{
  if(x!=0)
  {
    gpio_init(A4,GPO,0);                             //ǰ
    gpio_init(A6,GPO,0);
    ftm_pwm_init(ftm0,ftm_ch2,20000,0);//PWM1
    ftm_pwm_init(ftm0,ftm_ch4,20000,0);//PWM3
  }
  if(x==0)
  {
    gpio_init(A5,GPO,0);                           //��
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

void bianma_test()//����������
{
  gpio_init(A8 ,GPI,0);
  gpio_init(A17,GPI,0);
  FTM1_input_encoder();//��������������벶��
  FTM2_input_encoder();//�����ұ��������벶��
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


