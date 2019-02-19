#include "data_process.h"

uint16 AD_MAX[3]={3600,3600,3600};
uint16 ADC_ave[3];
uint16 ADC_one_and_inverse[3]={0};//用来存放归一化的值

int16 Road_err=0;

uint8 island_direction;//环岛方向 0为未判断 1为“左环岛” 2为“右环岛”
uint8 island_flag;//环岛标志位 0为未判断

/*--------------------------------------------------------------------------------
*  @brief      根号计算函数 InvSqrt()
*  @param      x         被开方数
*  @since      v1.0
*  @Sample usage:    get_img_bin(image[0], threshold)
*  @note       计算速度比直接使用math里面的函数更快
--------------------------------------------------------------------------------*/
float InvSqrt(float x)//开方
{
  float xhalf = 0.5f*x;
  int i = *(int*)&x; // get bits for floating VALUE 
  i = 0x5f375a86- (i>>1); // gives initial guess y0
  x = *(float*)&i; // convert bits BACK to float
  x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
  x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
  return 1/x;
}

/*--------------------------------------------------------------------------------
*  @brief      获取ADC的采集值
*  @param      void
*  @since      v1.0
*  @note       采集adc的值，利用adc的库直接进行采集adc值，也可以用手动对数据进行处理
--------------------------------------------------------------------------------*/
void get_data(void)
{
  uint16 ADC_get[3][5]={0};
  uint16 ADC_temp=0;
  int min;
  
  for(int i = 0;i<5;i++)
  {
    ADC_get[0][i]=adc_once(ADC1_SE6a,ADC_12bit);
    ADC_get[1][i]=adc_once(ADC1_SE5a,ADC_12bit);
    ADC_get[2][i]=adc_once(ADC1_SE4a,ADC_12bit);
  }
  
  //选择法排序 从小到大进行排序
  
//  for(int i=0;i<3;i++)     //6个电感
//  {
//    for(int j=0;j<4;j++)                 //五个数据排序  之前是5导致数组溢出了，MMP啊！
//    {
//      for(int k=0;k<4-j;k++)
//      {
//        if(ADC_get[i][k] > ADC_get[i][k+1])     //前面的比后面的大  则进行交换
//        {
//          ADC_temp = ADC_get[i][k+1];
//          ADC_get[i][k+1] = ADC_get[i][k];
//          ADC_get[i][k] = ADC_temp;
//        }
//      }
//    }
//  }
  
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<4;j++)
    {
      min=j;
      for(int k=5-j;k<5;k++)
      {
        if(ADC_get[i][min]>ADC_get[i][k])
          min=k;
      }
      if(j!=min)
      {
        ADC_temp=ADC_get[i][j];
        ADC_get[i][j]=ADC_get[i][min];
        ADC_get[i][min]=ADC_temp;
      }
    }      
  }
  
  //计算平均值并去除个位
  for(int i=0;i<3;i++)
  {
    ADC_ave[i]=(ADC_get[i][1] + ADC_get[i][2] + ADC_get[i][3])/3;
    ADC_ave[i]=ADC_ave[i]/10*10;
  }    
}

/*--------------------------------------------------------------------------------
*  @brief      电感数据的处理
*  @param      void
*  @since      v1.0
*  @note       对电感数据进行归一化并采用 开方差比和 处理，得出差值进入pid
--------------------------------------------------------------------------------*/
void normalize(void)
{
  //归一化
  ADC_one_and_inverse[0] = 100*(ADC_ave[0]-ADC_0_MIN)/(ADC_0_MAX-ADC_0_MIN);
  ADC_one_and_inverse[1] = 100*(ADC_ave[1]-ADC_1_MIN)/(ADC_1_MAX-ADC_1_MIN);
  ADC_one_and_inverse[2] = 100*(ADC_ave[2]-ADC_2_MIN)/(ADC_2_MAX-ADC_2_MIN);
  //限幅
  for(int i=0;i<3;i++)
  {
    if(ADC_one_and_inverse[i]>99)
      ADC_one_and_inverse[i]=100;
    if(ADC_one_and_inverse[i]<2)
      ADC_one_and_inverse[i]=1;
  }
  //开方差比和
  Road_err = (int)(500*(InvSqrt((float)ADC_one_and_inverse[0]) - InvSqrt((float)ADC_one_and_inverse[2]))
                         /(ADC_one_and_inverse[0] + ADC_one_and_inverse[2]));
  //差值限幅
}

/*--------------------------------------------------------------------------------
*  @brief      电感道路元素判断
*  @param      void
*  @since      v1.0
*  @note       对电感归一化后的数据进行处理，主要是对环岛位置进行判断
--------------------------------------------------------------------------------*/


void Road_Cal()
{
  if(island_direction==0&&island_flag==0)
  {
    if((ADC_one_and_inverse[0]>65 && ADC_one_and_inverse[1]>50)||
       (118>=(ADC_one_and_inverse[0]+ADC_one_and_inverse[2]) && (ADC_one_and_inverse[0]+ADC_one_and_inverse[2])>105 && ADC_one_and_inverse[1]>87)||
       ((ADC_one_and_inverse[0]+ADC_one_and_inverse[2])>118 && ADC_one_and_inverse[1]>95)||
       (ADC_one_and_inverse[2]>65 && ADC_one_and_inverse[1]>50))
    {
      //island_flag = 1;
      //BEEP(4);
    }
  }
}