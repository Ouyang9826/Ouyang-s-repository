#include "data_process.h"

uint16 AD_MAX[3]={3600,3600,3600};
uint16 ADC_ave[3];
uint16 ADC_one_and_inverse[3]={0};//������Ź�һ����ֵ

int16 Road_err=0;

uint8 island_direction;//�������� 0Ϊδ�ж� 1Ϊ���󻷵��� 2Ϊ���һ�����
uint8 island_flag;//������־λ 0Ϊδ�ж�

/*--------------------------------------------------------------------------------
*  @brief      ���ż��㺯�� InvSqrt()
*  @param      x         ��������
*  @since      v1.0
*  @Sample usage:    get_img_bin(image[0], threshold)
*  @note       �����ٶȱ�ֱ��ʹ��math����ĺ�������
--------------------------------------------------------------------------------*/
float InvSqrt(float x)//����
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
*  @brief      ��ȡADC�Ĳɼ�ֵ
*  @param      void
*  @since      v1.0
*  @note       �ɼ�adc��ֵ������adc�Ŀ�ֱ�ӽ��вɼ�adcֵ��Ҳ�������ֶ������ݽ��д���
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
  
  //ѡ������ ��С�����������
  
//  for(int i=0;i<3;i++)     //6�����
//  {
//    for(int j=0;j<4;j++)                 //�����������  ֮ǰ��5������������ˣ�MMP����
//    {
//      for(int k=0;k<4-j;k++)
//      {
//        if(ADC_get[i][k] > ADC_get[i][k+1])     //ǰ��ıȺ���Ĵ�  ����н���
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
  
  //����ƽ��ֵ��ȥ����λ
  for(int i=0;i<3;i++)
  {
    ADC_ave[i]=(ADC_get[i][1] + ADC_get[i][2] + ADC_get[i][3])/3;
    ADC_ave[i]=ADC_ave[i]/10*10;
  }    
}

/*--------------------------------------------------------------------------------
*  @brief      ������ݵĴ���
*  @param      void
*  @since      v1.0
*  @note       �Ե�����ݽ��й�һ�������� ������Ⱥ� �����ó���ֵ����pid
--------------------------------------------------------------------------------*/
void normalize(void)
{
  //��һ��
  ADC_one_and_inverse[0] = 100*(ADC_ave[0]-ADC_0_MIN)/(ADC_0_MAX-ADC_0_MIN);
  ADC_one_and_inverse[1] = 100*(ADC_ave[1]-ADC_1_MIN)/(ADC_1_MAX-ADC_1_MIN);
  ADC_one_and_inverse[2] = 100*(ADC_ave[2]-ADC_2_MIN)/(ADC_2_MAX-ADC_2_MIN);
  //�޷�
  for(int i=0;i<3;i++)
  {
    if(ADC_one_and_inverse[i]>99)
      ADC_one_and_inverse[i]=100;
    if(ADC_one_and_inverse[i]<2)
      ADC_one_and_inverse[i]=1;
  }
  //������Ⱥ�
  Road_err = (int)(500*(InvSqrt((float)ADC_one_and_inverse[0]) - InvSqrt((float)ADC_one_and_inverse[2]))
                         /(ADC_one_and_inverse[0] + ADC_one_and_inverse[2]));
  //��ֵ�޷�
}

/*--------------------------------------------------------------------------------
*  @brief      ��е�·Ԫ���ж�
*  @param      void
*  @since      v1.0
*  @note       �Ե�й�һ��������ݽ��д�����Ҫ�ǶԻ���λ�ý����ж�
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