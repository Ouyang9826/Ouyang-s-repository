#include "IMAGE_MANIPULATION.h"

uint8 flag_stop = 0;                                   //ֹͣ�����Ǳ�־λ

uint8 image_little[image_row][image_col];              //���ڴ�����С���ͼ��

uint8 add_line[image_row];                             //���ڲ��ߵ�����

char  image_midline[image_row];                        //��������
uint8 image_bin[image_row][image_col];                 //��ֵ��ͼƬ  
uint8 edge_right[image_row],edge_left[image_row];      //�����ұ߽�����߽磬����Ѱ����ʼ������

uint8 start_row ;           //������ʼ��     �ɽ���Զ  59-0
uint8 start_col;            //������ʼ�е�   
uint8 end_row;              //�����յ���
uint8 end_col;              //�����յ���     

uint8 R_LostCount;          //�ұ߽綪ʧ��
uint8 L_LostCount;          //��߽綪ʧ��
uint8 R_CorrectCount;       //�ұ߽���ȷ��
uint8 L_CorrectCount;       //��߽���ȷ��

uint8 flag_lostdoubleline=0;//˫�߶��߱�־λ

uint8 R_Lost_Line[2][2];            //�����Ե���ߵ�����,��ά����
uint8 L_Lost_Line[2][2];

uint8 DivLine;             //��Ч��
uint8 StraightLine;        //��Чֱ������

uint8 Roundabout_L_flag=0; //������־λ 1�������� 0��������
uint8 Roundabout_R_flag=0; //������־λ

uint8 Roundabout_L_G_flag=0; //������־λ
uint8 Roundabout_L_O_flag=0;
uint8 Roundabout_R_G_flag=0;
uint8 Roundabout_R_O_flag=0;

uint8 state_flag=0;          //����debug
uint8 debug=0;               //����debug

short signed int OLS_X[image_row];    //������С���˷�
short signed int OLS_Y[image_row];    //������С���˷�

float OLS_K = 0;     //б��
float OLS_R = 0;     //�ؾ�

float Straight_L_K;  //ֱ����߽�б��
float Straight_R_K;  //ֱ���ұ߽�б��

float AngleError_Now;                     //ͼ���ֵ

float Costant_K = 23;
float Costant_R = 7/7;//Ԥ��ǰհ


signed short int search_hang;          //�����ұ߽�


/*--------------------------------------------------------------------------------
*  @brief      ��ͼƬ������С����
*  @since      v1.0
*  @mote       ����ֱ����������ͷ�ֱ��ʵõ���ͼƬ����188*120�ü������ģ��ʽ�����С����
--------------------------------------------------------------------------------*/
void image_reduce_size()
{
  int i,j;
  for(i=0;i<image_row;i++)
    for(j=0;j<image_col;j++)
    {
      image_little[i][j] = image[i*2][j*2+1];
    }
}
/*--------------------------------------------------------------------------------
*  @brief      ���Ҷ�ͼ���ɶ�ֵ��ͼƬ
*  @param      *p1         �Ҷ�ͼƬ��ַ
               threshold           ��ֵ
*  @since      v1.0
*  @Sample usage:    get_img_bin(image[0], threshold)
*  @note       ʹ��ǰ�����ô�򷨼�����ֵ
--------------------------------------------------------------------------------*/
void get_img_bin(uint8 *p,uint8 threshold)
{
  
  int i,j;
  uint16 temp;
  
  for (i=0; i<image_row ; i++)
    for(j=0; j<image_col ; j++)
    {
      temp = *(p + j + i * image_col);//��ȡ���ص�
      if(temp>threshold)  image_bin[i][j] = 1;
      else                image_bin[i][j] = 0;
    }
  
}
/*--------------------------------------------------------------------------------
*  @brief      �õ�ֱ��������������б��
*  @since      v2.0
*  @mote       
--------------------------------------------------------------------------------*/
void Get_Straight_K()
{
  char k=0;
  for(char x=DivLine;x <= image_row-1;x++)//��
  {
    if(edge_left[x] == 0)continue;
    OLS_X[k]=edge_left[x];
    OLS_Y[k]=-x;
    k++;
  }
  OLS(OLS_X,OLS_Y,k);
  Straight_L_K=OLS_K;
  k=0;
  for(char x=DivLine;x <= image_row-1;x++)//��
  {
    if(edge_right[x] == image_col-1)continue;
    OLS_X[k]=edge_right[x];
    OLS_Y[k]=-x;
    k++;
  }
  OLS(OLS_X,OLS_Y,k);
  Straight_R_K=OLS_K; 
}

/*--------------------------------------------------------------------------------
*  @brief      �õ���������(����������)
*  @since      v1.0
*  @mote       ���鴢����flash�У�����Key1ʱ����
--------------------------------------------------------------------------------*/
void add_line_init()
{
  int i;
  for(i=0;i<60;i++)
  {
  add_line[i]=flash_read(10,i,uint8);
  }
}
void Get_addline()
{
  int i;
  if(DivLine <= 9 && L_LostCount <=10 && R_LostCount <= 10)//�����Ч������50 �� ���ұ߽綪��������10
  {
    //add_line[59]=1;//���п϶����ò��ߣ���������Ϊһ����־
    for(i=59;i >= 0;i--)
    {
      add_line[i] = (edge_right[i] - edge_left[i])/2;
    }
    FLASH_GetSectorSize();//flash��ʼ��
    FLASH_EraseSector(10);//��������
    if( FLASH_WriteSector(10,(const uint8 *)add_line,60,0) );//��add_lineд��flash
    {
      BEEP(4);
      systick_delay_ms(4);
      BEEP(4);              //˵��д��ɹ�
    }
  }
}



/*---------------------------------------------------------------------------------
*  @brief      Get_Midline����
*  @since      v1.0
*  @note       ��������
˼·��1.����Ѱ����ʼ�����ұ߽�
2.����ʼ�б߽�Ļ����ϣ�������λ�ü��ټ������ص㣬���������ұ߽�
���������ڵ�Ƭ�����ټ�����

�ο�hou@text�Ĵ���
---------------------------------------------------------------------------------*/
void Get_Midline()
{
  
  signed short int ROW_flag, COL_flag;       //�б�־���б�־
  signed short int Left_Range, Right_Range;  //����ƽ��֮��Ѱ�ұ߽��


  R_LostCount       = 0;          //�ұ߽綪ʧ��
  L_LostCount       = 0;          //��߽綪ʧ��
  R_CorrectCount    = 0;          //�ұ߽���ȷ��
  L_CorrectCount    = 0;          //��߽���ȷ��

  start_row = image_row - 1;                //������ʼ��   ������
  start_col = image_col / 2;                //������ʼ�е� ������
  end_row   = 0  ;                          //�����յ���
  end_col   = image_col - 1;                //�����յ���
  
  
  
  /*----------------------------Ѱ����ʼ���ұ߽�-------------------------------*/
  
  for(ROW_flag = start_row , COL_flag = start_col; COL_flag <= end_col; ++COL_flag)
  {
    if(image_bin[ROW_flag][COL_flag] == 0)
    {
      edge_right[start_row] = COL_flag;
      break;
    }
  }
  if(COL_flag > end_col)
  {
    edge_right[start_row] = end_col ;       //��Чֵ 93
    R_LostCount++;
  } 
  else R_CorrectCount++;
  
  /*--------------------------Ѱ����ʼ����߽�---------------------------------*/
  
  for(ROW_flag = start_row , COL_flag = start_col; COL_flag >= 0      ; --COL_flag)
  {                                                               
    if(image_bin[ROW_flag][COL_flag] == 0)
    {
      edge_left[start_row]  = COL_flag;
      break;
    }
  }                                                                  
  if(COL_flag < 0      )
  {
    edge_left[start_row] = 0        ;        //��Чֵ 0
    L_LostCount++;
  }  
  else L_CorrectCount++;
  
  /*-------------------------������ʼ���е�-----------------------------------*/
  
  image_midline[start_row] = (edge_right[start_row] + edge_left[start_row])/2;
  
  
  
  
  /*-----------------------����ʼ�еĻ�����Ѱ��ʣ�µ�����---------------------*/
  
  for(ROW_flag = start_row - 1; ROW_flag >= 0; --ROW_flag)  //�ɽ���Զ��
  {
    
    //��
    Left_Range = edge_left[ROW_flag + 1] + 4;//����һ���������4���㿪ʼѰ�ң�������ֵ����̫���Ӱ��Զ��������
    while((!image_bin[ROW_flag][Left_Range]) && (Left_Range<93)) 
    {
      Left_Range += 4;
    }
    if(Left_Range > image_col-1) Left_Range = 0;   //��Чֵ  0
    for(COL_flag = Left_Range ;COL_flag >= 0; --COL_flag)
    {
      if(image_bin[ROW_flag][COL_flag] == 0)
      {
        edge_left[ROW_flag] = COL_flag;      
        break;
      }
    }
    if(COL_flag < 0)     //�ж��Ƿ���˱߽�
    {
      edge_left[ROW_flag] = 0;               //��Чֵ   0
      L_LostCount++;
    }
    else
    {
      L_CorrectCount++;
    }
    
    //��
    Right_Range = edge_right[ROW_flag + 1] - 4;//����һ���������4���㿪ʼѰ��
    while((!image_bin[ROW_flag][Right_Range]) && (Right_Range > 0))
    {
      Right_Range -= 4;
    }
    if(Right_Range < 0) Left_Range = 93;   //��Чֵ   93
    for(COL_flag = Right_Range ;COL_flag <= end_col; ++COL_flag)
    {
      if(image_bin[ROW_flag][COL_flag] == 0)
      {
        edge_right[ROW_flag] = COL_flag;
        break;
      }
    }
    if(COL_flag > end_col)//�ж��Ƿ���˱߽�
    {
      edge_right[ROW_flag] = end_col;        //��Чֵ 93
      R_LostCount++;
    }
    else
    {
      R_CorrectCount++;
    }
    
 
    
    /*-------------�����������ʱֹͣ�����ͼƬ�Ĵ���Ѱ����Ч��--------------*/
    //������㣺1�����������ͷ��һ�� �� 2�������ұ߽翿��̫��  ����  ���߽�̫����!!!!!  ����  �Ҳ�߽�̫��������������
    if(ROW_flag <= start_row 
        && ((edge_right[ROW_flag+1] - edge_left[ROW_flag+1] < 10)  //�˴��� 10 ���Ը�����Ҫ����
          ||(edge_left[ROW_flag+1] > image_col-3)
            ||(edge_right[ROW_flag+1] < 2)))
    {
      DivLine = ROW_flag;//ȷ���÷�ͼ����Ч�У��������������б�ʡ��ؾ�
      break;
    }
    else 
      DivLine=0;//�������ֱ����Զ��һ�ж�����Ч��
    
  }
 
  
  /*---------------------ȷ����Ե��������������ж���������Ԫ��---------------------*/ 
  //V2.0   //ע������������ͣ�
  for(char i = start_row , s = 0 , e = 1 ,  L = 0; i >= DivLine; i--)//�ɽ���Զ ע��start_row(59)��DivLine��ʾ�����ɽ���Զ
  {                                                                     //s����־λ e�յ��־λ (1�ҵ� 0δ�ҵ�) L:�α�־
    if((s != 1) && (edge_left[i] == 0) && (edge_left[i + 1] == 0) && (edge_left[i + 2] == 0))//��
    {
      L_Lost_Line[L][0] = i + 2; 
      s = 1; //��һ������ҵ� 
      e = 0; //��ʼ���յ�
    }
    if((e != 1) && (edge_left[i] != 0) && (edge_left[i + 1] != 0) && (edge_left[i + 2] == 0))
    {
      L_Lost_Line[L][1] = i + 2;
      s = 0;//Ѱ����һ����㣬����еĻ�
      e = 1;//�յ��ҵ�
      L +=1;
    }
    if((s==0)&&(e==1)&&(i==DivLine))//���һֱû�ж��߻��߲����ڵڶ��ζ���
    {
      if(L == 0) //һֱû�ж���
      {
        L_Lost_Line[0][0] = 0;L_Lost_Line[0][1] = 0;L_Lost_Line[1][0] = 0;L_Lost_Line[1][1] = 0;//����Чλ
      }
      if(L == 1) //�ڶ��ζ��߲�����
      {
        L_Lost_Line[1][0] = 0;L_Lost_Line[1][1] = 0;
      }
    }
    if((i==DivLine)&&(s==1)&&(e==0))//���ҵ��������ȷ�Ҳ��������յ�
    {
      L_Lost_Line[L][1] = DivLine;  
    }
  }
  for(char i = start_row , s = 0 , e = 1 ,  L = 0; i >= DivLine; i--)//�ɽ���Զ ע��start_row(59)��DivLine��ʾ�����ɽ���Զ
  {                                                                     //s����־λ e�յ��־λ (1�ҵ� 0δ�ҵ�) L:�α�־
    if((s != 1) && (edge_right[i] == image_col-1) && (edge_right[i + 1] == image_col-1) && (edge_right[i + 2] == image_col-1))//��
    {
      R_Lost_Line[L][0] = i + 2; 
      s = 1; //��һ������ҵ� 
      e = 0; //��ʼ���յ�
    }
    if((e != 1) && (edge_right[i] != image_col-1) && (edge_right[i + 1] != image_col-1) && (edge_right[i + 2] == image_col-1))
    {
      R_Lost_Line[L][1] = i + 2;
      s = 0;//Ѱ����һ����㣬����еĻ�
      e = 1;//�յ��ҵ�
      L +=1;
    }
    if((s==0)&&(e==1)&&(i==DivLine))//���һֱû�ж��߻��߲����ڵڶ��ζ���
    {
      if(L == 0) //һֱû�ж���
      {
        R_Lost_Line[0][0] = 0;R_Lost_Line[0][1] = 0;R_Lost_Line[1][0] = 0;R_Lost_Line[1][1] = 0;//����Чλ
      }
      if(L == 1) //�ڶ��ζ��߲�����
      {
        R_Lost_Line[1][0] = 0;R_Lost_Line[1][1] = 0;
      }
    }
    if((i==DivLine)&&(s==1)&&(e==0))//���ҵ��������ȷ�Ҳ��������յ�
    {
      R_Lost_Line[L][1] = DivLine;  
    }
  }
 
//  //�����ж���������Ԫ�صı�־λ
//  //1.���߶������  2.˫�߶���
//  //˫�߶������
//  if((R_Lost_Line[0][0] >= 4) && ( L_Lost_Line[0][0] >= 4))
//  {
//    if(((R_Lost_Line[0][0] >= L_Lost_Line[0][0]) && (R_Lost_Line[0][1] <= L_Lost_Line[0][0])) ||
//       ((L_Lost_Line[0][0] >= R_Lost_Line[0][0]) && (L_Lost_Line[0][1] <= R_Lost_Line[0][0])))
//      flag_lostdoubleline = 1;
//    else flag_lostdoubleline=0;
//  }
//  else flag_lostdoubleline=0;
                                                            
    
  
  /*----------------------------������Чֱ����Χ------------------------------*/
  
  for(short int E_hang = image_row - 1; E_hang >= 0; E_hang--)
  {
    if((image_bin[E_hang][image_col/2 -2] == 0)                   //1Ϊ�ף�0Ϊ��
       &&(image_bin[E_hang][image_col/2 - 1] == 0)
         &&(image_bin[E_hang][image_col/2    ] == 0)
           &&(image_bin[E_hang][image_col/2 + 1] == 0)
             &&(image_bin[E_hang][image_col/2 + 2] == 0))
    {
      StraightLine = E_hang;
      break;
    }
    else StraightLine = 0;
  }
        
        
  
  /*-----------------------------��������-------------------------------------*/
  //���滹����в���
  for(char ab = image_row; ab > DivLine; --ab)
  {
    image_midline[ab] = (edge_right[ab] + edge_left[ab])/2;
    //image_bin[ab][image_midline[ab]] = 0;
   }
   
  /*------------------------------���߳���------------------------------------*/
  //V3.0
  //��ͨ����
  if((L_Lost_Line[0][0] - L_Lost_Line[0][1])>0 ) //�ж��Ƿ���Ҫ����
  {
    //��ಹ��     
    for(ROW_flag = L_Lost_Line[0][0]; ROW_flag >= L_Lost_Line[0][1] ;ROW_flag--)
    {
      if(edge_right[ROW_flag] != image_col - 1)
      {
        image_midline[ROW_flag] = edge_right[ROW_flag] - add_line[ROW_flag];
      }
    }
    
  } 
  if((R_Lost_Line[0][0] - R_Lost_Line[0][1])>0 )
  {
    //�Ҳಹ��
    for(ROW_flag = R_Lost_Line[0][0]; ROW_flag >= R_Lost_Line[0][1] ;ROW_flag--)
    {
      if(edge_left[ROW_flag] != 0)
      {
        image_midline[ROW_flag] = edge_left[ROW_flag] + add_line[ROW_flag];
      }
    }
  }
//  //��������  ֱ�Ž����� б�Ž�����֪����ô����   ͻȻ�о��е�ɵ�ƣ�ͻȻ�о������������岻�� 
//  //��ʵ���廹�ǱȽϴ��
//  //�������ұ� �� �Ҳ�������ߣ����Ϊֱ��
//  Get_Straight_K();
//  if((R_LostCount >= 35) && (Straight_L_K <= 1.3) && (Straight_L_K >= 1.1) && (StraightLine <= 5))
//    for(char i = 0;i <= image_row -1; i--)
//    {
//      image_midline[i] = edge_left[i] + add_line[i];//BEEP(4);
//    }
//  //��
//  if((L_LostCount >= 35) && (Straight_R_K <= -1.1) && (Straight_L_K >= -1.3) && (StraightLine <= 5))
//    for(char i = 0;i <= image_row -1; i--)
//    {
//      image_midline[i] = edge_right[i] - add_line[i];//BEEP(4);
//    }
  
  /*-------------------------------��������-----------------------------------*/    
  //V5.0
  //��ԭ���Ļ����ϸĽ� ԭ���ģ�һ��Ϊֱ����һ�ߴ�������
  //                  �Ľ��ģ�����һ��Ϊֱ����һ�ߴ������ߣ������Դ������ߵ���һ�߼���б��
  //                �¸Ľ��ģ�б�ʿ��ܻ�����ĳЩԭ����ţ��ʲ�����б���ˣ���Ϊ�ҽ�
  //������
  Get_Straight_K();
          
//  if((R_LostCount >= 40) && (Straight_L_K <= 1.3) && (Straight_L_K >= 1.1) && (R_Lost_Line[1][0]==0) && flag_lostdoubleline!=1)//�һ�����
//  {
//    char flag = 0;
//    for(char i = DivLine; i <= 30;i++)
//    {
//      if(edge_right[i+1]-edge_right[i]>=15)flag++;
//    }
//    if(flag >= 1)//�һ�����
//    {      
//      Roundabout_R_flag+=1;//ֻ��һ��
//      if(Roundabout_R_flag==1)
//      {
//        BEEP(4);
//        S3010_Control(790);
//        cesu_right_count=0;
//        while(cesu_right_count <= 2200);
//        Roundabout_R_G_flag+=1;
//        Roundabout_R_O_flag+=1;
//      }
//    }
//  }  
//  if((L_LostCount >= 30) && (Straight_R_K <= -1.1) && (Straight_R_K >= -1.3))//�󻷵���
//  {
//    char flag = 0;
//    for(char i = DivLine; i <= 30;i++)
//    {
//      if(edge_left[i] - edge_left[i+1]>=15)
//        if(edge_left[i+10] == 0)flag++;
//    }
//    if(flag >= 1)//�󻷵���
//    {
//      Roundabout_L_flag+=1;
//      if(Roundabout_L_flag==1)//ֻ��һ��
//      {
//        BEEP(4);
//        S3010_Control(910);
//        cesu_left_count=0;
//        while(cesu_left_count <= 2200);
//        Roundabout_L_G_flag+=1;
//        Roundabout_L_O_flag+=1;
//      }
//    }
//  } 
  //��
  if(L_LostCount >= 30)  //��߽��������
  {
    char i,flag = 0;
    for(i = DivLine; i <= 30;i++)
    {
      if(edge_left[i] - edge_left[i+3]>=15)//���ֶϵ�
        if(edge_left[i+8] == 0)
        {
          flag++;break;
        }
    }
    if(flag >= 1)
    {   state_flag=1; //debugʱ�õ�
//      char k=0;     //б�ʲ�̫����
//      for(char x = i;x <= image_row-1;x++)//һ���ұ߽��б��
//      {
//        OLS_X[k]=edge_right[x];
//        OLS_Y[k]=-x;
//        k++;
//      }debug=i;
//      OLS(OLS_X,OLS_Y,k);
//      Straight_R_K=OLS_K;
      if((i <= 18) && (i >= 14) //���ݽǵ�λ��ȷ���뻷����λ��
         && (ADC_one_and_inverse[0]>65||ADC_one_and_inverse[1]>65||ADC_one_and_inverse[2]>65))//��Ÿ�����⻷��
      {
        Roundabout_L_flag+=1;
        if(Roundabout_L_flag==1)//ֻ��һ��
        {
          BEEP(4);
          S3010_Control(910);
          Road_HD=2200;
          while(Road_HD <= 0);
          Roundabout_L_G_flag+=1;
          Roundabout_L_O_flag+=1;
        }
      }
    }     else state_flag=0;
  }
  //��
  if(R_LostCount >= 30)  //�ұ߽��������
  {
    char i,flag = 0;
    for(i = DivLine; i <= 30;i++)
    {
      if(edge_right[i+3] - edge_right[i]>=15)
        if(edge_right[i+8] == image_col-1)
        {
          flag++;break;
        }
    }
    if(flag >= 1)
    {
//      char k=0;//BEEP(4);
//      for(char x = i;x <= image_row-1;x++)//һ����߽��б��
//      {
//        OLS_X[k]=edge_left[x];
//        OLS_Y[k]=-x;
//        k++;
//      }
//      OLS(OLS_X,OLS_Y,k);
//      Straight_L_K=OLS_K;
      if((i <= 18) && (i >= 14) //���ݽǵ�λ��ȷ���뻷����λ��
         && (ADC_one_and_inverse[0]>65||ADC_one_and_inverse[1]>65||ADC_one_and_inverse[2]>65))//��Ÿ�����⻷��
      {
        Roundabout_R_flag+=1;
        if(Roundabout_R_flag==1)//ֻ��һ��
        {
          BEEP(4);
          S3010_Control(910);
          Road_HD=2200;
          while(Road_HD <= 0);
          Roundabout_R_G_flag+=1;
          Roundabout_R_O_flag+=1;
        }
      }
    }     else state_flag=0;
    }
  }
  
  
  //���������� 
  //��
  if((Roundabout_L_O_flag==1) && (L_Lost_Line[0][0] == 59) && (R_Lost_Line[0][0] == 59) && (L_Lost_Line[0][0] - L_Lost_Line[0][1] >= 10) &&(R_Lost_Line[0][0] - R_Lost_Line[0][1] >= 10))
  {
    BEEP(4);
    S3010_Control(940);
    flag_stop=1;//����ԭ�����
    Road_HD=2200;
    while(Road_HD <= 0);
    flag_stop=0;
    Roundabout_L_O_flag+=1;BEEP(4);
  }
  //��
  if((Roundabout_R_O_flag==1) && (L_Lost_Line[0][0] == 59) && (R_Lost_Line[0][0] == 59) && (L_Lost_Line[0][0] - L_Lost_Line[0][1] >= 10) &&(R_Lost_Line[0][0] - R_Lost_Line[0][1] >= 10))
  {
    BEEP(4);
    S3010_Control(800);
    flag_stop=1;//����ԭ�����
    cesu_left_count=0;
    while(cesu_left_count <= 2200);
    flag_stop=0;
    Roundabout_R_O_flag+=1;BEEP(4);
  }
//  if(( Straight_R_K <= -1.2) && (Roundabout_L_O_flag==2))
//  {
//    BEEP(4);
//    flag_stop=0;
//    Roundabout_L_O_flag+=1;
//  }
    
    
//  //һ����������µĳ�����˼·    
//  if((Roundabout_L_O_flag==1) && (cesu_left_count >= 12000))//������ //��
//  {
//    S3010_Control(S3010_duty);
//    flag_stop=1;//����ԭ�����
//    if(Roundabout_L_flag == 2)
//    {
//      BEEP(4);
//      flag_stop=0;//�ָ����
//      Roundabout_L_O_flag +=1;
//    } 
//  }
//  if((Roundabout_R_O_flag==1) && (cesu_right_count >= 12000))//������ //��
//  {
//    flag_stop=1;
//    S3010_Control(S3010_duty);
//    if(Roundabout_R_flag == 2)
//    {
//      BEEP(4);
//      flag_stop=0;//�ָ����
//      Roundabout_R_O_flag +=1;
//    } 
//  }
  
  /*-------------------------------��·����-----------------------------------*/    
  //if((StraightLine < )&&())
  
  
  
  
  
        
  /*------------------------------������ƫת�Ƕ�----------------------------*/
  if(DivLine < image_row - 5)
  {
    uint8 k = 0;
    for(ROW_flag = start_row; ROW_flag >= DivLine; ROW_flag--)
    {
      OLS_X[k] = image_row - 1 - ROW_flag;
      OLS_Y[k] = image_midline[ROW_flag];
      
      k++;
    }
    OLS(OLS_X,OLS_Y,k);
    
    //����һ��AngleError_Now���������ͼ��ѡ�����
    AngleError_Now = (0.00 - OLS_K) * Costant_K  +  (46.5 - OLS_R) * Costant_R;//0.00   46.5
//    if(Roundabout_L_flag >= 1)
//      AngleError_Now = (0.00 - OLS_K) * Costant_K  +  (50 - OLS_R) * Costant_R;
  }
  
  
}


















