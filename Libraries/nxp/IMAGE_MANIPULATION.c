#include "IMAGE_MANIPULATION.h"

uint8 flag_stop = 0;                                   //停止舵机打角标志位

uint8 image_little[image_row][image_col];              //用于储存缩小后的图像

uint8 add_line[image_row];                             //用于补线的数组

char  image_midline[image_row];                        //中线数组
uint8 image_bin[image_row][image_col];                 //二值化图片  
uint8 edge_right[image_row],edge_left[image_row];      //定义右边界与左边界，用于寻找起始行中线

uint8 start_row ;           //定义起始行     由近到远  59-0
uint8 start_col;            //定义起始中点   
uint8 end_row;              //定义终点行
uint8 end_col;              //定义终点列     

uint8 R_LostCount;          //右边界丢失数
uint8 L_LostCount;          //左边界丢失数
uint8 R_CorrectCount;       //右边界正确数
uint8 L_CorrectCount;       //左边界正确数

uint8 flag_lostdoubleline=0;//双边丢线标志位

uint8 R_Lost_Line[2][2];            //储存边缘丢线的数据,二维数组
uint8 L_Lost_Line[2][2];

uint8 DivLine;             //有效行
uint8 StraightLine;        //有效直道行数

uint8 Roundabout_L_flag=0; //环岛标志位 1：环岛内 0：环岛外
uint8 Roundabout_R_flag=0; //环岛标志位

uint8 Roundabout_L_G_flag=0; //环岛标志位
uint8 Roundabout_L_O_flag=0;
uint8 Roundabout_R_G_flag=0;
uint8 Roundabout_R_O_flag=0;

uint8 state_flag=0;          //用于debug
uint8 debug=0;               //用于debug

short signed int OLS_X[image_row];    //用于最小二乘法
short signed int OLS_Y[image_row];    //用于最小二乘法

float OLS_K = 0;     //斜率
float OLS_R = 0;     //截距

float Straight_L_K;  //直道左边界斜率
float Straight_R_K;  //直道右边界斜率

float AngleError_Now;                     //图像差值

float Costant_K = 23;
float Costant_R = 7/7;//预设前瞻


signed short int search_hang;          //用于找边界


/*--------------------------------------------------------------------------------
*  @brief      对图片进行缩小处理
*  @since      v1.0
*  @mote       由于直接设置摄像头分辨率得到的图片是由188*120裁剪而来的，故进行缩小处理
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
*  @brief      将灰度图像变成二值化图片
*  @param      *p1         灰度图片地址
               threshold           阈值
*  @since      v1.0
*  @Sample usage:    get_img_bin(image[0], threshold)
*  @note       使用前首先用大津法计算阈值
--------------------------------------------------------------------------------*/
void get_img_bin(uint8 *p,uint8 threshold)
{
  
  int i,j;
  uint16 temp;
  
  for (i=0; i<image_row ; i++)
    for(j=0; j<image_col ; j++)
    {
      temp = *(p + j + i * image_col);//读取像素点
      if(temp>threshold)  image_bin[i][j] = 1;
      else                image_bin[i][j] = 0;
    }
  
}
/*--------------------------------------------------------------------------------
*  @brief      得到直道赛道左右两边斜率
*  @since      v2.0
*  @mote       
--------------------------------------------------------------------------------*/
void Get_Straight_K()
{
  char k=0;
  for(char x=DivLine;x <= image_row-1;x++)//左
  {
    if(edge_left[x] == 0)continue;
    OLS_X[k]=edge_left[x];
    OLS_Y[k]=-x;
    k++;
  }
  OLS(OLS_X,OLS_Y,k);
  Straight_L_K=OLS_K;
  k=0;
  for(char x=DivLine;x <= image_row-1;x++)//右
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
*  @brief      得到补线数组(半个赛道宽度)
*  @since      v1.0
*  @mote       数组储存于flash中，按下Key1时重置
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
  if(DivLine <= 9 && L_LostCount <=10 && R_LostCount <= 10)//如果有效行少于50 且 左右边界丢线数少于10
  {
    //add_line[59]=1;//首行肯定不用补线，因此这个作为一个标志
    for(i=59;i >= 0;i--)
    {
      add_line[i] = (edge_right[i] - edge_left[i])/2;
    }
    FLASH_GetSectorSize();//flash初始化
    FLASH_EraseSector(10);//擦除扇区
    if( FLASH_WriteSector(10,(const uint8 *)add_line,60,0) );//将add_line写入flash
    {
      BEEP(4);
      systick_delay_ms(4);
      BEEP(4);              //说明写入成功
    }
  }
}



/*---------------------------------------------------------------------------------
*  @brief      Get_Midline函数
*  @since      v1.0
*  @note       计算中线
思路：1.首先寻找起始行左右边界
2.在起始行边界的基础上，向中心位置减少几个像素点，再向左右找边界
这样有利于单片机减少计算量

参考hou@text的代码
---------------------------------------------------------------------------------*/
void Get_Midline()
{
  
  signed short int ROW_flag, COL_flag;       //行标志与列标志
  signed short int Left_Range, Right_Range;  //用于平移之后寻找边界点


  R_LostCount       = 0;          //右边界丢失数
  L_LostCount       = 0;          //左边界丢失数
  R_CorrectCount    = 0;          //右边界正确数
  L_CorrectCount    = 0;          //左边界正确数

  start_row = image_row - 1;                //定义起始行   待定义
  start_col = image_col / 2;                //定义起始中点 待定义
  end_row   = 0  ;                          //定义终点行
  end_col   = image_col - 1;                //定义终点列
  
  
  
  /*----------------------------寻找起始行右边界-------------------------------*/
  
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
    edge_right[start_row] = end_col ;       //无效值 93
    R_LostCount++;
  } 
  else R_CorrectCount++;
  
  /*--------------------------寻找起始行左边界---------------------------------*/
  
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
    edge_left[start_row] = 0        ;        //无效值 0
    L_LostCount++;
  }  
  else L_CorrectCount++;
  
  /*-------------------------计算起始行中点-----------------------------------*/
  
  image_midline[start_row] = (edge_right[start_row] + edge_left[start_row])/2;
  
  
  
  
  /*-----------------------在起始行的基础上寻找剩下的中线---------------------*/
  
  for(ROW_flag = start_row - 1; ROW_flag >= 0; --ROW_flag)  //由近到远找
  {
    
    //左
    Left_Range = edge_left[ROW_flag + 1] + 4;//从上一行左侧往左4个点开始寻找，如果这个值设置太大会影响远处找中线
    while((!image_bin[ROW_flag][Left_Range]) && (Left_Range<93)) 
    {
      Left_Range += 4;
    }
    if(Left_Range > image_col-1) Left_Range = 0;   //无效值  0
    for(COL_flag = Left_Range ;COL_flag >= 0; --COL_flag)
    {
      if(image_bin[ROW_flag][COL_flag] == 0)
      {
        edge_left[ROW_flag] = COL_flag;      
        break;
      }
    }
    if(COL_flag < 0)     //判断是否出了边界
    {
      edge_left[ROW_flag] = 0;               //无效值   0
      L_LostCount++;
    }
    else
    {
      L_CorrectCount++;
    }
    
    //右
    Right_Range = edge_right[ROW_flag + 1] - 4;//从上一行左侧往右4个点开始寻找
    while((!image_bin[ROW_flag][Right_Range]) && (Right_Range > 0))
    {
      Right_Range -= 4;
    }
    if(Right_Range < 0) Left_Range = 93;   //无效值   93
    for(COL_flag = Right_Range ;COL_flag <= end_col; ++COL_flag)
    {
      if(image_bin[ROW_flag][COL_flag] == 0)
      {
        edge_right[ROW_flag] = COL_flag;
        break;
      }
    }
    if(COL_flag > end_col)//判断是否出了边界
    {
      edge_right[ROW_flag] = end_col;        //无效值 93
      R_LostCount++;
    }
    else
    {
      R_CorrectCount++;
    }
    
 
    
    /*-------------满足以下情况时停止对这幅图片的处理并寻找有效行--------------*/
    //如果满足：1，不是最靠近车头的一行 且 2，（左右边界靠得太近  或者  左侧边界太靠右!!!!!  或者  右侧边界太靠左）则跳出找线
    if(ROW_flag <= start_row 
        && ((edge_right[ROW_flag+1] - edge_left[ROW_flag+1] < 10)  //此处的 10 可以根据需要来改
          ||(edge_left[ROW_flag+1] > image_col-3)
            ||(edge_right[ROW_flag+1] < 2)))
    {
      DivLine = ROW_flag;//确定该幅图像有效行，便于下面程序算斜率、截距
      break;
    }
    else 
      DivLine=0;//否则就是直到最远处一行都是有效的
    
  }
 
  
  /*---------------------确定边缘丢线情况，用于判断赛道各种元素---------------------*/ 
  //V2.0   //注意变量符号类型！
  for(char i = start_row , s = 0 , e = 1 ,  L = 0; i >= DivLine; i--)//由近到远 注意start_row(59)到DivLine表示的是由近到远
  {                                                                     //s起点标志位 e终点标志位 (1找到 0未找到) L:段标志
    if((s != 1) && (edge_left[i] == 0) && (edge_left[i + 1] == 0) && (edge_left[i + 2] == 0))//左
    {
      L_Lost_Line[L][0] = i + 2; 
      s = 1; //第一个起点找到 
      e = 0; //开始找终点
    }
    if((e != 1) && (edge_left[i] != 0) && (edge_left[i + 1] != 0) && (edge_left[i + 2] == 0))
    {
      L_Lost_Line[L][1] = i + 2;
      s = 0;//寻找下一个起点，如果有的话
      e = 1;//终点找到
      L +=1;
    }
    if((s==0)&&(e==1)&&(i==DivLine))//如果一直没有丢线或者不存在第二段丢线
    {
      if(L == 0) //一直没有丢线
      {
        L_Lost_Line[0][0] = 0;L_Lost_Line[0][1] = 0;L_Lost_Line[1][0] = 0;L_Lost_Line[1][1] = 0;//置无效位
      }
      if(L == 1) //第二段丢线不存在
      {
        L_Lost_Line[1][0] = 0;L_Lost_Line[1][1] = 0;
      }
    }
    if((i==DivLine)&&(s==1)&&(e==0))//能找到丢线起点确找不到丢线终点
    {
      L_Lost_Line[L][1] = DivLine;  
    }
  }
  for(char i = start_row , s = 0 , e = 1 ,  L = 0; i >= DivLine; i--)//由近到远 注意start_row(59)到DivLine表示的是由近到远
  {                                                                     //s起点标志位 e终点标志位 (1找到 0未找到) L:段标志
    if((s != 1) && (edge_right[i] == image_col-1) && (edge_right[i + 1] == image_col-1) && (edge_right[i + 2] == image_col-1))//右
    {
      R_Lost_Line[L][0] = i + 2; 
      s = 1; //第一个起点找到 
      e = 0; //开始找终点
    }
    if((e != 1) && (edge_right[i] != image_col-1) && (edge_right[i + 1] != image_col-1) && (edge_right[i + 2] == image_col-1))
    {
      R_Lost_Line[L][1] = i + 2;
      s = 0;//寻找下一个起点，如果有的话
      e = 1;//终点找到
      L +=1;
    }
    if((s==0)&&(e==1)&&(i==DivLine))//如果一直没有丢线或者不存在第二段丢线
    {
      if(L == 0) //一直没有丢线
      {
        R_Lost_Line[0][0] = 0;R_Lost_Line[0][1] = 0;R_Lost_Line[1][0] = 0;R_Lost_Line[1][1] = 0;//置无效位
      }
      if(L == 1) //第二段丢线不存在
      {
        R_Lost_Line[1][0] = 0;R_Lost_Line[1][1] = 0;
      }
    }
    if((i==DivLine)&&(s==1)&&(e==0))//能找到丢线起点确找不到丢线终点
    {
      R_Lost_Line[L][1] = DivLine;  
    }
  }
 
//  //用于判断赛道各种元素的标志位
//  //1.单边丢线情况  2.双边丢线
//  //双边丢线情况
//  if((R_Lost_Line[0][0] >= 4) && ( L_Lost_Line[0][0] >= 4))
//  {
//    if(((R_Lost_Line[0][0] >= L_Lost_Line[0][0]) && (R_Lost_Line[0][1] <= L_Lost_Line[0][0])) ||
//       ((L_Lost_Line[0][0] >= R_Lost_Line[0][0]) && (L_Lost_Line[0][1] <= R_Lost_Line[0][0])))
//      flag_lostdoubleline = 1;
//    else flag_lostdoubleline=0;
//  }
//  else flag_lostdoubleline=0;
                                                            
    
  
  /*----------------------------计算有效直道范围------------------------------*/
  
  for(short int E_hang = image_row - 1; E_hang >= 0; E_hang--)
  {
    if((image_bin[E_hang][image_col/2 -2] == 0)                   //1为白，0为黑
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
        
        
  
  /*-----------------------------计算中线-------------------------------------*/
  //后面还会进行补线
  for(char ab = image_row; ab > DivLine; --ab)
  {
    image_midline[ab] = (edge_right[ab] + edge_left[ab])/2;
    //image_bin[ab][image_midline[ab]] = 0;
   }
   
  /*------------------------------补线程序------------------------------------*/
  //V3.0
  //普通补线
  if((L_Lost_Line[0][0] - L_Lost_Line[0][1])>0 ) //判断是否需要补线
  {
    //左侧补线     
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
    //右侧补线
    for(ROW_flag = R_Lost_Line[0][0]; ROW_flag >= R_Lost_Line[0][1] ;ROW_flag--)
    {
      if(edge_left[ROW_flag] != 0)
      {
        image_midline[ROW_flag] = edge_left[ROW_flag] + add_line[ROW_flag];
      }
    }
  }
//  //环岛补线  直着进环岛 斜着进还不知道怎么处理   突然感觉有点傻逼，突然感觉环岛补线意义不大 
//  //其实意义还是比较大的
//  //环岛在右边 即 右侧大量丢线，左侧为直线
//  Get_Straight_K();
//  if((R_LostCount >= 35) && (Straight_L_K <= 1.3) && (Straight_L_K >= 1.1) && (StraightLine <= 5))
//    for(char i = 0;i <= image_row -1; i--)
//    {
//      image_midline[i] = edge_left[i] + add_line[i];//BEEP(4);
//    }
//  //左
//  if((L_LostCount >= 35) && (Straight_R_K <= -1.1) && (Straight_L_K >= -1.3) && (StraightLine <= 5))
//    for(char i = 0;i <= image_row -1; i--)
//    {
//      image_midline[i] = edge_right[i] - add_line[i];//BEEP(4);
//    }
  
  /*-------------------------------环岛程序-----------------------------------*/    
  //V5.0
  //再原来的基础上改进 原来的：一边为直道，一边大量丢线
  //                  改进的：照样一边为直道，一边大量丢线，不过对大量丢线的那一边计算斜率
  //                新改进的：斜率可能会由于某些原因干扰，故不计算斜率了，改为找角
  //进环岛
  Get_Straight_K();
          
//  if((R_LostCount >= 40) && (Straight_L_K <= 1.3) && (Straight_L_K >= 1.1) && (R_Lost_Line[1][0]==0) && flag_lostdoubleline!=1)//右环岛进
//  {
//    char flag = 0;
//    for(char i = DivLine; i <= 30;i++)
//    {
//      if(edge_right[i+1]-edge_right[i]>=15)flag++;
//    }
//    if(flag >= 1)//右环岛进
//    {      
//      Roundabout_R_flag+=1;//只进一次
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
//  if((L_LostCount >= 30) && (Straight_R_K <= -1.1) && (Straight_R_K >= -1.3))//左环岛进
//  {
//    char flag = 0;
//    for(char i = DivLine; i <= 30;i++)
//    {
//      if(edge_left[i] - edge_left[i+1]>=15)
//        if(edge_left[i+10] == 0)flag++;
//    }
//    if(flag >= 1)//左环岛进
//    {
//      Roundabout_L_flag+=1;
//      if(Roundabout_L_flag==1)//只进一次
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
  //左
  if(L_LostCount >= 30)  //左边界大量丢线
  {
    char i,flag = 0;
    for(i = DivLine; i <= 30;i++)
    {
      if(edge_left[i] - edge_left[i+3]>=15)//出现断点
        if(edge_left[i+8] == 0)
        {
          flag++;break;
        }
    }
    if(flag >= 1)
    {   state_flag=1; //debug时用到
//      char k=0;     //斜率不太好用
//      for(char x = i;x <= image_row-1;x++)//一段右边界的斜率
//      {
//        OLS_X[k]=edge_right[x];
//        OLS_Y[k]=-x;
//        k++;
//      }debug=i;
//      OLS(OLS_X,OLS_Y,k);
//      Straight_R_K=OLS_K;
      if((i <= 18) && (i >= 14) //根据角的位置确定入环岛的位置
         && (ADC_one_and_inverse[0]>65||ADC_one_and_inverse[1]>65||ADC_one_and_inverse[2]>65))//电磁辅助检测环岛
      {
        Roundabout_L_flag+=1;
        if(Roundabout_L_flag==1)//只进一次
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
  //右
  if(R_LostCount >= 30)  //右边界大量丢线
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
//      for(char x = i;x <= image_row-1;x++)//一段左边界的斜率
//      {
//        OLS_X[k]=edge_left[x];
//        OLS_Y[k]=-x;
//        k++;
//      }
//      OLS(OLS_X,OLS_Y,k);
//      Straight_L_K=OLS_K;
      if((i <= 18) && (i >= 14) //根据角的位置确定入环岛的位置
         && (ADC_one_and_inverse[0]>65||ADC_one_and_inverse[1]>65||ADC_one_and_inverse[2]>65))//电磁辅助检测环岛
      {
        Roundabout_R_flag+=1;
        if(Roundabout_R_flag==1)//只进一次
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
  
  
  //出环岛程序 
  //左
  if((Roundabout_L_O_flag==1) && (L_Lost_Line[0][0] == 59) && (R_Lost_Line[0][0] == 59) && (L_Lost_Line[0][0] - L_Lost_Line[0][1] >= 10) &&(R_Lost_Line[0][0] - R_Lost_Line[0][1] >= 10))
  {
    BEEP(4);
    S3010_Control(940);
    flag_stop=1;//保持原来打角
    Road_HD=2200;
    while(Road_HD <= 0);
    flag_stop=0;
    Roundabout_L_O_flag+=1;BEEP(4);
  }
  //右
  if((Roundabout_R_O_flag==1) && (L_Lost_Line[0][0] == 59) && (R_Lost_Line[0][0] == 59) && (L_Lost_Line[0][0] - L_Lost_Line[0][1] >= 10) &&(R_Lost_Line[0][0] - R_Lost_Line[0][1] >= 10))
  {
    BEEP(4);
    S3010_Control(800);
    flag_stop=1;//保持原来打角
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
    
    
//  //一种理想情况下的出环岛思路    
//  if((Roundabout_L_O_flag==1) && (cesu_left_count >= 12000))//出环岛 //左
//  {
//    S3010_Control(S3010_duty);
//    flag_stop=1;//保持原来打角
//    if(Roundabout_L_flag == 2)
//    {
//      BEEP(4);
//      flag_stop=0;//恢复打角
//      Roundabout_L_O_flag +=1;
//    } 
//  }
//  if((Roundabout_R_O_flag==1) && (cesu_right_count >= 12000))//出环岛 //右
//  {
//    flag_stop=1;
//    S3010_Control(S3010_duty);
//    if(Roundabout_R_flag == 2)
//    {
//      BEEP(4);
//      flag_stop=0;//恢复打角
//      Roundabout_R_O_flag +=1;
//    } 
//  }
  
  /*-------------------------------断路程序-----------------------------------*/    
  //if((StraightLine < )&&())
  
  
  
  
  
        
  /*------------------------------计算舵机偏转角度----------------------------*/
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
    
    //计算一次AngleError_Now，用于这幅图像选择参数
    AngleError_Now = (0.00 - OLS_K) * Costant_K  +  (46.5 - OLS_R) * Costant_R;//0.00   46.5
//    if(Roundabout_L_flag >= 1)
//      AngleError_Now = (0.00 - OLS_K) * Costant_K  +  (50 - OLS_R) * Costant_R;
  }
  
  
}


















