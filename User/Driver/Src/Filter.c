/* Copyright (c)  2019-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
/*----------------------------------------------------------------------------------------------------------------------/
                开源并不等于免费，先驱者的历史已经证明，在国内这个环境下，毫无收益的开源，单靠坊间个人爱好者，自发地参与项
								目完善的方式行不通，好的开源项目需要请专职人员做好售后服务、手把手教学、统计用户反馈需求、在实践中完成对产
								品的一次次完善与迭代升级。经过综合考虑，无名飞控开源代码中，程序仅保留公司正版激活功能代码，版本激活无实际
								功能，属于公司产品常规出厂操作，不影响客户学习，其余代码全部开放给客户学习，客户移植和二次开发代码请保留代
								码版权。
-----------------------------------------------------------------------------------------------------------------------/
*               本程序只供购买者学习使用，版权著作权属于无名科创团队，无名科创团队将飞控程序源码提供给购买者，
*               购买者要为无名科创团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载， 
*               更不能以此销售牟利，如发现上述行为，无名科创团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------
*                                                 为什么选择无名创新？
*                                         感动人心价格厚道，最靠谱的开源飞控；
*                                         国内业界良心之作，最精致的售后服务；
*                                         追求极致用户体验，高效进阶学习之路；
*                                         萌新不再孤单求索，合理把握开源尺度；
*                                         响应国家扶贫号召，促进教育体制公平；
*                                         新时代奋斗最出彩，建人类命运共同体。 
-----------------------------------------------------------------------------------------------------------------------
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               无名科创开源飞控QQ群：2号群465082224、1号群540707961（人员已满）
*               CSDN博客：http://blog.csdn.net/u011992534
*               优酷ID：NamelessCotrun无名小哥
*               B站教学视频：https://space.bilibili.com/67803559/#/video
*               客户使用心得、改进意见征集贴：http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               淘宝店铺：https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               百度贴吧:无名科创开源飞控
*               公司官网:www.nameless.tech
*               无名创新国内首款TI开源飞控设计初衷、知乎专栏:https://zhuanlan.zhihu.com/p/54471146
*               修改日期:2020/03/28                    
*               版本：筑梦者——GankerPilot_V2.0
*               版权所有，盗版必究。
*               Copyright(C) 2019-2025 武汉无名创新科技有限公司 
*               All rights reserved
-----------------------------------------------------------------------------------------------------------------------
*               重要提示：
*               正常淘宝咸鱼转手的飞控、赠送朋友、传给学弟的都可以进售后群学习交流，
*               不得直接在网上销售无名创新资料，无名创新代码有声明版权，他人不得将
*               资料代码传网上供他人下载，不得以谋利为目的销售资料代码，发现有此操
*               作者，公司会提前告知，请1天内及时处理，否则你的学校、单位、姓名、电
*               话、地址信息会被贴出在公司官网、官方微信公众平台、官方技术博客、知乎
*               专栏以及淘宝店铺首页予以公示公告，此种所作所为，会成为个人污点，影响
*               升学、找工作、社会声誉、很快就很在无人机界出名，后果很严重。
*               因此行为给公司造成重大损失者，会以法律途径解决，感谢您的合作，谢谢！！！
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "Filter.h"

int  Ce[10]; 
//-----Butterworth变量-----//
Butter_Parameter Butter_80HZ_Parameter_Acce,Butter_60HZ_Parameter_Acce,Butter_51HZ_Parameter_Acce,
Butter_30HZ_Parameter_Acce,Butter_20HZ_Parameter_Acce,Butter_15HZ_Parameter_Acce,
Butter_10HZ_Parameter_Acce,Butter_5HZ_Parameter_Acce,Butter_2HZ_Parameter_Acce;
Butter_BufferData Butter_Buffer[3];
Butter_BufferData Butter_Buffer_Feedback[3];
Butter_BufferData Butter_Buffer_SINS[3];
Butter_Parameter Butter_5HZ_Parameter_RC;
/****************************************
Butterworth低通滤波器参数初始化：http://blog.csdn.net/u011992534/article/details/73743955
***************************************/
/***********************************************************
@函数名：Butterworth_Parameter_Init
@入口参数：无
@出口参数：无
功能描述：巴特沃斯低通滤波器初始化
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void Butterworth_Parameter_Init(void)
{
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 80,&Butter_80HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 60,&Butter_60HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 51,&Butter_51HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 30,&Butter_30HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 20,&Butter_20HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 15,&Butter_15HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 10,&Butter_10HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 5 ,&Butter_5HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 2 ,&Butter_2HZ_Parameter_Acce);
	Set_Cutoff_Frequency(50, 10 ,&Butter_5HZ_Parameter_RC);
	BTfilter_Init();
  pascalTriangle(10,1,Ce);
}

/*************************************************
函数名:	float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
说明:	加速度计低通滤波器
入口:	float curr_input 当前输入加速度计,滤波器参数，滤波器缓存
出口:	无
备注:	2阶Butterworth低通滤波器
*************************************************/
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
  /* 加速度计Butterworth滤波 */
  /* 获取最新x(n) */
  Buffer->Input_Butter[2]=curr_input;
  /* Butterworth滤波 */
  Buffer->Output_Butter[2]=
    Parameter->b[0] * Buffer->Input_Butter[2]
      +Parameter->b[1] * Buffer->Input_Butter[1]
        +Parameter->b[2] * Buffer->Input_Butter[0]
          -Parameter->a[1] * Buffer->Output_Butter[1]
            -Parameter->a[2] * Buffer->Output_Butter[0];
  /* x(n) 序列保存 */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) 序列保存 */
  Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
  return Buffer->Output_Butter[2];
}

Butter_Parameter Notch_Filter_Parameter_55hz={
  //200hz---55hz-5hz  采样-陷波频率-带宽
  1,   0.3008218748187,   0.9229897627825,
  0.9614948813912,   0.3008218748187,   0.9614948813912
};
Butter_Parameter Notch_Filter_Parameter_60hz={
  //200hz---60hz-5hz  采样-陷波频率-带宽
  1,   0.5942365167088,   0.9229897627825,
  0.9614948813912,   0.5942365167088,   0.9614948813912
};
Butter_Parameter Notch_Filter_Parameter_65hz={
  //200hz---65hz-5hz  采样-陷波频率-带宽
  1,   0.8730190833996,   0.9229897627825,
  0.9614948813912,   0.8730190833996,   0.9614948813912
};

Butter_Parameter Notch_Filter_Parameter_75hz={
  //200hz---75hz-50hz  采样-陷波频率-带宽
  1,   0.9372808715784,   0.3255153203391,
  0.6627576601696,   0.9372808715784,   0.6627576601696
};
Notch_Filter_BufferData  Notch_Filter_Buffer;
/***********************************************************
@函数名：Notch_Filter
@入口参数：float curr_input,Notch_Filter_BufferData *Buffer
,Butter_Parameter *Parameter
@出口参数：滤波结果
功能描述：陷波滤波器
@作者：WUST
@日期：2019年01月27日
*************************************************************/
float Notch_Filter(float curr_input,Notch_Filter_BufferData *Buffer,Butter_Parameter *Parameter)
{
  float temp=0;
  /* 获取最新x(n) */ 
  Buffer->Output_Butter[0]= Parameter->a[0]*curr_input
    -Parameter->a[1]*Buffer->Output_Butter[1]
      -Parameter->a[2]*Buffer->Output_Butter[2];
  /* 计算陷波滤波器输出 */
  temp=Parameter->b[0]*Buffer->Output_Butter[0]
    +Parameter->b[1]*Buffer->Output_Butter[1]
      +Parameter->b[2]*Buffer->Output_Butter[2];
  /* y(n) 序列保存 */
  Buffer->Output_Butter[2]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[0]; 
  return temp;
}

Butter_Parameter Bandstop_Filter_Parameter_30_98={
  //200hz---30hz-98hz  采样-阻带
  1,   0.627040f,  -0.290527f,
  0.354737f,   0.627040f,    0.354737f
};
Butter_Parameter Bandstop_Filter_Parameter_30_94={
  //200hz---30hz-94hz  采样-阻带
  1,   0.5334540355829,  -0.2235264828971,
  0.3882367585514,   0.5334540355829,   0.3882367585514
};

Butter_BufferData  Bandstop_Filter_Buffer[2];
/***********************************************************
@函数名：BPF_Butterworth
@入口参数：float curr_input,Notch_Filter_BufferData *Buffer
,Butter_Parameter *Parameter
@出口参数：滤波结果
功能描述：带通滤波器
@作者：WUST
@日期：2019年01月27日
*************************************************************/
float BPF_Butterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
  /* 加速度计Butterworth滤波 */
  /* 获取最新x(n) */
  Buffer->Input_Butter[2]=curr_input;
  /* Butterworth滤波 */
  Buffer->Output_Butter[2]=
    Parameter->b[0] * Buffer->Input_Butter[2]
      +Parameter->b[1] * Buffer->Input_Butter[1]
        +Parameter->b[2] * Buffer->Input_Butter[0]
          -Parameter->a[1] * Buffer->Output_Butter[1]
            -Parameter->a[2] * Buffer->Output_Butter[0];
  /* x(n) 序列保存 */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) 序列保存 */
  Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
  return Buffer->Output_Butter[2];
}

// discrete low pass filter, cuts out the
// high frequency noise that can drive the controller crazy
//derivative = _last_derivative + _d_lpf_alpha * (derivative - _last_derivative);
float set_lpf_alpha(int16_t cutoff_frequency, float time_step)
{
  // calculate alpha
  float lpf_alpha;
  float rc = 1/(2*PI*cutoff_frequency);
  lpf_alpha = time_step / (time_step + rc);
  return lpf_alpha;
}

//https://blog.csdn.net/sszhouplus/article/details/43113505
//https://blog.csdn.net/shengzhadon/article/details/46784509
//https://blog.csdn.net/shengzhadon/article/details/46791903
//https://blog.csdn.net/shengzhadon/article/details/46803401
/***********************************************************
@函数名：Set_Cutoff_Frequency
@入口参数：float sample_frequent, float cutoff_frequent,
Butter_Parameter *LPF
@出口参数：无
功能描述：巴特沃斯低通滤波器初始化
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF)
{
  float fr = sample_frequent / cutoff_frequent;
  float ohm = tanf(M_PI_F / fr);
  float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
  if (cutoff_frequent <= 0.0f) {
    // no filtering
    return;
  }
  LPF->b[0] = ohm * ohm / c;
  LPF->b[1] = 2.0f * LPF->b[0];
  LPF->b[2] = LPF->b[0];
  LPF->a[0]=1.0f;
  LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
  LPF->a[2] = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}

float Filter_data[4];
void Test_Filter()
{
  static uint32_t Filter_Cnt=0;
  static float Ts=0.005f;
  Filter_Cnt++;
  Filter_data[0]=100*sin(2*PI*30*Filter_Cnt*Ts)
    +100*sin(2*PI*90*Filter_Cnt*Ts)
      //+100*sin(2*PI*150*Filter_Cnt*Ts)
      +100*sin(2*PI*75*Filter_Cnt*Ts);//多种频率混合信号
  Filter_data[1]=Notch_Filter(Filter_data[0],&Notch_Filter_Buffer,&Notch_Filter_Parameter_75hz);
  Filter_data[2]=BPF_Butterworth(Filter_data[0],&Bandstop_Filter_Buffer[0],&Bandstop_Filter_Parameter_30_98);
  Filter_data[3]=BPF_Butterworth(Filter_data[0],&Bandstop_Filter_Buffer[1],&Butter_51HZ_Parameter_Acce);
}

#define SYMBOL_ADD  0
#define SYMBOL_SUB  1
/*======================================================================
* 函数名：  pascalTriangle
* 函数功能：计算杨辉三角的第N行的值（数组），该系列值为(x+1)^N的系数，
*         加改进(x-1)^N的系数，最低次数在第一个
*
* 变量名称：
*          N      - 杨辉三角第N行，N=0,1,...,N
*          symbol - 运算符号，0——(x+1)^N，1——(x-1)^N
*          vector - 返回数组，杨辉三角的第N行的值
*
* 返回值：  void
*=====================================================================*/
void pascalTriangle(int N,int symbol,int *vector)
{
  vector[0] = 1;
  if(N == 0)
  {
    return;
  }
  else if (N == 1)
  {
    if(symbol == SYMBOL_ADD)
    {
      vector[1] = 1;
    }
    else
    {
      vector[0] = -1; //如果是减号，则第二项系数是-1
      vector[1] = 1;
    }
    return;
  }
  int length = N + 1; //数组长度
  int temp[11];   //定义中间变量
  
  temp[0] = 1;
  temp[1] = 1;
  
  for(int i = 2; i <= N; i++)
  {
    vector[i] = 1;
    for(int j = 1; j < i; j++)
    {
      vector[j] = temp[j - 1] + temp[j]; //x[m][n] = x[m-1][n-1] + x[m-1][n]
    }
    if(i == N) //最后一次不需要给中间变量赋值
    {
      if(symbol == SYMBOL_SUB) //运算符为减号
      {
        for(int k = 0; k < length; k++)
        {
          vector[k] = vector[k] * pow(-1, length - 1 - k);
        }
      }
      return;
    }
    for(int j = 1; j <= i; j++)
    {
      temp[j] = vector[j];
    }
  }
}

float Data_X_MAG[N2];
float Data_Y_MAG[N2];
float Data_Z_MAG[N2];
/***********************************************************
@函数名：GildeAverageValueFilter_MAG
@入口参数：float NewValue,float *Data
@出口参数：无
功能描述：滑动窗口滤波
@作者：WUST
@日期：2019年01月27日
*************************************************************/
float GildeAverageValueFilter_MAG(float NewValue,float *Data)
{
  float max,min;
  float sum;
  unsigned char i;
  Data[0]=NewValue;
  max=Data[0];
  min=Data[0];
  sum=Data[0];
  for(i=N2-1;i!=0;i--)
  {
    if(Data[i]>max) max=Data[i];
    else if(Data[i]<min) min=Data[i];
    sum+=Data[i];
    Data[i]=Data[i-1];
  }
  i=N2-2;
  sum=sum-max-min;
  sum=sum/i;
  return(sum);
}
ButterFilter_Parameter  ButterFilter_Parameter_20hz={
 2.395964410378e-05,0.0001916771528302,0.0006708700349057, 0.001341740069811,
 0.001677175087264, 0.001341740069811,0.0006708700349057,0.0001916771528302,2.395964410378e-05,
   
 1,   -4.784514894996,    10.44504106553,   -13.45771989024,11.12933103916,   
 -6.025260397298,    2.079273803012,  -0.4172171569898,0.03720010070485
};

ButterFilter_Parameter  ButterFilter_Parameter_60hz={
0.0292610171415,    0.234088137132,    0.819308479962,    1.638616959924,
2.048271199905,    1.638616959924,    0.819308479962,    0.234088137132,0.0292610171415,
	
1,    1.590566495785,    2.083813300268,    1.532625563294,0.8694409154849,   
0.3191759432528,   0.0820901315715,  0.01224667018615,0.0008613683811974
};

ButterFilter_Parameter  ButterFilter_Parameter_30hz={
  0.0003584389449098, 0.002867511559278,  0.01003629045747,  0.02007258091495,0.02509072614368,
  0.02007258091495,  0.01003629045747, 0.002867511559278,0.0003584389449098,

	1,   -3.184639500575,    5.182875452367,   -5.216195177838,3.495496357771,   -1.572964272335,
	0.4607417913074,  -0.0797799386734,0.006225657872695
};	

ButterFilter_Parameter  ButterFilter_Parameter_15hz={
3.250205999303e-06,2.600164799443e-05,9.100576798049e-05, 0.000182011535961,0.0002275144199512, 
0.000182011535961,9.100576798049e-05,2.600164799443e-05,3.250205999303e-06,

1,   -5.586778072871,    13.92678962164,   -20.17204012269,18.52850023508,   -11.03326804615,    
4.154023314407,  -0.9031234042111,0.08672852752674
};
float BTfilter(ButterFilterStruct *butterValue,float data)
{
    int N = butterValue->N; //系数数组的长度 
    butterValue->input[N]=data;
    //第一层循环，计算length个y的输出值
    butterValue->output[N]=0.0f;
		 for(int i = 0; i<=N; i++)
    {
      butterValue->output[N]+=butterValue->para.num[i]*butterValue->input[N-i];
    }
     for(int i = 1; i<=N; i++)
    {
      butterValue->output[N]-=butterValue->para.den[i]*butterValue->output[N-i];
    }
		for(int i = 0; i < N; i++)
		{
		  butterValue->output[i]=butterValue->output[i+1];
			butterValue->input[i] =butterValue->input[i+1];
		}
		return butterValue->output[N];
}

ButterFilterStruct Accel_X_INS_ButterFilter,Accel_Y_INS_ButterFilter,Accel_Z_INS_ButterFilter;
ButterFilterStruct Accel_X_IMU_ButterFilter,Accel_Y_IMU_ButterFilter,Accel_Z_IMU_ButterFilter;
ButterFilterStruct Gyro_X_IMU_ButterFilter,Gyro_Y_IMU_ButterFilter,Gyro_Z_IMU_ButterFilter;
void BTfilter_Init(void)
{
	Accel_X_IMU_ButterFilter.N=8;
	Accel_Y_IMU_ButterFilter.N=8;
	Accel_Z_IMU_ButterFilter.N=8;
	
	Accel_X_INS_ButterFilter.N=8;
	Accel_Y_INS_ButterFilter.N=8;
	Accel_Z_INS_ButterFilter.N=8;
	
  Gyro_X_IMU_ButterFilter.N=8;
	Gyro_Y_IMU_ButterFilter.N=8;
	Gyro_Z_IMU_ButterFilter.N=8;
	for(int i=0;i<=8;i++)
	{
		Accel_X_IMU_ButterFilter.para.num[i]=ButterFilter_Parameter_30hz.num[i];
		Accel_X_IMU_ButterFilter.para.den[i]=ButterFilter_Parameter_30hz.den[i];
		Accel_Y_IMU_ButterFilter.para.num[i]=ButterFilter_Parameter_30hz.num[i];
		Accel_Y_IMU_ButterFilter.para.den[i]=ButterFilter_Parameter_30hz.den[i];
		Accel_Z_IMU_ButterFilter.para.num[i]=ButterFilter_Parameter_30hz.num[i];
		Accel_Z_IMU_ButterFilter.para.den[i]=ButterFilter_Parameter_30hz.den[i];
		
	  Accel_X_INS_ButterFilter.para.num[i]=ButterFilter_Parameter_60hz.num[i];
		Accel_X_INS_ButterFilter.para.den[i]=ButterFilter_Parameter_60hz.den[i];
		Accel_Y_INS_ButterFilter.para.num[i]=ButterFilter_Parameter_60hz.num[i];
		Accel_Y_INS_ButterFilter.para.den[i]=ButterFilter_Parameter_60hz.den[i];
		Accel_Z_INS_ButterFilter.para.num[i]=ButterFilter_Parameter_60hz.num[i];
		Accel_Z_INS_ButterFilter.para.den[i]=ButterFilter_Parameter_60hz.den[i];
		
		
		Gyro_X_IMU_ButterFilter.para.num[i]=ButterFilter_Parameter_60hz.num[i];
		Gyro_X_IMU_ButterFilter.para.den[i]=ButterFilter_Parameter_60hz.den[i];
		Gyro_Y_IMU_ButterFilter.para.num[i]=ButterFilter_Parameter_60hz.num[i];
		Gyro_Y_IMU_ButterFilter.para.den[i]=ButterFilter_Parameter_60hz.den[i];
		Gyro_Z_IMU_ButterFilter.para.num[i]=ButterFilter_Parameter_60hz.num[i];
		Gyro_Z_IMU_ButterFilter.para.den[i]=ButterFilter_Parameter_60hz.den[i];
  }
}

/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/

