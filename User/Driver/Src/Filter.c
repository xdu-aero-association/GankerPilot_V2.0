/* Copyright (c)  2019-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
/*----------------------------------------------------------------------------------------------------------------------/
                ��Դ����������ѣ������ߵ���ʷ�Ѿ�֤�����ڹ�����������£���������Ŀ�Դ������������˰����ߣ��Է��ز�����
								Ŀ���Ƶķ�ʽ�в�ͨ���õĿ�Դ��Ŀ��Ҫ��רְ��Ա�����ۺ�����ְ��ֽ�ѧ��ͳ���û�����������ʵ������ɶԲ�
								Ʒ��һ�δ���������������������ۺϿ��ǣ������ɿؿ�Դ�����У������������˾���漤��ܴ��룬�汾������ʵ��
								���ܣ����ڹ�˾��Ʒ���������������Ӱ��ͻ�ѧϰ���������ȫ�����Ÿ��ͻ�ѧϰ���ͻ���ֲ�Ͷ��ο��������뱣����
								���Ȩ��
-----------------------------------------------------------------------------------------------------------------------/
*               ������ֻ��������ѧϰʹ�ã���Ȩ����Ȩ���������ƴ��Ŷӣ������ƴ��Ŷӽ��ɿس���Դ���ṩ�������ߣ�
*               ������ҪΪ�����ƴ��Ŷ��ṩ������δ���������ɣ����ý�Դ�����ṩ�����ˣ����ý�Դ����ŵ����Ϲ�����������أ� 
*               �������Դ�����Ĳ�����緢��������Ϊ�������ƴ��Ŷӽ���֮�Է��ɽ��������
-----------------------------------------------------------------------------------------------------------------------
*                                                 Ϊʲôѡ���������£�
*                                         �ж����ļ۸�������׵Ŀ�Դ�ɿأ�
*                                         ����ҵ������֮������µ��ۺ����
*                                         ׷�����û����飬��Ч����ѧϰ֮·��
*                                         ���²��ٹµ��������������տ�Դ�߶ȣ�
*                                         ��Ӧ���ҷ�ƶ���٣��ٽ��������ƹ�ƽ��
*                                         ��ʱ���ܶ�����ʣ����������˹�ͬ�塣 
-----------------------------------------------------------------------------------------------------------------------
*               ������Ϣ���ܶ���ֹ��ǰ�����������˳���������
*               ��Դ���ף���ѧ����ϧ��ף������Ϯ�����׳ɹ�������
*               ѧϰ�����ߣ��������Ƽ���DJI��ZEROTECH��XAG��AEE��GDU��AUTEL��EWATT��HIGH GREAT�ȹ�˾��ҵ
*               ��ְ�����뷢�ͣ�15671678205@163.com���豸ע��ְ����λ����λ��������
*               �����ƴ���Դ�ɿ�QQȺ��2��Ⱥ465082224��1��Ⱥ540707961����Ա������
*               CSDN���ͣ�http://blog.csdn.net/u011992534
*               �ſ�ID��NamelessCotrun����С��
*               Bվ��ѧ��Ƶ��https://space.bilibili.com/67803559/#/video
*               �ͻ�ʹ���ĵá��Ľ������������http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               �Ա����̣�https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               �ٶ�����:�����ƴ���Դ�ɿ�
*               ��˾����:www.nameless.tech
*               �������¹����׿�TI��Դ�ɿ���Ƴ��ԡ�֪��ר��:https://zhuanlan.zhihu.com/p/54471146
*               �޸�����:2020/03/28                    
*               �汾�������ߡ���GankerPilot_V2.0
*               ��Ȩ���У�����ؾ���
*               Copyright(C) 2019-2025 �人�������¿Ƽ����޹�˾ 
*               All rights reserved
-----------------------------------------------------------------------------------------------------------------------
*               ��Ҫ��ʾ��
*               �����Ա�����ת�ֵķɿء��������ѡ�����ѧ�ܵĶ����Խ��ۺ�Ⱥѧϰ������
*               ����ֱ�����������������������ϣ��������´�����������Ȩ�����˲��ý�
*               ���ϴ��봫���Ϲ��������أ�������ı��ΪĿ���������ϴ��룬�����д˲�
*               ���ߣ���˾����ǰ��֪����1���ڼ�ʱ�������������ѧУ����λ����������
*               ������ַ��Ϣ�ᱻ�����ڹ�˾�������ٷ�΢�Ź���ƽ̨���ٷ��������͡�֪��
*               ר���Լ��Ա�������ҳ���Թ�ʾ���棬����������Ϊ�����Ϊ�����۵㣬Ӱ��
*               ��ѧ���ҹ���������������ܿ�ͺ������˻����������������ء�
*               �����Ϊ����˾����ش���ʧ�ߣ����Է���;���������л���ĺ�����лл������
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "Filter.h"

int  Ce[10]; 
//-----Butterworth����-----//
Butter_Parameter Butter_80HZ_Parameter_Acce,Butter_60HZ_Parameter_Acce,Butter_51HZ_Parameter_Acce,
Butter_30HZ_Parameter_Acce,Butter_20HZ_Parameter_Acce,Butter_15HZ_Parameter_Acce,
Butter_10HZ_Parameter_Acce,Butter_5HZ_Parameter_Acce,Butter_2HZ_Parameter_Acce;
Butter_BufferData Butter_Buffer[3];
Butter_BufferData Butter_Buffer_Feedback[3];
Butter_BufferData Butter_Buffer_SINS[3];
Butter_Parameter Butter_5HZ_Parameter_RC;
/****************************************
Butterworth��ͨ�˲���������ʼ����http://blog.csdn.net/u011992534/article/details/73743955
***************************************/
/***********************************************************
@��������Butterworth_Parameter_Init
@��ڲ�������
@���ڲ�������
����������������˹��ͨ�˲�����ʼ��
@���ߣ�WUST
@���ڣ�2019��01��27��
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
������:	float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
˵��:	���ٶȼƵ�ͨ�˲���
���:	float curr_input ��ǰ������ٶȼ�,�˲����������˲�������
����:	��
��ע:	2��Butterworth��ͨ�˲���
*************************************************/
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
  /* ���ٶȼ�Butterworth�˲� */
  /* ��ȡ����x(n) */
  Buffer->Input_Butter[2]=curr_input;
  /* Butterworth�˲� */
  Buffer->Output_Butter[2]=
    Parameter->b[0] * Buffer->Input_Butter[2]
      +Parameter->b[1] * Buffer->Input_Butter[1]
        +Parameter->b[2] * Buffer->Input_Butter[0]
          -Parameter->a[1] * Buffer->Output_Butter[1]
            -Parameter->a[2] * Buffer->Output_Butter[0];
  /* x(n) ���б��� */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) ���б��� */
  Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
  return Buffer->Output_Butter[2];
}

Butter_Parameter Notch_Filter_Parameter_55hz={
  //200hz---55hz-5hz  ����-�ݲ�Ƶ��-����
  1,   0.3008218748187,   0.9229897627825,
  0.9614948813912,   0.3008218748187,   0.9614948813912
};
Butter_Parameter Notch_Filter_Parameter_60hz={
  //200hz---60hz-5hz  ����-�ݲ�Ƶ��-����
  1,   0.5942365167088,   0.9229897627825,
  0.9614948813912,   0.5942365167088,   0.9614948813912
};
Butter_Parameter Notch_Filter_Parameter_65hz={
  //200hz---65hz-5hz  ����-�ݲ�Ƶ��-����
  1,   0.8730190833996,   0.9229897627825,
  0.9614948813912,   0.8730190833996,   0.9614948813912
};

Butter_Parameter Notch_Filter_Parameter_75hz={
  //200hz---75hz-50hz  ����-�ݲ�Ƶ��-����
  1,   0.9372808715784,   0.3255153203391,
  0.6627576601696,   0.9372808715784,   0.6627576601696
};
Notch_Filter_BufferData  Notch_Filter_Buffer;
/***********************************************************
@��������Notch_Filter
@��ڲ�����float curr_input,Notch_Filter_BufferData *Buffer
,Butter_Parameter *Parameter
@���ڲ������˲����
�����������ݲ��˲���
@���ߣ�WUST
@���ڣ�2019��01��27��
*************************************************************/
float Notch_Filter(float curr_input,Notch_Filter_BufferData *Buffer,Butter_Parameter *Parameter)
{
  float temp=0;
  /* ��ȡ����x(n) */ 
  Buffer->Output_Butter[0]= Parameter->a[0]*curr_input
    -Parameter->a[1]*Buffer->Output_Butter[1]
      -Parameter->a[2]*Buffer->Output_Butter[2];
  /* �����ݲ��˲������ */
  temp=Parameter->b[0]*Buffer->Output_Butter[0]
    +Parameter->b[1]*Buffer->Output_Butter[1]
      +Parameter->b[2]*Buffer->Output_Butter[2];
  /* y(n) ���б��� */
  Buffer->Output_Butter[2]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[0]; 
  return temp;
}

Butter_Parameter Bandstop_Filter_Parameter_30_98={
  //200hz---30hz-98hz  ����-���
  1,   0.627040f,  -0.290527f,
  0.354737f,   0.627040f,    0.354737f
};
Butter_Parameter Bandstop_Filter_Parameter_30_94={
  //200hz---30hz-94hz  ����-���
  1,   0.5334540355829,  -0.2235264828971,
  0.3882367585514,   0.5334540355829,   0.3882367585514
};

Butter_BufferData  Bandstop_Filter_Buffer[2];
/***********************************************************
@��������BPF_Butterworth
@��ڲ�����float curr_input,Notch_Filter_BufferData *Buffer
,Butter_Parameter *Parameter
@���ڲ������˲����
������������ͨ�˲���
@���ߣ�WUST
@���ڣ�2019��01��27��
*************************************************************/
float BPF_Butterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
  /* ���ٶȼ�Butterworth�˲� */
  /* ��ȡ����x(n) */
  Buffer->Input_Butter[2]=curr_input;
  /* Butterworth�˲� */
  Buffer->Output_Butter[2]=
    Parameter->b[0] * Buffer->Input_Butter[2]
      +Parameter->b[1] * Buffer->Input_Butter[1]
        +Parameter->b[2] * Buffer->Input_Butter[0]
          -Parameter->a[1] * Buffer->Output_Butter[1]
            -Parameter->a[2] * Buffer->Output_Butter[0];
  /* x(n) ���б��� */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) ���б��� */
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
@��������Set_Cutoff_Frequency
@��ڲ�����float sample_frequent, float cutoff_frequent,
Butter_Parameter *LPF
@���ڲ�������
����������������˹��ͨ�˲�����ʼ��
@���ߣ�WUST
@���ڣ�2019��01��27��
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
      +100*sin(2*PI*75*Filter_Cnt*Ts);//����Ƶ�ʻ���ź�
  Filter_data[1]=Notch_Filter(Filter_data[0],&Notch_Filter_Buffer,&Notch_Filter_Parameter_75hz);
  Filter_data[2]=BPF_Butterworth(Filter_data[0],&Bandstop_Filter_Buffer[0],&Bandstop_Filter_Parameter_30_98);
  Filter_data[3]=BPF_Butterworth(Filter_data[0],&Bandstop_Filter_Buffer[1],&Butter_51HZ_Parameter_Acce);
}

#define SYMBOL_ADD  0
#define SYMBOL_SUB  1
/*======================================================================
* ��������  pascalTriangle
* �������ܣ�����������ǵĵ�N�е�ֵ�����飩����ϵ��ֵΪ(x+1)^N��ϵ����
*         �ӸĽ�(x-1)^N��ϵ������ʹ����ڵ�һ��
*
* �������ƣ�
*          N      - ������ǵ�N�У�N=0,1,...,N
*          symbol - ������ţ�0����(x+1)^N��1����(x-1)^N
*          vector - �������飬������ǵĵ�N�е�ֵ
*
* ����ֵ��  void
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
      vector[0] = -1; //����Ǽ��ţ���ڶ���ϵ����-1
      vector[1] = 1;
    }
    return;
  }
  int length = N + 1; //���鳤��
  int temp[11];   //�����м����
  
  temp[0] = 1;
  temp[1] = 1;
  
  for(int i = 2; i <= N; i++)
  {
    vector[i] = 1;
    for(int j = 1; j < i; j++)
    {
      vector[j] = temp[j - 1] + temp[j]; //x[m][n] = x[m-1][n-1] + x[m-1][n]
    }
    if(i == N) //���һ�β���Ҫ���м������ֵ
    {
      if(symbol == SYMBOL_SUB) //�����Ϊ����
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
@��������GildeAverageValueFilter_MAG
@��ڲ�����float NewValue,float *Data
@���ڲ�������
�������������������˲�
@���ߣ�WUST
@���ڣ�2019��01��27��
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
    int N = butterValue->N; //ϵ������ĳ��� 
    butterValue->input[N]=data;
    //��һ��ѭ��������length��y�����ֵ
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
