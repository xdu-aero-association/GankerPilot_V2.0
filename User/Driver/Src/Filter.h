#ifndef _FILTER_H_
#define _FILTER_H_
/*----------------------------------------------------------------------------------------------------------------------/
        *               ������ֻ����Уѧ��ѧϰʹ�ã���Ȩ����Ȩ�����人�Ƽ���ѧ��
        *               �人�Ƽ���ѧ���ɿس���Դ���ṩ����Уѧ����
        *               ��Уѧ��ҪΪ�人�Ƽ���ѧ�ṩ������
        *               δ��ѧУ��ɣ����ý�Դ�����ṩ������
        *               ���ý�Դ����ŵ����Ϲ�����������أ�
        *               �������Դ�����Ĳ�����緢��������Ϊ��
        *               �人�Ƽ���ѧ����֮�Է��ɽ��������
-----------------------------------------------------------------------------------------------------------------------/
        *               ������Ϣ���ܶ���ֹ��ǰ�����������˳���������
        *               ��Դ���ף���ѧ����ϧ��ף������Ϯ�����׳ɹ�������
-----------------------------------------------------------------------------------------------------------------------/
	*		�����ƴ���Դ�ɿ� V1.1	�人�Ƽ���ѧ  By.YuYi
	*		CSDN����: http://blog.csdn.net/u011992534
	*               �ſ�ID��NamelessCotrunWUST
	*               �����ƴ���Դ�ɿ�QQȺ��540707961
        *               https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
        *               �ٶ�����:�����ƴ���Դ�ɿ�
        *               �޸�����:2017/10/30
        *               �汾��V1.1
        *               ��Ȩ���У�����ؾ���
        *               Copyright(C) �人�Ƽ���ѧ�人�Ƽ���ѧ 2017-2019
        *               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/


typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;


typedef struct
{
 float Output_Butter[3];
}Notch_Filter_BufferData;


typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;

//������˹�˲�������
typedef struct
{
  float num[9];
  float den[9];
}ButterFilter_Parameter;
typedef struct
{
    int N;          //������˹�˲�����Сʵ�ֽ���
	  ButterFilter_Parameter para;
	  float input[9];
	  float output[9];
}ButterFilterStruct;

extern Butter_Parameter Bandstop_Filter_Parameter_30_98,Bandstop_Filter_Parameter_30_94;

extern Butter_Parameter Butter_80HZ_Parameter_Acce,Butter_60HZ_Parameter_Acce,Butter_51HZ_Parameter_Acce,
                 Butter_30HZ_Parameter_Acce,Butter_20HZ_Parameter_Acce,Butter_15HZ_Parameter_Acce,
                 Butter_10HZ_Parameter_Acce,Butter_5HZ_Parameter_Acce,Butter_2HZ_Parameter_Acce,Butter_5HZ_Parameter_RC;

float set_lpf_alpha(int16_t cutoff_frequency, float time_step);
void Acce_Control_Filter(void);
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);


void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF);
void Butterworth_Parameter_Init(void);
void Test_Filter(void);
float BPF_Butterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);


void pascalTriangle(int N,int symbol,int *vector);
#define N2 3
extern float Data_X_MAG[N2];
extern float Data_Y_MAG[N2];
extern float Data_Z_MAG[N2];
extern Butter_Parameter Bandstop_Filter_Parameter_30_98;
extern Butter_Parameter Bandstop_Filter_Parameter_30_94;
float GildeAverageValueFilter_MAG(float NewValue,float *Data);
void BTfilter_Init(void);
float BTfilter(ButterFilterStruct *butterValue,float data);
extern ButterFilterStruct Accel_X_INS_ButterFilter,Accel_Y_INS_ButterFilter,Accel_Z_INS_ButterFilter;
extern ButterFilterStruct Accel_X_IMU_ButterFilter,Accel_Y_IMU_ButterFilter,Accel_Z_IMU_ButterFilter;
extern ButterFilterStruct Gyro_X_IMU_ButterFilter,Gyro_Y_IMU_ButterFilter,Gyro_Z_IMU_ButterFilter;
#endif


