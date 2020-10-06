#ifndef _FILTER_H_
#define _FILTER_H_
/*----------------------------------------------------------------------------------------------------------------------/
        *               本程序只供本校学生学习使用，版权著作权属于武汉科技大学，
        *               武汉科技大学将飞控程序源码提供给本校学生，
        *               本校学生要为武汉科技大学提供保护，
        *               未经学校许可，不得将源代码提供给他人
        *               不得将源代码放到网上供他人免费下载，
        *               更不能以此销售牟利，如发现上述行为，
        *               武汉科技大学将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
        *               生命不息、奋斗不止；前人栽树，后人乘凉！！！
        *               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
-----------------------------------------------------------------------------------------------------------------------/
	*		无名科创开源飞控 V1.1	武汉科技大学  By.YuYi
	*		CSDN博客: http://blog.csdn.net/u011992534
	*               优酷ID：NamelessCotrunWUST
	*               无名科创开源飞控QQ群：540707961
        *               https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
        *               百度贴吧:无名科创开源飞控
        *               修改日期:2017/10/30
        *               版本：V1.1
        *               版权所有，盗版必究。
        *               Copyright(C) 武汉科技大学武汉科技大学 2017-2019
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

//巴特沃斯滤波器参数
typedef struct
{
  float num[9];
  float den[9];
}ButterFilter_Parameter;
typedef struct
{
    int N;          //巴特沃斯滤波器最小实现阶数
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


