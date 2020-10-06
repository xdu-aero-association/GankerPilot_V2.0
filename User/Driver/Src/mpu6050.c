/* Copyright (c)  2019-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
/*----------------------------------------------------------------------------------------------------------------------/
                ��Դ����������ѣ������ߵ���ʷ�Ѿ�֤�����ڹ�����������£���������Ŀ�Դ������������˰����ߣ��Է��ز�����
								Ŀ���Ƶķ�ʽ�в�ͨ���õĿ�Դ��Ŀ��Ҫ��רְ��Ա�����ۺ�����ְ��ֽ�ѧ��ͳ���û�����������ʵ������ɶԲ�
								Ʒ��һ�δ���������������������ۺϿ��ǣ������ɿؿ�Դ�����У������������˾���漤��ܴ��룬�汾������ʵ��
								���ܣ����ڹ�˾��Ʒ���������������Ӱ��ͻ�ѧϰ���������ȫ�����Ÿ��ͻ�ѧϰ���ͻ���ֲ�Ͷ��ο��������뱣����
								���Ȩ��
-----------------------------------------------------------------------------------------------------------------------/
*               ������ֻ��������ѧϰʹ�ã���Ȩ����Ȩ���������ƴ��Ŷӣ������ƴ��Ŷӽ��ɿس���Դ���ṩ�������ߣ�
*               ������ҪΪ�����ƴ��Ŷ��ṩ������δ��������ɣ����ý�Դ�����ṩ�����ˣ����ý�Դ����ŵ����Ϲ�����������أ� 
*               �������Դ�����Ĳ�����緢��������Ϊ�������ƴ��Ŷӽ���֮�Է��ɽ��������
-----------------------------------------------------------------------------------------------------------------------
*                                                 Ϊʲôѡ���������£�
*                                         �ж����ļ۸�������׵Ŀ�Դ�ɿأ�
*                                         ����ҵ������֮������µ��ۺ����
*                                         ׷�����û����飬��Ч����ѧϰ֮·��
*                                         ���²��ٹµ�������������տ�Դ�߶ȣ�
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
*               ���ߣ���˾����ǰ��֪����1���ڼ�ʱ�����������ѧУ����λ����������
*               ������ַ��Ϣ�ᱻ�����ڹ�˾�������ٷ�΢�Ź���ƽ̨���ٷ��������͡�֪��
*               ר���Լ��Ա�������ҳ���Թ�ʾ���棬����������Ϊ�����Ϊ�����۵㣬Ӱ��
*               ��ѧ���ҹ���������������ܿ�ͺ������˻����������������ء�
*               �����Ϊ����˾����ش���ʧ�ߣ����Է���;���������л���ĺ�����лл������
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "myiic.h"
#include "mpu6050.h"
Butter_Parameter Ins_Accel_Parameter;
Butter_Parameter Accel_Parameter;
Butter_Parameter Gyro_Parameter;
Butter_Parameter Calibrate_Parameter;
uint8_t IMU_ID=0x68; 
/***********************************************************
@��������Init_MPU6050
@��ڲ�������
@���ڲ�������
����������MPU6050��ʼ��
@���ߣ�����С��
@���ڣ�2019��01��27��
*************************************************************/
void Init_MPU6050(void)//MPU6050��ʼ��
{
	IMU_ID=Single_ReadI2C(MPU_ADRESS,WHO_AM_I);
  if(IMU_ID==IMU_MPU6050_ID)//MPU6050
	{
		Single_WriteI2C(MPU_ADRESS,PWR_MGMT_1  , 0x00);//�ر������ж�,�������
		Single_WriteI2C(MPU_ADRESS,SMPLRT_DIV  , 0x00); // sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz
		Single_WriteI2C(MPU_ADRESS,MPU_CONFIG  , 0x00); //�ڲ���ͨ�˲�Ƶ�ʣ����ٶȼ�94hz��������98hz
//		Single_WriteI2C(MPU_ADRESS,MPU_CONFIG  , 0x03);//�ڲ���ͨ�˲�Ƶ�ʣ����ٶȼ�44hz��������42hz
		Single_WriteI2C(MPU_ADRESS,GYRO_CONFIG , 0x10);//1000deg/s
		Single_WriteI2C(MPU_ADRESS,ACCEL_CONFIG, 0x18);// Accel scale 16g (2048 LSB/g)
	}
	else if(IMU_ID==IMU_ICM20689_ID)//ICM20689
	{	
		Single_WriteI2C(MPU_ADRESS,PWR_MGMT_1  , 0x00);//�ر������ж�,�������
		Single_WriteI2C(MPU_ADRESS,SMPLRT_DIV  , 0x00); // sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz	
		Single_WriteI2C(MPU_ADRESS,MPU_CONFIG  , 0x00);//���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������41hz����������59hz���¶�42hz
//		Single_WriteI2C(MPU_ADRESS,MPU_CONFIG  , 0x03);//���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������41hz����������59hz���¶�42hz
		Single_WriteI2C(MPU_ADRESS,GYRO_CONFIG , 0x10);//�������������̣�1000deg/s
		Single_WriteI2C(MPU_ADRESS,ACCEL_CONFIG, 0x18);// Accel scale 16g (2048 LSB/g)	
		Single_WriteI2C(MPU_ADRESS,ACCEL_CONFIG2,0x03);//���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�44.8hz����������61.5hz
	}
  delay_ms(500);
  IMU_Calibration();
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 50,&Gyro_Parameter);//��̬���ٶȷ����˲�����  50
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 30,&Accel_Parameter);//��̬����Ӽ������˲�ֵ 30
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 2,&Calibrate_Parameter);//������У׼�Ӽ��˲�ֵ
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 30,&Ins_Accel_Parameter);//INS�Ӽ��˲�ֵ
}

/***********************************************************
@��������MPU6050_Read_Data
@��ڲ�����vector3f *gyro,vector3f *accel
@���ڲ�������
����������MPU6050���ݲɼ�
@���ߣ�����С��
@���ڣ�2019��01��27��
*************************************************************/
void MPU6050_Read_Data(vector3f *gyro,vector3f *accel)//��ȡMPU6050����
{
  accel->x=-Double_ReadI2C(MPU_ADRESS,ACCEL_XOUT_H);
  accel->y=Double_ReadI2C(MPU_ADRESS,ACCEL_YOUT_H);
  accel->z=-Double_ReadI2C(MPU_ADRESS,ACCEL_ZOUT_H);
	
  gyro->x=-Double_ReadI2C(MPU_ADRESS,GYRO_XOUT_H);
  gyro->y=Double_ReadI2C(MPU_ADRESS,GYRO_YOUT_H);
  gyro->z=-Double_ReadI2C(MPU_ADRESS,GYRO_ZOUT_H);
	
}


/***********************************************************
@MPU6050_Read_Temperature
@��ڲ�����float *temperature
@���ڲ�������
����������MPU6050�¶����ݲɼ�
@���ߣ�����С��
@���ڣ�2019��01��27��
*************************************************************/
void MPU6050_Read_Temperature(float *temperature)
{
	if(IMU_ID==IMU_MPU6050_ID)
	{
		//MPU6050
		//Sensitivity=340 LSB/��C
		//Temperature Offset=35 ��C
		*temperature=36.53+((double)Double_ReadI2C(MPU_ADRESS,TEMP_OUT_H))/340;
	}
	if(IMU_ID==IMU_ICM20689_ID)
	{
		//ICM20689
		//Room Temperature Offset=25 ��C
		//Sensitivity=326.8 LSB/��C
		//TEMP_degC = ((TEMP_OUT �CRoomTemp_Offset)/Temp_Sensitivity) +25degC
		*temperature=25.0f+((double)Double_ReadI2C(MPU_ADRESS,TEMP_OUT_H)-25.0f)/326.8;
	}
	
  //ICM20602
	//Room Temperature Offset=25 ��C
	//Sensitivity=326.8 LSB/��C
	//TEMP_degC =(TEMP_OUT)/Temp_Sensitivity +25degC
	//*temperature=25.0f+(double)Double_ReadI2C(MPU_ADRESS,TEMP_OUT_H)/326.8;
}
Vector3f gyro_offset;
s32 g_Gyro_xoffset = 0, g_Gyro_yoffset = 0, g_Gyro_zoffset = 0;
/***********************************************************
@��������IMU_Calibration
@��ڲ�������
@���ڲ�������
���������������ǿ�����ƫ�궨
@���ߣ�����С��
@���ڣ�2019��01��27��
*************************************************************/
void IMU_Calibration(void)
{
  u8 i;
  delay_ms(1000);
  for (i = 0; i < 100; i++)			//��������30�Σ�һ����ʱ30*3=90ms
  {
    g_Gyro_xoffset -=Double_ReadI2C(MPU_ADRESS,GYRO_XOUT_H);
    g_Gyro_yoffset +=Double_ReadI2C(MPU_ADRESS,GYRO_YOUT_H);
    g_Gyro_zoffset -=Double_ReadI2C(MPU_ADRESS,GYRO_ZOUT_H);
    delay_ms(5);
  }
  gyro_offset.x =(g_Gyro_xoffset/100);//�õ��궨ƫ��
  gyro_offset.y =(g_Gyro_yoffset/100);
  gyro_offset.z =(g_Gyro_zoffset/100);
}

