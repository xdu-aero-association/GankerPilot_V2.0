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
#include "RC.h"

uint16 Throttle_Control=0;
int16 Pitch_Control=0,Roll_Control=0,Yaw_Control=0;
int16 Target_Angle[2]={0};
uint16 Last_Throttle_Control,Pre_Last_Throttle_Control;
uint16 Throttle_Base=1000,TempThrottle_Control;
uint16_t Controler_State=0;
uint16_t Auto_ReLock_Cnt=0;//�Զ�����������
uint8_t Auto_Relock_Flag=0;
uint8_t Auto_Relock_Flag_Set=0;
int16 Temp_RC=0;
uint16_t Unlock_Makesure_Cnt=0,Lock_Makesure_Cnt=0;
uint16_t Unwanted_Lock_Flag=0;
uint16_t Forced_Lock_Makesure_Cnt=0;
uint16 RC_NewData[4]={0};

uint16_t RC_Buttom=1000,RC_Top=2000,RC_Middle=1500,RC_Deadband=100;
uint16_t RC_Deadzone_Buttom=0,RC_Deadzone_Top=0;
Butter_BufferData RC_LPF_Buffer[4];
uint16_t PPM_LPF_Databuf[4];

int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high){
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/***********************************************************
@��������RC_Data_LPF_Filter
@��ڲ�������
@���ڲ�������
����������ң����ԭʼ�����˲�
@���ߣ�WUST
@���ڣ�2019��01��27��
*************************************************************/
void RC_Data_LPF_Filter()
{
  uint16_t i=0;
  for(i=0;i<4;i++)//��ң����ǰ�ĸ�ͨ���˲�����
  {
    PPM_LPF_Databuf[i]=(uint16_t)(LPButterworth(PPM_Databuf[i],&RC_LPF_Buffer[i],&Butter_5HZ_Parameter_RC));
  }
}

int16_t Throttle_Rate=0;
/***********************************************************
@��������Get_Thr_Rate
@��ڲ�����float Thr
@���ڲ�������
�����������������ݱ仯��
@���ߣ�WUST
@���ڣ�2019��01��27��
*************************************************************/
int16_t Get_Thr_Rate(float Thr)//���ű仯��
{
  static float Thr_Rec[20];
  uint16_t i=0;
  for(i=19;i>0;i--)
  {
    Thr_Rec[i]=Thr_Rec[i-1];
  }
  Thr_Rec[0]=Thr;
  return (int16_t)((Thr_Rec[0]-Thr_Rec[9])/1.0f);
}

/***********************************************************
@��������RC_Scale_Set
@��ڲ�����Vector_RC *rc_date
@���ڲ�������
����������ң����ͨ���г̻�ȡ
@���ߣ�WUST
@���ڣ�2019��01��27��
*************************************************************/
void RC_Scale_Set(Vector_RC *rc_date)
{
  RC_Buttom=(uint16_t)(rc_date->min);
  RC_Top=(uint16_t)(rc_date->max);
  RC_Middle=(uint16_t)(rc_date->middle);
  RC_Deadband=(uint16_t)(rc_date->deadband);
  RC_Deadzone_Buttom=RC_Middle-RC_Deadband/2;
  RC_Deadzone_Top=RC_Middle+RC_Deadband/2;
}
/***********************************************************
@��������Remote_Control
@��ڲ�������
@���ڲ�������
����������ң��������ת��
@���ߣ�WUST
@���ڣ�2019��01��27��
*************************************************************/
void Remote_Control(void )
{
	if(ppm_update_flag==1||rc_update_flag==1)
	{
		  ppm_update_flag=0;
		  rc_update_flag=0;
			RC_Data_LPF_Filter();
			RC_Scale_Set(&RC_Calibration[0]);
			if(PPM_Databuf[0]<=RC_Deadzone_Buttom)  Roll_Control=(RC_Deadzone_Buttom-PPM_LPF_Databuf[0])*Pit_Rol_Max/(RC_Deadzone_Buttom-RC_Buttom);//����г̿�����+-45
			else if(PPM_Databuf[0]>=RC_Deadzone_Top)  Roll_Control=(RC_Deadzone_Top-PPM_LPF_Databuf[0])*Pit_Rol_Max/(RC_Top-RC_Deadzone_Top);
			else Roll_Control=0;
			Roll_Control=constrain_int16_t(Roll_Control,-Pit_Rol_Max,Pit_Rol_Max);
			
			RC_Scale_Set(&RC_Calibration[1]);
			if(PPM_Databuf[1]<=RC_Deadzone_Buttom)  Pitch_Control=(RC_Deadzone_Buttom-PPM_LPF_Databuf[1])*Pit_Rol_Max/(RC_Deadzone_Buttom-RC_Buttom);//����г̿�����+-45
			else if(PPM_Databuf[1]>=RC_Deadzone_Top)  Pitch_Control=(RC_Deadzone_Top-PPM_LPF_Databuf[1])*Pit_Rol_Max/(RC_Top-RC_Deadzone_Top);
			else Pitch_Control=0;
			Pitch_Control=constrain_int16_t(Pitch_Control,-Pit_Rol_Max,Pit_Rol_Max);
			
			Target_Angle[0]=-Pitch_Control;//����ʱ������������
			Target_Angle[1]=-Roll_Control;//����ʱ�����������
			
			RC_Scale_Set(&RC_Calibration[2]);
			Temp_RC=(PPM_LPF_Databuf[2]-(RC_Buttom+Buttom_Safe_Deadband));//Ϊ�˰�ȫ�����Ÿ˵�λ����ΪButtom_Safe_Deadband
			Throttle_Control=constrain_int16_t(Temp_RC,0,1000);
			
			RC_Scale_Set(&RC_Calibration[3]);
			if(PPM_Databuf[3]<=RC_Deadzone_Buttom)  Yaw_Control=-(PPM_LPF_Databuf[3]-RC_Deadzone_Buttom)*Yaw_Max/(RC_Deadzone_Buttom-RC_Buttom);//ƫ������г̿�����+-150
			else if(PPM_Databuf[3]>=RC_Deadzone_Top)  Yaw_Control=-(PPM_LPF_Databuf[3]-RC_Deadzone_Top)*Yaw_Max/(RC_Top-RC_Deadzone_Top);
			else Yaw_Control=0;
			Yaw_Control=constrain_int16_t(Yaw_Control,-Yaw_Max,Yaw_Max);
			RC_NewData[0]=Throttle_Control;//ң������ԭʼ�г���
			Throttle_Rate=Get_Thr_Rate(Throttle_Control);
			Throttle_Control=Throttle_Base+Throttle_Control;			
			/***************************************************************
			��������ʱ��ң�����������������ҡ�˴��ڵ�λ�����ڲ���
			��������ʱ��ң�����������������ҡ�˴��ڵ�λ���������
			***************************************************************/
			if(Throttle_Control==1000
				 &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
					 &&Roll_Control==0
						 &&Pitch_Control==0)
			{
				Unlock_Makesure_Cnt++;
				if(Forced_Lock_Makesure_Cnt<2000) Forced_Lock_Makesure_Cnt++;
			}
					
			if((Throttle_Control==1000
				 &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
					 &&Roll_Control==0
						 &&Pitch_Control==0
							 &&(Unlock_Makesure_Cnt>50*2.0//����2.0S
									||Forced_Lock_Makesure_Cnt>=50*5))
									||unlock_flag==0)//����
			{
				Controler_State=Lock_Controler;
				Unlock_Makesure_Cnt=0;
				Lock_Makesure_Cnt=0;
				Forced_Lock_Makesure_Cnt=0;
				Bling_Set(&Light_1,3000,200,0.5,0,GPIO_PORT_P1,GPIO_PIN6,0);
				Bling_Mode=0;
				Page_Number=0;//OLED�ָ���ҳ
				Reset_Mag_Calibartion(1);
				Reset_Accel_Calibartion(1);
				Reset_RC_Calibartion(1);
			}
			
			if(Throttle_Control==1000
				 &&Yaw_Control<=-Yaw_Max*Scale_Pecent_Max
					 &&Roll_Control==0
						 &&Pitch_Control==0)
				Lock_Makesure_Cnt++;
			
			if((Throttle_Control==1000
				 &&Yaw_Control<=-Yaw_Max*Scale_Pecent_Max
					 &&Roll_Control==0
						 &&Pitch_Control==0
							 &&Lock_Makesure_Cnt>50*2.0//����2.0S
								 &&Gyro_Safety_Calibration_Flag==1
									&&Check_Calibration_Flag()==0x00)
									||unlock_flag==1)//����
			{
				Controler_State=Unlock_Controler;
				if(Controler_High_Mode==2)//������ڶ���ģʽ�½��� 
				{  
					 Unwanted_Lock_Flag=1;   //����Ҫ�Զ���������ģʽ
		#if    (YAW_Pos_Control_Accel_Disable==1)
					Total_Controller.High_Speed_Control.Integrate=-Total_Controller.High_Speed_Control.Integrate_Max;
		#else
					Total_Controller.High_Acce_Control.Integrate=-Total_Controller.High_Acce_Control.Integrate_Max;
		#endif
				}
				else Unwanted_Lock_Flag=0;
				Lock_Makesure_Cnt=0;
				Unlock_Makesure_Cnt=0;
				Forced_Lock_Makesure_Cnt=0;
				Bling_Set(&Light_1,3000,1000,0.5,0,GPIO_PORT_P1,GPIO_PIN6,0);
				Bling_Mode=0;
				Page_Number=0;//OLED�ָ���ҳ
				Reset_Mag_Calibartion(1);
				Reset_Accel_Calibartion(1);
				Reset_RC_Calibartion(1);
				Auto_ReLock_Cnt=50*6;//����6S
				Auto_Relock_Flag_Set=0;
			}
					
			if(Controler_State==Unlock_Controler
				 &&Auto_Relock_Flag_Set==0//�Զ�����λδ����
					 &&Throttle_Control==1000
						 &&Pitch_Control==0
							 &&Roll_Control==0
								 &&Yaw_Control==0//������ң�л���
									 &&Unwanted_Lock_Flag==0)
			{
				Auto_Relock_Flag=1;//������ʼʱ�����Զ�������־λ
				Auto_Relock_Flag_Set=1;//���ν�����ֻ��λһ��
			}			
			
			if(Auto_Relock_Flag==1)
			{
				if(Throttle_Control==1000
					 &&Pitch_Control==0
						 &&Roll_Control==0
							 &&Yaw_Control==0
								 &&Controler_State==Unlock_Controler//�Զ�����������
									 &&Unwanted_Lock_Flag==0)
				{
					Auto_ReLock_Cnt--;
					if(Auto_ReLock_Cnt<=0)  Auto_ReLock_Cnt=0;
					if(Auto_ReLock_Cnt==0)
					{
						Controler_State=Lock_Controler;//�ٴ�����
						Bling_Set(&Light_1,3000,200,0.5,0,GPIO_PORT_P1,GPIO_PIN6,0);
						Bling_Mode=0;
					}
				}
				
				if((Throttle_Control!=1000
						||Pitch_Control!=0
							||Roll_Control!=0
								||Yaw_Control!=0)
					 &&Auto_ReLock_Cnt>0//�Զ�����������
						 &&Unwanted_Lock_Flag==0)
				{
					Auto_Relock_Flag=0;//ֻҪң���ж��������ν������Զ������Ͳ�����
					Auto_ReLock_Cnt=0;
					Bling_Set(&Light_1,3000,1000,0.5,0,GPIO_PORT_P1,GPIO_PIN6,0);
					Bling_Mode=0;
				}
			}
	}
	
	if(shutdown_now_cal_flag==1)
	{
		Reset_Mag_Calibartion(1);
		Reset_Accel_Calibartion(1);
		Reset_RC_Calibartion(1);
		shutdown_now_cal_flag=0;
		Bling_Set(&Light_1,0,1000,0.3,0,GPIO_PORT_P1,GPIO_PIN6,0);
		Page_Number=0;//OLED�ָ���ҳ
	}
}

/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
