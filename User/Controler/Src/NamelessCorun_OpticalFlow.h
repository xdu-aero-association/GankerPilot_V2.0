/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
/*----------------------------------------------------------------------------------------------------------------------/
*               ������ֻ��������ѧϰʹ�ã���Ȩ����Ȩ���������ƴ��Ŷӣ������ƴ��Ŷӽ��ɿس���Դ���ṩ�������ߣ�
*               ������ҪΪ�����ƴ��Ŷ��ṩ������δ���������ɣ����ý�Դ�����ṩ�����ˣ����ý�Դ����ŵ����Ϲ�����������أ� 
*               �������Դ�����Ĳ�����緢��������Ϊ�������ƴ��Ŷӽ���֮�Է��ɽ��������
-----------------------------------------------------------------------------------------------------------------------/
*               ������Ϣ���ܶ���ֹ��ǰ�����������˳���������
*               ��Դ���ף���ѧ����ϧ��ף������Ϯ�����׳ɹ�������
*               ѧϰ�����ߣ��������Ƽ���DJI��ZEROTECH��XAG��AEE��GDU��AUTEL��EWATT��HIGH GREAT�ȹ�˾��ҵ
*               ��ְ�����뷢�ͣ�15671678205@163.com���豸ע��ְ����λ����λ��������
*               �����ƴ���Դ�ɿ�QQȺ��540707961
*               CSDN���ͣ�http://blog.csdn.net/u011992534
*               �ſ�ID��NamelessCotrun����С��
*               Bվ��ѧ��Ƶ��https://space.bilibili.com/67803559/#/video
*               �ͻ�ʹ���ĵá��Ľ������������http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               �Ա����̣�https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               �ٶ�����:�����ƴ���Դ�ɿ�
*               ��˾����:www.nameless.tech
*               �޸�����:2019/4/12
*               �汾����Ӯ�ߡ���CarryPilot_V1.0
*               ��Ȩ���У�����ؾ���
*               Copyright(C) 2017-2025 �人�������¿Ƽ����޹�˾ 
*               All rights reserved
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
#ifndef _NAMELESSCOTRUN_OPTICALFLOW_
#define _NAMELESSCOTRUN_OPTICALFLOW_

#define LC306_OPTICAL_FLOW 1
#define CL_OPTICAL_FLOW 0

#if (CL_OPTICAL_FLOW==1)
	#define FLOW_BUF_ONE_FRAME  9
#endif

#if (LC306_OPTICAL_FLOW==1)
	#define FLOW_BUF_ONE_FRAME  14
#endif

#define FLOW_BUF_CNT  FLOW_BUF_ONE_FRAME*2


typedef struct{
  unsigned short frame_count_since_last_readout;
  signed short pixel_flow_x_integral;
  signed short pixel_flow_y_integral;
  signed short gyro_x_rate_integral;
  signed short gyro_y_rate_integral;
  signed short gyro_z_rate_integral;
  unsigned int integration_timespan;
  unsigned int sonar_timestamp;
  unsigned short ground_distance;
  signed short gyro_temperature;
  unsigned char qual;
	unsigned char sum;
}flow_integral_frame;

typedef struct{
  float x;
  float y;
  unsigned short dt;
  unsigned char qual;
  unsigned char update;
}flow_float;

extern  flow_integral_frame opt_origin_data;
extern Vector2f opt_filter_data,opt_gyro_filter_data,opt_gyro_data,gyro_filter_data;
extern SINS OpticalFlow_SINS;
void OpticalFlow_Init(void);
uint8_t Optflow_Prase(void);
void Optflow_Statemachine(void);
float OpticalFlow_Rotate_Complementary_Filter(float optflow_gyro,float gyro,uint8_t axis);
extern flow_float opt_data;
extern uint8_t OpticalFlow_Is_Work;
extern Vector2f OpticalFlow_Speed,OpticalFlow_Speed_Err,OpticalFlow_Position;
extern Testime Optical_Delta;
void Version_Declaration(void);
void System_Start_Init(void);
void Copyright_Show(void);

#endif


