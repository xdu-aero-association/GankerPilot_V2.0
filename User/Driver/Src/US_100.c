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
#include "US_100.h"



float US_Distance=0,Last_US_Distance=0,US_Distance_Div=0,Last_US_Distance_Div=0;
float US_Distance_Acc=0;
uint16_t Start_Tail=0,Last_Start_Tail;



uint16 Sample_Cnt=0;
void US100_Init(void)
{
  MAP_GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P6, GPIO_PIN6);
	GPIO_interruptEdgeSelect(GPIO_PORT_P6,GPIO_PIN6,GPIO_LOW_TO_HIGH_TRANSITION);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P6, GPIO_PIN6);
	MAP_GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN6);
	MAP_Interrupt_enableInterrupt(INT_PORT6);
}


void US100_UP()
{
	GPIO_interruptEdgeSelect(GPIO_PORT_P6,GPIO_PIN6,GPIO_LOW_TO_HIGH_TRANSITION); 
}

void US100_DN()
{
	GPIO_interruptEdgeSelect(GPIO_PORT_P6,GPIO_PIN6,GPIO_HIGH_TO_LOW_TRANSITION); 
}



void PORT6_IRQHandler(void)
{
	uint32_t status;
	static uint16_t us100_time=0;
	status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P6);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P6, status);
	if(status & GPIO_PIN6)
  {
    if(us100_time==0)
		{
		  us100_time=10000*TIME_ISR_CNT+Timer_A_getCounterValue(TIMER_A1_BASE);//us
		
		}
  }
}





/***********************************************************
@��������US_100_Distance
@��ڲ�����uint8 MSB,uint8 LSB
@���ڲ�������
@�����������������߶����ݺϳ�
@���ߣ�����С��
@���ڣ�2019��01��27��
*************************************************************/
float US_100_Distance(uint8 MSB,uint8 LSB)
{
  return (256*(MSB)+(LSB))/10.0f;//��λcm
}

/***********************************************************
@��������US_100_Temperature
@��ڲ�����uint8 data
@���ڲ�������
@�����������������¶����ݺϳ�
@���ߣ�����С��
@���ڣ�2019��01��27��
*************************************************************/
float US_100_Temperature(uint8 data)
{
  return (data-45)/1.0;
}

/***********************************************************
@��������US_100_Start
@��ڲ�������
@���ڲ�������
@�����������������߶����ݴ���
@���ߣ�����С��
@���ڣ�2019��01��27��
*************************************************************/
void US_100_Start(void)
{	
  USART1_Send_Byte(US_100_Distance_CMD);
}


uint8_t US_100_Update_Flag=0;
uint8 US_100_Cnt=0;

/***********************************************************
@��������US_100_Statemachine
@��ڲ�������
@���ڲ�������
@�������������������ݽ���״̬��
@���ߣ�����С��
@���ڣ�2019��01��27��
*************************************************************/
void US_100_Statemachine(void)
{
  US_100_Cnt++;
  if(US_100_Cnt>=20)//100ms
  {
		Last_US_Distance=US_Distance;
    US_100_Start();//�������������������ⲿ�ж����洦��
    US_100_Cnt=0;
    Start_Tail=COM1_Rx_Buf.Tail;
    if(ABS(Start_Tail-Last_Start_Tail)==2)
    {
      if(Start_Tail==0)
      {
        US_Distance=US_100_Distance(COM1_Rx_Buf.Ring_Buff[2],
                                    COM1_Rx_Buf.Ring_Buff[3]);
        US_100_Update_Flag=1;
      }
      else if(Start_Tail==1)
      {
        US_Distance=US_100_Distance(COM1_Rx_Buf.Ring_Buff[3],
                                    COM1_Rx_Buf.Ring_Buff[0]);
        US_100_Update_Flag=1;
      }
      else if(Start_Tail==2)
      {
        US_Distance=US_100_Distance(COM1_Rx_Buf.Ring_Buff[0],
                                    COM1_Rx_Buf.Ring_Buff[1]);
        US_100_Update_Flag=1;
      }
      else if(Start_Tail==3)
      {
        US_Distance=US_100_Distance(COM1_Rx_Buf.Ring_Buff[1],
                                    COM1_Rx_Buf.Ring_Buff[2]);
        US_100_Update_Flag=1;
      }
			WP_Sensor.us100_updtate_flag=1;
    }
    Last_Start_Tail=Start_Tail;
		US_Distance_Div=(US_Distance-Last_US_Distance)/0.1f;
		US_Distance_Acc=(US_Distance_Div-Last_US_Distance_Div)/0.1f;
		Last_US_Distance_Div=US_Distance_Div;
  }
  if(US_Distance<=User_Height_Max&&US_Distance>0)  Sensor_Flag.Hcsr04_Health=1;
  else  Sensor_Flag.Hcsr04_Health=0; 
}



/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
