/* Copyright (C) 2015 Kristian Sloth Lauszus. All rights reserved.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

Contact information
-------------------

Kristian Sloth Lauszus
Web      :  http://www.lauszus.com
e-mail   :  lauszus@gmail.com
*/
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
uint8 i2c_r=2;
const eUSCI_I2C_MasterConfig i2cConfig =
{
        EUSCI_B_I2C_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        12000000,                                // SMCLK = 3MHz (default)
        EUSCI_B_I2C_SET_DATA_RATE_1MBPS,      // Desired I2C Clock of 100khz
        0,                                      // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
};
/***********************************************************
@��������Init_I2C
@��ڲ�������
@���ڲ�������
����������MSP432���I2C��ʼ��
@���ߣ�WUST
@���ڣ�2019��01��27��
*************************************************************/
void Init_I2C(void) 
{
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN4);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN5);
	delay_ms(10);
	MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P6,GPIO_PIN4);
	MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P6,GPIO_PIN5);
}
///////////////////////////////////////
void SDA_IOOUT(void)
{
	GPIO_setAsOutputPin(GPIO_PORT_P6,GPIO_PIN4);//SDA
}

void SDA_IOIN(void)
{
	GPIO_setAsInputPin(GPIO_PORT_P6,GPIO_PIN4);//SDA
}

static void I2C_delay(void)
{
  //volatile int i = 1;	//7
  //while (i)
  //    i--;
}


//**************************************
void I2C_Start(void)
{
  SDA_IOOUT();
  SCL_H;
  SDA_H;
  I2C_delay();
  SDA_L;
  I2C_delay();
}
//**************************************
void I2C_Stop(void)
{
  SDA_IOOUT();
  SCL_L;
  I2C_delay();
  SDA_L;
  I2C_delay();
  SCL_H;
  I2C_delay();
  SDA_H;
  I2C_delay();
}

//**************************************
unsigned char I2C_SlaveAck(void)
{
  SDA_IOOUT();
  SCL_L;
  I2C_delay();
  SDA_H;
  
  SDA_IOIN();
  I2C_delay();
  SCL_H;
  
  I2C_delay();
  
  if(SDA_read)
  {
    SCL_L;
    return 0;
  }
  SCL_L;
  I2C_delay();
  return 1;
}

//**************************************
void I2C_SendByte(unsigned char byte)
{
  unsigned char i = 8;
  SDA_IOOUT();
  while (i--)
  {
    SCL_L;
    I2C_delay();
    if (byte & 0x80)
      SDA_H;
    else
      SDA_L;
    byte <<= 1;
    I2C_delay();
    SCL_H;
    I2C_delay();
  }
  SCL_L;
  if(I2C_SlaveAck()==0)
  {
    return ;
  }
}

//**************************************
unsigned char I2C_RecvByte(void)
{
  unsigned char i;
  unsigned char dat = 0;
  SDA_IOIN();
  for (i=0; i<8; i++)
  {
    dat <<= 1;
    SCL_H;
    I2C_delay();
    dat |= SDA_read;
    SCL_L;
    I2C_delay();
  }
  return dat;
}

//**************************************
void Single_WriteI2C(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)
{
	I2C_Start();
  I2C_SendByte(SlaveAddress);
  I2C_SendByte(REG_Address);
  I2C_SendByte(REG_data);
  I2C_Stop();
}
bool OK=0;
unsigned char reg_data;
unsigned char Single_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address)
{
  unsigned char REG_data;
  I2C_Start();
  I2C_SendByte(SlaveAddress);
  I2C_SendByte(REG_Address);
  I2C_Start();
  I2C_SendByte(SlaveAddress+1);
  REG_data=I2C_RecvByte();
  I2C_SlaveAck();
  I2C_Stop();
  return REG_data;
}
///////////////////////////////
int16_t Double_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address)
{
  uint8_t msb , lsb ;
  msb = Single_ReadI2C(SlaveAddress,REG_Address);
  lsb = Single_ReadI2C(SlaveAddress,REG_Address+1);
  return ((msb<< 8 )+ lsb) ;
}

unsigned long int Three_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address)
{
  unsigned char msb , lsb,xlsb;
  msb = Single_ReadI2C(SlaveAddress,REG_Address);
  lsb = Single_ReadI2C(SlaveAddress,REG_Address+1);
  xlsb= Single_ReadI2C(SlaveAddress,REG_Address+2);
  return (((unsigned long int)msb<< 16)|((unsigned long int)lsb<<8)|xlsb) ;
}



static void I2C_Delay_Adjust(void)
{
  volatile int i = 1;	//7
  while (i)
    i--;
}


//**************************************
void I2C_Start_Adjust(void)
{
  SDA_IOOUT();
  SCL_H;
  SDA_H;
  I2C_Delay_Adjust();
  SDA_L;
  I2C_Delay_Adjust();
}
//**************************************
void I2C_Stop_Adjust(void)
{
  SDA_IOOUT();
  SCL_L;
  I2C_Delay_Adjust();
  SDA_L;
  I2C_Delay_Adjust();
  SCL_H;
  I2C_Delay_Adjust();
  SDA_H;
  I2C_Delay_Adjust();
}

//**************************************
unsigned char I2C_SlaveAck_Adjust(void)
{
  SDA_IOOUT();
  SCL_L;
  I2C_Delay_Adjust();
  SDA_H;
  
  SDA_IOIN();
  I2C_Delay_Adjust();
  SCL_H;
  
  I2C_Delay_Adjust();
  
  if(SDA_read)
  {
    SCL_L;
    return 0;
  }
  SCL_L;
  I2C_Delay_Adjust();
  return 1;
}

//**************************************
void I2C_SendByte_Adjust(unsigned char byte)
{
  unsigned char i = 8;
  SDA_IOOUT();
  while (i--)
  {
    SCL_L;
    I2C_Delay_Adjust();
    if (byte & 0x80)
      SDA_H;
    else
      SDA_L;
    byte <<= 1;
    I2C_Delay_Adjust();
    SCL_H;
    I2C_Delay_Adjust();
  }
  SCL_L;
  if(I2C_SlaveAck_Adjust()==0)
  {
    return ;
  }
}

//**************************************
unsigned char I2C_RecvByte_Adjust(void)
{
  unsigned char i;
  unsigned char dat = 0;
  SDA_IOIN();
  for (i=0; i<8; i++)
  {
    dat <<= 1;
    SCL_H;
    I2C_Delay_Adjust();
    dat |= SDA_read;
    SCL_L;
    I2C_Delay_Adjust();
  }
  return dat;
}

//**************************************
void Single_WriteI2C_Adjust(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)
{
	MAP_I2C_setSlaveAddress(EUSCI_B1_BASE, SlaveAddress>>1);
	while (MAP_I2C_masterIsStopSent(EUSCI_B1_BASE));
	/* Send out EEPROM Mock Read Cmd (2 databytes) */
	MAP_I2C_masterSendMultiByteStart(EUSCI_B1_BASE, REG_Address);  // Start + 1Byte
	MAP_I2C_masterSendMultiByteFinish(EUSCI_B1_BASE, REG_data); // Poll for TXINT,Send 1Byte
}

//**************************************
unsigned char Single_ReadI2C_Adjust(unsigned char SlaveAddress,unsigned char REG_Address)
{
  unsigned char REG_data;
	OK=0;
	MAP_I2C_setSlaveAddress(EUSCI_B1_BASE, SlaveAddress>>1);
	while (MAP_I2C_masterIsStopSent(EUSCI_B1_BASE));
	/* Send out EEPROM Mock Read Cmd (2 databytes) */
	MAP_I2C_masterSendMultiByteStart(EUSCI_B1_BASE, REG_Address);  // Start + 1Byte
	/* Now we need to initiate the read */
	/* Wait until 2nd Byte has been output to shift register */
	while(!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));
	// Send the restart condition, read one byte, send the stop condition right away
	EUSCI_B1->CTLW0 &= ~(EUSCI_B_CTLW0_TR);
	EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
	while(MAP_I2C_masterIsStartSent(EUSCI_B1_BASE));
	EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
	while(!OK);
	REG_data=reg_data;
  return REG_data;
}

unsigned short int Double_ReadI2C_Adjust(unsigned char SlaveAddress,unsigned char REG_Address)
{
  unsigned char msb , lsb ;
  msb = Single_ReadI2C_Adjust(SlaveAddress,REG_Address);
  lsb = Single_ReadI2C_Adjust(SlaveAddress,REG_Address+1);
  return ( ((unsigned short int)msb) << 8 | lsb) ;
}

unsigned long int Three_ReadI2C_Adjust(unsigned char SlaveAddress,unsigned char REG_Address)
{
  unsigned char msb , lsb,xlsb;
  msb = Single_ReadI2C_Adjust(SlaveAddress,REG_Address);
  lsb = Single_ReadI2C_Adjust(SlaveAddress,REG_Address+1);
  xlsb= Single_ReadI2C_Adjust(SlaveAddress,REG_Address+2);
  return (((unsigned long int)msb<< 16)|((unsigned long int)lsb<<8)|xlsb) ;
}
void EUSCIB1_IRQHandler(void)
{
    uint_fast16_t status;

    status = MAP_I2C_getEnabledInterruptStatus(EUSCI_B1_BASE);
    MAP_I2C_clearInterruptFlag(EUSCI_B1_BASE, status);

    /* Receives bytes into the receive buffer. If we have received all bytes,
     * send a STOP condition */
    if (status & EUSCI_B_I2C_RECEIVE_INTERRUPT0)
    {
        // One-byte Read
			reg_data = MAP_I2C_masterReceiveSingle(EUSCI_B1_BASE);
			OK=1;
			
    }
}

/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/

