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
#include "Soft_I2C.h"


#define USE_SOFT_I2C 0
#define GPIO_I2C   		GPIO_PORT_P10 
#define SCL_PIN   		GPIO_PIN3
#define SDA_PIN   		GPIO_PIN2
#define I2C_READ_SDA  GPIO_getInputPinValue(GPIO_I2C,SDA_PIN)			//SDA 
#define I2C_SDA_H  		GPIO_setOutputHighOnPin(GPIO_I2C,SDA_PIN)//SDA 
#define I2C_SDA_L 		GPIO_setOutputLowOnPin(GPIO_I2C,SDA_PIN)			//SDA
#define I2C_SCL_H  		GPIO_setOutputHighOnPin(GPIO_I2C,SCL_PIN)//SCL
#define I2C_SCL_L  		GPIO_setOutputLowOnPin(GPIO_I2C,SCL_PIN)			//SCL


void I2C_GPIO_Config(void)
{
#if defined(USE_SOFT_I2C)	
	MAP_GPIO_setAsOutputPin(GPIO_I2C, SCL_PIN);
	MAP_GPIO_setAsOutputPin(GPIO_I2C, SDA_PIN);
	
  Delay_Ms(10);
	MAP_GPIO_setOutputHighOnPin(GPIO_I2C,SCL_PIN);
	MAP_GPIO_setOutputHighOnPin(GPIO_I2C,SDA_PIN);
#else
  Init_I2C0();
#endif	
}

#if defined(USE_SOFT_I2C)
void I2C_SDA_OUT(void)
{
	GPIO_setAsOutputPin(GPIO_I2C,SDA_PIN);//SDA
}

void I2C_SDA_IN(void)
{
  GPIO_setAsInputPin(GPIO_I2C,SDA_PIN);//SDA
}

#define I2C_SDA_OUT I2C_SDA_OUT()
#define I2C_SDA_IN  I2C_SDA_IN()
#define I2C_sdaRead I2C_READ_SDA  		


static void i2cDelay()
{
//    volatile int i = 7;//7
//    while (i)
//    i--;
}
 
// SCL�ߵ�ƽ�ڼ䣬SDA�����½���Ϊ��ʼ�ź�
static bool i2cStart()
{
    I2C_SDA_OUT;
    I2C_SCL_H;
    I2C_SDA_H;
    i2cDelay();
    if (!I2C_sdaRead)  // ���SDAΪ�͵�ƽ��������æ���˳�
        return false;
    I2C_SDA_L;
    if (I2C_sdaRead)  // ���SDAΪ�ߵ�ƽ��������æ���˳�
        return false;
    I2C_SDA_L;
    return true;
}
 
// SCL�ߵ�ƽ�ڼ䣬SDA����������Ϊֹͣ�ź�
static void i2cStop(void)
{
    I2C_SDA_OUT;
    I2C_SCL_L; 
    I2C_SDA_L;
    i2cDelay();  // STOP:when CLK is high DATA from low to high 
    I2C_SCL_H;
    I2C_SDA_H;  
    i2cDelay();
}
 
static void i2cAck(void)
{
    I2C_SDA_OUT;
    I2C_SCL_L;
    i2cDelay();
    I2C_SDA_L;
    i2cDelay();
    I2C_SCL_H;
    i2cDelay();
    I2C_SCL_L;
}
 
static void i2cNoAck(void)
{
    I2C_SDA_OUT;
    I2C_SCL_L;
    i2cDelay();
    I2C_SDA_H;
    i2cDelay();
    I2C_SCL_H;
    i2cDelay();
    I2C_SCL_L;
}
 
// SCL�ߵ�ƽ�ڼ䣬SDA��ƽ�����豸���ͱ�ʾӦ��
static bool i2cWaitAck(void)
{
    uint8_t errTimes = 0;
    I2C_SDA_IN;
    I2C_SDA_H;
    i2cDelay();
    I2C_SCL_H;
    i2cDelay();
    while (I2C_sdaRead) {
        if (errTimes++ > 20) {
            I2C_SCL_L;
            return false;
        }           
        i2cDelay();
    }
    I2C_SCL_L;
    return true;
}
 
// �������ݣ����ݴӸ�λ����λ����  
static void i2cSendByte(uint8_t byte)  
{
    uint8_t i = 8;
 
    I2C_SDA_OUT;
    while (i--) {      
        I2C_SCL_L;  // ʱ���ź�Ϊ�͵�ƽ�ڼ䣬���������ߵ�ƽ�仯
        i2cDelay();
        if (byte & 0x80)
            I2C_SDA_H;
        else
            I2C_SDA_L; 
        byte <<= 1; 
        i2cDelay();
        I2C_SCL_H;
        i2cDelay();
    }
    I2C_SCL_L;
}
 
static uint8_t i2cReceiveByte()  
{
    uint8_t i = 8;
    uint8_t byte = 0;
    I2C_SDA_IN;
    I2C_SDA_H;
    while (i--) {
        byte <<= 1;
        I2C_SCL_H;
        i2cDelay();
        if (I2C_sdaRead) {
            byte |= 0x01;
        }
        I2C_SCL_L;
        i2cDelay();
    }
    I2C_SCL_L;
    return byte; 
}
 
 

 
/**
 * ͨ��I2C����дһ�ֽ�����
 * @param[in] dev:�豸I2C��ַ
 * @param[in] reg:�Ĵ�����ַ
 * @param[in] data:Ҫд�������
 */
bool i2cWriteOneByte(uint8_t dev, uint8_t reg, uint8_t data)
{
    if (!i2cStart())        
        return false;
    i2cSendByte(dev << 1);  // �ӻ���ַ�ɸ�7λ+��дλ����   
    if (!i2cWaitAck()) {     
        i2cStop();
        return false;
    }
    i2cSendByte(reg);       
    i2cWaitAck();
    i2cSendByte(data);     
    i2cWaitAck();
    return true;
}
 

/**
 *  
 * @param[in] dev:�豸I2C��ַ
 * @param[in] reg:�Ĵ�����ַ
 * @param[in] len:�ֽ��� 
 * @param[in] data:��д������� 
 */
uint8_t i2cReadOneBytes(uint8_t dev, uint8_t reg)
{
	unsigned char REG_data;
  i2cStart();
  i2cSendByte(dev<<1);
	i2cWaitAck();
  i2cSendByte(reg);
	i2cWaitAck();
  i2cStart();
  i2cSendByte((dev << 1) | 0x01);
  i2cWaitAck();
	REG_data=i2cReceiveByte();
  i2cStop();
  return REG_data;	
} 


/**
 *  
 * @param[in] dev:�豸I2C��ַ
 * @param[in] reg:�Ĵ�����ַ
 * @param[in] len:�ֽ��� 
 * @param[in] data:��д������� 
 */
bool i2cWriteBytes(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data)
{
    uint8_t i;
 
    if (!i2cStart())        
        return false;
    i2cSendByte(dev << 1);          
    if (!i2cWaitAck()) {     
        i2cStop();
        return false;
    }
    i2cSendByte(dev);   
    i2cWaitAck();
    for (i = 0; i < len; i++) {
        i2cSendByte(data[i]);
        if (!i2cWaitAck()) {
            i2cStop();
            return false;
        }
    }
    i2cStop();
    return true;
}
 
 
/**
 * ��I2C�豸�ж�ȡ����
 * @param[in] dev:�豸I2C��ַ
 * @param[in] reg:�Ĵ�����ַ
 * @param[in] len:�����ֽ���
 * @param[out] data:����������
 */
bool i2cReadBytes(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data)
{
    if (!i2cStart())        
        return false;
    i2cSendByte(dev << 1);      
    if (!i2cWaitAck()) {     
        i2cStop();
        return false;
    }
    i2cSendByte(reg);     
    i2cWaitAck();
    i2cStart();           
    i2cSendByte((dev << 1) | 0x01);  // ������ַ+������    
    i2cWaitAck();
    while (len) {
        *data = i2cReceiveByte();
        if (len == 1)
            i2cNoAck();  // ���һ���ֽڲ�Ӧ��
        else
            i2cAck();
        data++;
        len--;
    }
    i2cStop();
    return true;
}
#endif


void Single_WriteI2C0(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)
{
#if defined(USE_SOFT_I2C)
  i2cWriteOneByte(SlaveAddress,REG_Address,REG_data);
#else
  i2c0Write(SlaveAddress,REG_Address,REG_data);
#endif
}

//**************************************
unsigned char Single_ReadI2C0(unsigned char SlaveAddress,unsigned char REG_Address)
{
#if defined(USE_SOFT_I2C)
  return i2cReadOneBytes(SlaveAddress,REG_Address);
#else
  return i2c0Read(SlaveAddress,REG_Address);
#endif
}

/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/

