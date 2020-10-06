#ifndef __HEADFILE_H__
#define __HEADFILE_H__
/*----------------------------------------------------------------------------------------------------------------------/
*               ������ֻ����Уѧ��ѧϰʹ�ã���Ȩ����Ȩ�����人�Ƽ���ѧ���人�Ƽ���ѧ���ɿس���Դ���ṩ����Уѧ����
*               ��Уѧ��ҪΪ�人�Ƽ���ѧ�ṩ������δ��ѧУ��ɣ����ý�Դ�����ṩ�����ˣ����ý�Դ����ŵ����Ϲ�����������أ� 
*               �������Դ�����Ĳ�����緢��������Ϊ���人�Ƽ���ѧ����֮�Է��ɽ��������
-----------------------------------------------------------------------------------------------------------------------/
*               ������Ϣ���ܶ���ֹ��ǰ�����������˳���������
*               ��Դ���ף���ѧ����ϧ��ף������Ϯ�����׳ɹ�������
*               ѧϰ�����ߣ��������Ƽ���DJI��ZEROTECH��XAG��AEE��GDU��AUTEL��EWATT��HIGH GREAT�ȹ�˾��ҵ
*               ��ְ�����뷢�ͣ�15671678205@163.com���豸ע��ְ����λ����λ��������
*               �����ƴ���Դ�ɿ�QQȺ��540707961
*               CSDN���ͣ�http://blog.csdn.net/u011992534
*               �ſ�ID��NamelessCotrunWUST
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
*               ѧУ����˾����ǰ��֪����1���ڼ�ʱ�����������ѧУ����λ����������
*               ������ַ��Ϣ�ᱻ�����ڹ�˾�������ٷ�΢�Ź���ƽ̨���ٷ��������͡�֪��
*               ר���Լ��Ա�������ҳ���Թ�ʾ���棬����������Ϊ�����Ϊ�����۵㣬Ӱ��
*               ��ѧ���ҹ���������������ܿ�ͺ������˻����������������ء�
*               �����Ϊ����˾����ش���ʧ�ߣ����Է���;���������л���ĺ�����лл������
----------------------------------------------------------------------------------------------------------------------*/



#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>


#define ABS(X)  (((X)>0)?(X):-(X))
#define MAX(a,b)  ((a)>(b)?(a):(b))
#define Imu_Sampling_Freq  200
#define WP_Duty_Freq  200
#define WP_Duty_Dt  (1.0f/WP_Duty_Freq)


typedef   signed           char int8;
typedef unsigned           char u8;
typedef unsigned           char uint8;
typedef unsigned           char byte;
typedef   signed short     int int16;
typedef unsigned short     int uint16;
typedef unsigned short     int u16;
typedef unsigned long     int u32; 

#define US100  0
#define TFMINI 1
#define Ground_Distance_Sensor  US100


#define _YAW    0
#define _PITCH  1
#define _ROLL   2

#define Delta 0.005f
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
//#include "hw_memmap.h"
//#include "hw_types.h"
//#include "hw_ints.h"
//#include "debug.h"
//#include "fpu.h"
//#include "gpio.h"
////#include "pin_map.h"
////#include "pwm.h"
//#include "rom.h"
//#include "sysctl.h"
//#include "uart.h"
//#include "interrupt.h"
//#include "timer.h"
//#include "hw_gpio.h"
//#include "eeprom.h"
/////////////////////////
#include "WP_DataType.h"
#include "Usart.h"
//#include "uartstdio.h"
#include "Time.h"
#include "Time_Cnt.h"
//#include "Schedule.h"
#include "License.h"
#include "ssd1306.h"
#include "oled.h"
#include "mpu6050.h"
#include "Soft_I2C.h"
#include "ist8310.h"
#include "spl06.h"
#include "PPM.h"
#include "wp_flash.h"
#include "PID.h"
#include "Ringbuf.h"
#include "WP_PWM.h"
#include "Sensor.h"
#include "Key.h"
#include "Filter.h"
#include "SINS.h"
#include "Calibration.h"
#include "CalibrationRoutines.h"
#include "US_100.h"
#include "tfmini.h"
#include "NamelessCorun_OpticalFlow.h"
#include "OpticalFlow_LC306.h"
#include "OpticalFlow_Control.h"
#include "NamelessCotrun_SDK.h"
#include "WP_ADC.h"
#include "Bling.h"
#include "GPS.h"
#include "RC.h"
#include "SYSTEM.h"
#include "WP_Ctrl.h"
#include "control_config.h"
#include "Temperature_Ctrl.h"
#include "Compass.h"
#include "NCLink.h"
extern SINS NamelessQuad,Filter_Feedback_NamelessQuad;
extern Sensor WP_Sensor;

#endif

