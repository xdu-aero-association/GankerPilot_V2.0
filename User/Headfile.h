#ifndef __HEADFILE_H__
#define __HEADFILE_H__
/*----------------------------------------------------------------------------------------------------------------------/
*               本程序只供本校学生学习使用，版权著作权属于武汉科技大学，武汉科技大学将飞控程序源码提供给本校学生，
*               本校学生要为武汉科技大学提供保护，未经学校许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载， 
*               更不能以此销售牟利，如发现上述行为，武汉科技大学将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               无名科创开源飞控QQ群：540707961
*               CSDN博客：http://blog.csdn.net/u011992534
*               优酷ID：NamelessCotrunWUST
*               B站教学视频：https://space.bilibili.com/67803559/#/video
*               客户使用心得、改进意见征集贴：http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               淘宝店铺：https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               百度贴吧:无名科创开源飞控
*               公司官网:www.nameless.tech
*               修改日期:2019/4/12
*               版本：躺赢者——CarryPilot_V1.0
*               版权所有，盗版必究。
*               Copyright(C) 2017-2025 武汉无名创新科技有限公司 
*               All rights reserved
*               重要提示：
*               正常淘宝咸鱼转手的飞控、赠送朋友、传给学弟的都可以进售后群学习交流，
*               不得直接在网上销售无名创新资料，无名创新代码有声明版权，他人不得将
*               资料代码传网上供他人下载，不得以谋利为目的销售资料代码，发现有此操
*               学校，公司会提前告知，请1天内及时处理，否则你的学校、单位、姓名、电
*               话、地址信息会被贴出在公司官网、官方微信公众平台、官方技术博客、知乎
*               专栏以及淘宝店铺首页予以公示公告，此种所作所为，会成为个人污点，影响
*               升学、找工作、社会声誉、很快就很在无人机界出名，后果很严重。
*               因此行为给公司造成重大损失者，会以法律途径解决，感谢您的合作，谢谢！！！
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

