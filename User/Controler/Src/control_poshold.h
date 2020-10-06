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
*               修改日期:2018/8/30
*               版本：慧飞者——V1.0.1
*               版权所有，盗版必究。
*               Copyright(C) 武汉科技大学武汉科技大学 2017-2025
*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
#ifndef _CONTROL_POSHOLD_H
#define _CONTROL_POSHOLD_H

float ncq_speed_mapping(float input,uint16_t input_max,float output_max);
void ncq_control_poshold(void);
uint8_t get_stopping_point_xy(Vector3f *stopping_point);
void accel_to_lean_angles(Vector2f _accel_target,Vector2f *target_angle);


extern Vector3_Nav Earth_Frame_Accel_Target;   //导航（地理）坐标系，正东、正北方向目标运动加速度期望
extern Vector3_Nav Earth_Frame_Pos_Err;        //导航（地理）坐标系，正东、正北方向w位置偏差
extern Vector2_Ang Body_Frame_Accel_Target;      //导航（地理）坐标系，机体横滚(Y正)、俯仰(X正)方向目标运动加速度期望
extern Vector2_Ang Body_Frame_Speed_Feedback;    //导航（地理）坐标系，机体横滚(Y正)、俯仰(X正)方向目标运动速度反馈
extern Vector2_Ang Body_Frame_Pos_Err;           //机体方向上位置偏差
extern Vector2_Ang Body_Frame_Brake_Speed;       //机体方向上刹车速度
extern uint8 GPS_Speed_Control_Mode;
extern Vector2f accel_target,angle_target;
#endif

