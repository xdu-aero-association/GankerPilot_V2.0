#ifndef __KEY_H__
#define __KEY_H__

#define QuadKey1  MAP_GPIO_getInputPinValue(GPIO_PORT_P1 , GPIO_PIN1)
#define QuadKey2  MAP_GPIO_getInputPinValue(GPIO_PORT_P1 , GPIO_PIN4)

void Key_Init(void);
bool Key_Scan(uint8_t release);
void QuadShow(uint8_t show_enable);


extern uint8_t Key_Right_Release,Oled_Show_Enable;
extern int16_t Page_Number;


#endif


