#ifndef __OLED_H__
#define __OLED_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "drv_iic.h"
#include "func_oled_font.h"

/* 想要使用此文件, 用户需要在自己的 drv_iic.c/h 文件中完成 iic 外设的初始化
   并在其中定义函数 rt_err_t iic_write_reg(rt_uint16_t SLAVE_ADDR, rt_uint8_t cmd, rt_uint8_t dat)
   该函数会向外发送两字节的数据, 其中 SLAVE_ADDR 为从机地址, cmd 为发送的第一字节数据, dat 为发送的第二字节数据
*/

#define USE_EXTERN_IIC//使用rtt的iic接口函数

#define USE_HALF_Y 1//使用窄屏幕
//开启使用窄屏幕选项后，可以将6*8char相关显示，以及图形绘制函数适配到窄屏幕上

/*
用户若使用oled.c自带的软件iic, 需要修改引脚，
修改本处宏定义的引脚
并在oled_init()函数开头处对其初始化即可
*/
#define OLED_SCL_H rt_pin_write(SCL_PIN, PIN_HIGH)	//SCL引脚输出高电平
#define OLED_SCL_L rt_pin_write(SCL_PIN, PIN_LOW)	//SCL引脚输出低电平

#define OLED_SDA_H rt_pin_write(SDA_PIN, PIN_HIGH)		//SDA引脚输出高电平
#define OLED_SDA_L rt_pin_write(SDA_PIN, PIN_LOW)		//SDA引脚输出低电平


//OLED初始化函数，必须调用后才能正常使用屏幕，如果通信失败，会重新通信，如果没有连接屏幕，则此函数将不会退出
int OLED_init(void);


/*
说明：清除oled显示
*/
extern void OLED_clear(void);

//在指定位置绘制矩形，可自行设定绘制图形的沿Y方向的截面状态
//如果使用HALF_Y设置，则此处的colorset只有低4位有效
void OLED_Draw_sqar(char X_StartPOS, char Y_StartPOS, char ColorSet, char Draw_Len);

/*
说明：以6*8(8*16)的大小在oled上显示一个字符
参数：x 取值范围0~127, 单位为像素 从左到右
			y 取值范围0~7(8*16时范围为0-6)，与x一起决定显示位置
			character 你要显示的字符
*/
extern void OLED_show6x8char(rt_uint8_t x,rt_uint8_t y,char character);
extern void OLED_show8x16char(rt_uint8_t x,rt_uint8_t y,char character);
/*
说明：以6*8(8*16)的大小在oled上显示一个以'\0'字符结尾的字符串
参数：x 取值范围0~127, 单位为像素 从左到右
			y 取值范围0~7(8*16时范围为0-6)，与x一起决定显示位置
			string 你要显示的字符串
			
			该函数遇到'\0'字符才会结束。
*/
extern void OLED_show6x8string(rt_uint8_t x,rt_uint8_t y,char* string);
extern void OLED_show8x16string(rt_uint8_t x,rt_uint8_t y,char* string);
/*
说明：以6*8(8*16)的大小在oled上显示一个数字
参数：x 取值范围0~127, 单位为像素 从左到右
			y 取值范围0~7(8*16时为0-6)，与x一起决定显示位置
			number 用户要显示的数字，正负均可
*/
extern void OLED_show6x8number(rt_uint8_t x,rt_uint8_t y,int number);
extern void OLED_show8x16number(rt_uint8_t x,rt_uint8_t y,int number);

#define OLED_COMMAND 	0
#define OLED_DATA 		1


#endif

