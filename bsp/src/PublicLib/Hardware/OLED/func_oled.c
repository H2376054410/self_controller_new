#include "func_oled.h"
/* 想要使用此文件, 用户需要在自己的 drv_iic.c/h 文件中完成 iic 外设的初始化
   并在其中定义函数 rt_err_t iic_write_reg(rt_uint16_t SLAVE_ADDR, rt_uint8_t cmd, rt_uint8_t dat)
   该函数会向外发送两字节的数据, 其中 SLAVE_ADDR 为从机地址, cmd 为发送的第一字节数据, dat 为发送的第二字节数据
*/
static char ERR_Count = 0;
static int POSx_Rem = -1, POSy_Rem = -1;

//该代码块中的函数对外界不开放
#ifndef USE_EXTERN_IIC 				//创建一个可折叠的代码块用于存放软件IIC操作函数
void IIC_start(void)//传输开始
{
	OLED_SCL_H;
	OLED_SDA_H;
	OLED_SDA_L;
	OLED_SCL_L;
}
void IIC_stop(void)//传输结束
{
	OLED_SCL_H;
	OLED_SDA_L;
	OLED_SDA_H;
}
void IIC_waitack(void)//假装等待应答
{
	OLED_SCL_H;
	OLED_SCL_L;
}
void IIC_sendbyte(char Byte)//传输一个字节
{
	OLED_SCL_L;
	for(int a = 0;a<8;a++)
	{
		if(Byte&0x80)
		{
			OLED_SDA_H;
		}
		else
		{
			OLED_SDA_L;
		}
		Byte <<= 1;
		OLED_SCL_H;
		OLED_SCL_L;
	}
}
void IIC_writedata(uint8_t data,char identifying)
{
	IIC_start();
	IIC_sendbyte(0x78);
	IIC_waitack();
	if(identifying)
	{
		IIC_sendbyte(0x40);
	}
	else
	{
		IIC_sendbyte(0x00);
	}
	IIC_waitack();
	IIC_sendbyte(data);
	IIC_waitack();
	IIC_stop();
}
#else

#define SLAVE_ADDR (rt_uint8_t)0x78

rt_err_t IIC_writedata(rt_uint8_t dat, rt_uint8_t identifying)
{
	rt_uint8_t cmd;
	if (identifying == OLED_DATA)
	{
		cmd = 0x40;
		POSx_Rem++;//推算光标位置
	}
	else
	{
		cmd = 0x00;
	}
	while (iic_write_reg(SLAVE_ADDR, cmd, dat)==RT_ERROR)
	{
		ERR_Count++;
		rt_thread_delay(1);
		if(ERR_Count>5)
		{
			return RT_ERROR;
		}
	}
	return RT_EOK;
}

#endif


int OLED_init(void)
{
	Restart_iic_init :

	rt_thread_delay(50);// 暂时挂起线程，没有连接OLED时避免长时间占用CPU时间

	//以下代码与oled通讯
	if (IIC_writedata(0xAE, OLED_COMMAND) == RT_ERROR) //--display off
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0x00,OLED_COMMAND)==RT_ERROR)//---set low column address
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0x10,OLED_COMMAND)==RT_ERROR)//---set high column address
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0x40,OLED_COMMAND)==RT_ERROR)//--set start line address  
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0xB0,OLED_COMMAND)==RT_ERROR)//--set page address
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0x81,OLED_COMMAND)==RT_ERROR) // contract control
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0xFF,OLED_COMMAND)==RT_ERROR)//--128   
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0xA1,OLED_COMMAND)==RT_ERROR)//set segment remap 
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0xA6,OLED_COMMAND)==RT_ERROR)//--normal / reverse
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0xA8,OLED_COMMAND)==RT_ERROR)//--set multiplex ratio(1 to 64)
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0x3F,OLED_COMMAND)==RT_ERROR)//--1/32 duty
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0xC8,OLED_COMMAND)==RT_ERROR)//Com scan direction
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0xD3,OLED_COMMAND)==RT_ERROR)//-set display offset
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0x00,OLED_COMMAND)==RT_ERROR)//
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0xD5,OLED_COMMAND)==RT_ERROR)//set osc division
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0x80,OLED_COMMAND)==RT_ERROR)//
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0xD8,OLED_COMMAND)==RT_ERROR)//set area color mode off
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0x05,OLED_COMMAND)==RT_ERROR)//
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0xD9,OLED_COMMAND)==RT_ERROR)//Set Pre-Charge Period
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0xF1,OLED_COMMAND)==RT_ERROR)//
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0xDA,OLED_COMMAND)==RT_ERROR)//set com pin configuartion
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0x12,OLED_COMMAND)==RT_ERROR)//
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0xDB,OLED_COMMAND)==RT_ERROR)//set Vcomh
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0x30,OLED_COMMAND)==RT_ERROR)//
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0x8D,OLED_COMMAND)==RT_ERROR)//set charge pump enable
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0x14,OLED_COMMAND)==RT_ERROR)//
	{
		goto Restart_iic_init;
	}
	if(IIC_writedata(0xAF,OLED_COMMAND)==RT_ERROR)//--turn on oled panel
	{
		goto Restart_iic_init;
	}
	OLED_clear();
	return RT_EOK;
}

void OLED_clear(void)  
{ 
	for(int i=0;i<8;i++)  
	{  
		IIC_writedata(0xb0+i,OLED_COMMAND);
		IIC_writedata(0x00,OLED_COMMAND);
		IIC_writedata(0x10,OLED_COMMAND);
		for(int n=0;n<128;n++)
		{
			IIC_writedata(0,OLED_DATA);
		}
	}
}

//设定写入光标位置
void OLED_setpos(rt_uint8_t x,rt_uint8_t y)
{
	if(POSx_Rem!=0xFF)
	{//此时不是第一次设置光标位置
		if (x == POSx_Rem && y == POSy_Rem)
		{
			return;
		}
	}
	IIC_writedata(0xb0+y,OLED_COMMAND);
	IIC_writedata(((x&0xf0)>>4)|0x10,OLED_COMMAND);
	IIC_writedata((x&0x0f),OLED_COMMAND);
	POSx_Rem = x;
	POSy_Rem = y;
}

#if USE_HALF_Y

static void HALF_Y_1x8_FIX(char ColorSet,char dat[2])
{
	char datMASK1, datMASK2, fori;
	dat[0] = 0;
	dat[1] = 0;
	datMASK1 = 0x01;
	datMASK2 = 0x02;
	for (fori = 0; fori < 4; fori++)
	{
		if (ColorSet & datMASK1)
		{
			dat[0] |= datMASK2;
		}
		datMASK1 <<= 1;
		datMASK2 <<= 2;
	}
	datMASK1 = 0x10;
	datMASK2 = 0x02;
	for (fori = 0; fori < 4; fori++)
	{
		if (ColorSet & datMASK1)
		{
			dat[1] |= datMASK2;
		}
		datMASK1 <<= 1;
		datMASK2 <<= 2;
	}
}

//对6x8数据进行HALF_Y修正
static void HALF_Y_6x8_FIX(char ch, char (*dat)[2])
{
	char fori;
	for (fori = 0; fori < 6;fori++)
	{
		HALF_Y_1x8_FIX(char6X8[ch][fori], dat[fori]);
	}
}
#endif

//在指定位置绘制矩形，可自行设定绘制图形的沿Y方向的截面状态
//如果使用HALF_Y设置，则此处的colorset只有低4位有效
void OLED_Draw_sqar(char X_StartPOS,char Y_StartPOS, char ColorSet, char Draw_Len)
{
	int fori1;
	char Color_Set_Use[2];
#if (USE_HALF_Y)
	HALF_Y_1x8_FIX(ColorSet,Color_Set_Use);
#else
	Color_Set_Use[0] = ColorSet;
#endif
	//设定起始位置
	OLED_setpos(X_StartPOS, Y_StartPOS);
	//按照截面设置字节进行矩形条绘制
	for (fori1 = 0; fori1 < Draw_Len;fori1++)
	{
		IIC_writedata(Color_Set_Use[0], OLED_DATA);
	}
}

#if USE_HALF_Y
void OLED_show6x8char(rt_uint8_t x, rt_uint8_t y, char character)
{
	char dat[6][2];
	character -= ' '; //消除字库中的偏移量
	OLED_setpos(x, y);
	HALF_Y_6x8_FIX(character,dat);
	for (int a = 0; a < 6; a++)
	{
		IIC_writedata(dat[a][0], OLED_DATA);
	}
	OLED_setpos(x, y+1);
	for (int a = 0; a < 6; a++)
	{
		IIC_writedata(dat[a][1], OLED_DATA);
	}
}
#else
void OLED_show6x8char(rt_uint8_t x,rt_uint8_t y,char character)
{
	character -= ' ';//消除字库中的偏移量
	OLED_setpos(x,y);
	for(int a = 0;a<6;a++)
	{
		IIC_writedata(char6X8[character][a],OLED_DATA);
	}
}
#endif

void OLED_show8x16char(rt_uint8_t x,rt_uint8_t y,char character)
{
	character -= ' ';//消除字库中的偏移量
	OLED_setpos(x,y);
	for(int a = 0;a<8;a++)
	{
		IIC_writedata(char8X16[character][a],OLED_DATA);
	}
	OLED_setpos(x,y+1);
	for(int a = 0;a<8;a++)
	{
		IIC_writedata(char8X16[character][a+8],OLED_DATA);
	}
}
void OLED_show6x8string(rt_uint8_t x,rt_uint8_t y,char* string)
{
	OLED_setpos(x,y);
	int location = 0;
	while(string[location] != '\0')
	{
		for(int a = 0;a<6;a++)
		{
			OLED_show6x8char(x + location*6, y, string[location]);
		}
		location++;
	}
}
void OLED_show8x16string(rt_uint8_t x,rt_uint8_t y,char* string)
{
	int location = 0;
	OLED_setpos(x,y);
	while(string[location] != '\0')
	{
		for(int a = 0;a<8;a++)
		{
			IIC_writedata(char8X16[string[location]-' '][a],OLED_DATA);
		}
		location++;
	}
	location = 0;
	OLED_setpos(x,y+1);
	while(string[location] != '\0')
	{
		for(int a = 0;a<8;a++)
		{
			IIC_writedata(char8X16[string[location]-' '][a+8],OLED_DATA);
		}
		location++;
	}
}
void OLED_show6x8number(rt_uint8_t x,rt_uint8_t y,int number)
{
	OLED_setpos(x,y);
	if(number<0)//显示负号并使数字变正
	{
		for(int a = 0;a<6;a++)
		{
			IIC_writedata(char6X8['-'-' '][a],OLED_DATA);
		}
		number = -number;
	}
	else
	{
		for(int a = 0;a<6;a++)//显示空白字符
		{
			IIC_writedata(char6X8[0][a],OLED_DATA);
		}
	}
	//转换数字为字符串
	char num[20];
	num[19] = '\0';
	int a = 18;
	while(1)
	{
		if(!number)//当数字为0时跳出
		{
			break;
		}
		num[a] = number%10 + '0';
		number /= 10;
		a--;
	}
	OLED_show6x8string(x+6,y,num+a+1);
}
void OLED_show8x16number(rt_uint8_t x,rt_uint8_t y,int number)
{
	static rt_uint8_t lastlen = 0;
	rt_uint8_t len = 0;
	if(number<0)//显示负号并使数字变正
	{
		OLED_show8x16char(x,y,'-');
		number = -number;
	}
	else
	{
		OLED_show8x16char(x,y,' ');
	}
	//转换数字为字符串
	char num[20];
	num[19] = '\0';
	int a = 18;
	while(1)
	{
		if(!number)//当数字为0时跳出
		{
			break;
		}
		len++;
		num[a] = number%10 + '0';
		number /= 10;
		a--;
		len++;
	}
	OLED_show8x16string(x+6,y,num+a+1);
	if(len>lastlen)
	{
		lastlen = len;
	}
	else
	{
		for(;lastlen-len>0;lastlen--)
		{
			OLED_show8x16char(x+6+8*lastlen,y,' ');
		}
	}
}

