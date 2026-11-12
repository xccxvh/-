#include "SPI0_LCD/lcd_init.h"
#include "bsp.h"

/*
 * 车底板 1.69 寸 TFT 的引脚初始化。
 *
 * 这块屏不是核心板 SPI0 那组 OLED 引脚，而是走车底板自己的排线：
 *   PB18=SCL、PB17=SDA、PC0=RES、PA14=DC、PA7=BLK、CS 接地。
 * 其中 PB17/PB18 在 SysConfig 里原本被建议给 ADC1，PA14 原本是 LED1，
 * 因此这里必须重新把它们配置为普通 GPIO 输出，再执行屏复位时序。
 */
void LCD_GPIO_Init(void)
{
    DL_GPIO_initDigitalOutput(LCD_SCL_IOMUX);
    DL_GPIO_initDigitalOutput(LCD_SDA_IOMUX);
    DL_GPIO_initDigitalOutput(LCD_RES_IOMUX);
    DL_GPIO_initDigitalOutput(LCD_DC_IOMUX);
    DL_GPIO_initDigitalOutput(LCD_BLK_IOMUX);

    DL_GPIO_enableOutput(LCD_SCL_PORT, LCD_SCL_PIN);
    DL_GPIO_enableOutput(LCD_SDA_PORT, LCD_SDA_PIN);
    DL_GPIO_enableOutput(LCD_RES_PORT, LCD_RES_PIN);
    DL_GPIO_enableOutput(LCD_DC_PORT, LCD_DC_PIN);
    DL_GPIO_enableOutput(LCD_BLK_PORT, LCD_BLK_PIN);

    /* 空闲电平：SCL 高（SPI 模式 3）、SDA 高、DC 高，背光先打开。 */
    LCD_SCL_Set();
    LCD_SDA_Set();
    LCD_DC_Set();
    LCD_BLK_Set();

    /* 屏复位时序：拉高、拉低、再拉高，各等待 100 ms。 */
    LCD_RES_Set();
    delay_ms(100);
    LCD_RES_Clr();
    delay_ms(100);
    LCD_RES_Set();
}


/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
/*
 * 软件 SPI 写一个字节。
 *
 * 车底板的屏接在 PB18(SCL)/PB17(SDA) 上，不是硬件 SPI0，
 * 因此这里用 GPIO 模拟 SPI：MSB 先发，SCL 空闲高（模式 3），
 * 数据在 SCL 低电平期间准备，SCL 上升沿由屏采样。
 * CS 在排线上接地，恒选中，这里不需要拉 CS。
 */
void LCD_Writ_Bus(u8 dat)
{
    uint8_t bit;

    for (bit = 0U; bit < 8U; bit++)
    {
        if ((dat & 0x80U) != 0U)
        {
            LCD_SDA_Set();
        }
        else
        {
            LCD_SDA_Clr();
        }
        dat <<= 1U;

        LCD_SCL_Clr();
        delay_cycles(10);
        LCD_SCL_Set();
        delay_cycles(10);
    }
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA8(u8 dat)
{
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA(u16 dat)
{
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
void LCD_WR_REG(u8 dat)
{
	LCD_DC_Clr();//写命令
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//写数据
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
	if(USE_HORIZONTAL==0)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+20);
		LCD_WR_DATA(y2+20);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==1)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+20);
		LCD_WR_DATA(y2+20);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==2)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+20);
		LCD_WR_DATA(x2+20);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
	else
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+20);
		LCD_WR_DATA(x2+20);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
}

void LCD_Init(void)
{
	LCD_GPIO_Init();//初始化GPIO
	
	LCD_RES_Clr();//复位
	delay_ms(100);
	LCD_RES_Set();
	delay_ms(100);
	
	LCD_BLK_Set();//打开背光
	delay_ms(100);
	
	//************* Start Initial Sequence **********//
	LCD_WR_REG(0x11); //Sleep out 
	delay_ms(120);              //Delay 120ms 
	//************* Start Initial Sequence **********// 
	LCD_WR_REG(0x36);
	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
	else LCD_WR_DATA8(0xA0);

	LCD_WR_REG(0x3A);			
	LCD_WR_DATA8(0x05);

	LCD_WR_REG(0xB2);			
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x0C); 
	LCD_WR_DATA8(0x00); 
	LCD_WR_DATA8(0x33); 
	LCD_WR_DATA8(0x33); 			

	LCD_WR_REG(0xB7);			
	LCD_WR_DATA8(0x35);

	LCD_WR_REG(0xBB);			
	LCD_WR_DATA8(0x32); //Vcom=1.35V
					
	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x01);

	LCD_WR_REG(0xC3);			
	LCD_WR_DATA8(0x15); //GVDD=4.8V  颜色深度
				
	LCD_WR_REG(0xC4);			
	LCD_WR_DATA8(0x20); //VDV, 0x20:0v

	LCD_WR_REG(0xC6);			
	LCD_WR_DATA8(0x0F); //0x0F:60Hz        	

	LCD_WR_REG(0xD0);			
	LCD_WR_DATA8(0xA4);
	LCD_WR_DATA8(0xA1); 

	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0xD0);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x0E);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x05);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x33);   
	LCD_WR_DATA8(0x48);   
	LCD_WR_DATA8(0x17);   
	LCD_WR_DATA8(0x14);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x34);   

	LCD_WR_REG(0xE1);     
	LCD_WR_DATA8(0xD0);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x0E);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x33);   
	LCD_WR_DATA8(0x48);   
	LCD_WR_DATA8(0x17);   
	LCD_WR_DATA8(0x14);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x34);
	LCD_WR_REG(0x21); 

	LCD_WR_REG(0x29);
} 








