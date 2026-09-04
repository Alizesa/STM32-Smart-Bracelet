#include "stm32f10x.h"                  // 设备头文件
#include "Delay.h"
#include "PinMap.h"

/*引脚配置*/

/**
  * 函    数：MyI2C写SCL引脚的电平
  * 参    数：BitValue 协议层传入的当前需要写入SCL的电平值，范围0~1
  * 返 回 值：无
  * 说    明：此函数需要用户实现，当BitValue为0时，需要置SCL为低电平，当BitValue为1时，需要置SCL为高电平
  */
void MyI2C_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)BitValue);		//根据BitValue写入SCL引脚的电平
	Delay_us(10);												//延时10us，防止时序频率超过要求
}

void MX30102_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(MX30102_I2C_PORT, MX30102_I2C_SCL, (BitAction)BitValue);
	Delay_us(10);
}

/**
  * 函    数：MyI2C写SDA引脚的电平
  * 参    数：BitValue 协议层传入的当前需要写入SDA的电平值，范围0~1
  * 返 回 值：无
  * 说    明：此函数需要用户实现，当BitValue为0时，需要置SDA为低电平，当BitValue为1时，需要置SDA为高电平
  */
void MyI2C_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)BitValue);		//根据BitValue写入SDA引脚的电平，BitValue要实现分0和1的区分
	Delay_us(10);												//延时10us，防止时序频率超过要求
}

void MX30102_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(MX30102_I2C_PORT, MX30102_I2C_SDA, (BitAction)BitValue);
	Delay_us(10);
}

/**
  * 函    数：MyI2C读SDA引脚的电平
  * 参    数：无
  * 返 回 值：协议层需要得到的当前SDA的电平值，范围0~1
  * 说    明：此函数需要用户实现，当前SDA为低电平时返回0，当前SDA为高电平时返回1
  */
uint8_t MyI2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);		//读取SDA电平
	Delay_us(10);												//延时10us，防止时序频率超过要求
	return BitValue;											//返回SDA电平
}

uint8_t MX30102_R_SDA(void)
{
	uint8_t BitValue;
	BitValue=GPIO_ReadInputDataBit(MX30102_I2C_PORT,MX30102_I2C_SDA);
	Delay_us(10);
	return BitValue;
}

/**
  * 函    数：MyI2C初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：此函数需要用户实现，实现SCL和SDA引脚的初始化
  */
void MyI2C_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//开启GPIOB的时钟

	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;		//开漏输出模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//将PB10和PB11引脚初始化为开漏输出

	/*设置默认电平*/
	GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);			//将PB10和PB11引脚初始化后默认置为高电平，释放总线
}

void MX30102_I2C_Init(void)
{
	RCC_APB2PeriphClockCmd(MX30102_I2C_RCC,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin=MX30102_I2C_SCL|MX30102_I2C_SDA;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(MX30102_I2C_PORT,&GPIO_InitStructure);

	GPIO_SetBits(MX30102_I2C_PORT,MX30102_I2C_SCL|MX30102_I2C_SDA);
}

/*协议层*/

/**
  * 函    数：MyI2C起始
  * 参    数：无
  * 返 回 值：无
  */
void MyI2C_Start(void)
{
	MyI2C_W_SDA(1);							//释放SDA，确保SDA为高电平
	MyI2C_W_SCL(1);							//释放SCL，确保SCL为高电平
	MyI2C_W_SDA(0);							//在SCL高电平期间，拉低SDA，产生起始信号
	MyI2C_W_SCL(0);							//起始后把SCL也拉低，即为了占用总线，也为了方便总线时序的拼接
}

void MX30102_I2C_Start(void)
{
	MX30102_W_SDA(1);
	MX30102_W_SCL(1);
	MX30102_W_SDA(0);
	MX30102_W_SCL(0);
}

/**
  * 函    数：MyI2C终止
  * 参    数：无
  * 返 回 值：无
  */
void MyI2C_Stop(void)
{
	MyI2C_W_SDA(0);							//拉低SDA，确保SDA为低电平
	MyI2C_W_SCL(1);							//释放SCL，使SCL呈现高电平
	MyI2C_W_SDA(1);							//在SCL高电平期间，释放SDA，产生终止信号
}

void MX30102_I2C_Stop(void)
{
	MX30102_W_SDA(0);
	MX30102_W_SCL(1);
	MX30102_W_SDA(1);
}

/**
  * 函    数：MyI2C发送一个字节
  * 参    数：Byte 要发送的一个字节数据，范围：0x00~0xFF
  * 返 回 值：无
  */
void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)				//循环8次，依次发送数据的每一位
	{
		/*这里利用!!将任意非0值统一转换为1，即!!(0) = 0，!!(非0) = 1*/
		MyI2C_W_SDA(!!(Byte & (0x80 >> i)));//使用掩码的方式取出Byte的指定一位数据并写入到SDA
		MyI2C_W_SCL(1);						//释放SCL，从机在SCL高电平期间读取SDA
		MyI2C_W_SCL(0);						//拉低SCL，主机开始发送下一位数据
	}
}

void MX30102_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for(i=0;i<8;i++)
	{
		MX30102_W_SDA(!!(Byte & (0x80 >> i)));
		MX30102_W_SCL(1);
		MX30102_W_SCL(0);
	}
}

/**
  * 函    数：MyI2C接收一个字节
  * 参    数：无
  * 返 回 值：接收到的一个字节数据，范围：0x00~0xFF
  */
uint8_t MyI2C_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;					//定义接收的数据，并赋初值0x00，此处赋初值0是为了防止数据不完整时，误读数
	MyI2C_W_SDA(1);							//释放SDA，确保SDA为高电平，方便从机发送数据
	for (i = 0; i < 8; i ++)				//循环8次，依次接收数据的每一位
	{
		MyI2C_W_SCL(1);						//释放SCL，主机在SCL高电平期间读取SDA
		if (MyI2C_R_SDA()){Byte |= (0x80 >> i);}	//读取SDA数据，并存储到Byte中
													//当SDA为1时，把变量的一位置1，当SDA为0时，不改变变量的初值0
		MyI2C_W_SCL(0);						//拉低SCL，从机在SCL低电平期间写入SDA
	}
	return Byte;							//返回接收到的一个字节数据
}

uint8_t MX30102_ReceiveByte(void)
{
	uint8_t i,Byte=0x00;
	MX30102_W_SDA(1);
	for(i=0;i<8;i++)
	{
		MX30102_W_SCL(1);
		if(MX30102_R_SDA())
		{
			Byte|=(0x80>>i);
		}
		MX30102_W_SCL(0);
	}
	return Byte;
}

/**
  * 函    数：MyI2C发送应答位
  * 参    数：Byte 要发送的应答位，范围：0~1，0表示应答，1表示非应答
  * 返 回 值：无
  */
void MyI2C_SendAck(uint8_t AckBit)
{
	MyI2C_W_SDA(AckBit);					//将应答位数据放到SDA上
	MyI2C_W_SCL(1);							//释放SCL，从机在SCL高电平期间读取应答位
	MyI2C_W_SCL(0);							//拉低SCL，开始下一个时序
}

void MX30102_I2C_SendAck(uint8_t AckBit)
{
	MX30102_W_SDA(AckBit);
	MX30102_W_SCL(1);
	MX30102_W_SCL(0);
}

/**
  * 函    数：MyI2C接收应答位
  * 参    数：无
  * 返 回 值：接收到的应答位，范围：0~1，0表示应答，1表示非应答
  */
uint8_t MyI2C_ReceiveAck(void)
{
	uint8_t AckBit;							//定义应答位变量
	MyI2C_W_SDA(1);							//释放SDA，确保SDA为高电平，方便从机发送应答位
	MyI2C_W_SCL(1);							//释放SCL，主机在SCL高电平期间读取SDA
	AckBit = MyI2C_R_SDA();					//读取应答位存到变量中
	MyI2C_W_SCL(0);							//拉低SCL，开始下一个时序
	return AckBit;							//返回应答位
}

uint8_t MX30102_I2C_ReceiveAck(void)
{
	uint8_t AckBit;
	MX30102_W_SDA(1);
	MX30102_W_SCL(1);
	AckBit=MX30102_R_SDA();
	MX30102_W_SCL(0);
	return AckBit;
}
