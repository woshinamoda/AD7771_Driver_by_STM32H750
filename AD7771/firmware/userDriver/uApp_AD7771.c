
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include "uApp_AD7771.h"
#include "main.h"
#include "spi.h"

uint8_t reg_list[101];

/* Private user interrupt function handle ----------------------------------------------------*/
/* USER HANDLE BEGIN 0 */

void Read_RegList_init()
{
	for(uint8_t i=0; i<100; i++)
	{
		reg_list[i] = ad7779_read_cmd(i);	
	}
}

//硬件重置
void ad7779_HardwareReset(void)
{
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);		//START同步引脚，菊花链YNSC
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);	
	HAL_Delay(20);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);		//硬件复位
	HAL_Delay(20);	
}

//ad777x 读取寄存器值
uint8_t ad7779_read_cmd(uint8_t reg_addr)
{
	uint8_t set_buf[2];
	uint8_t read_buf[2];
	set_buf[0] = 0x80 | (reg_addr & 0x7F);	//reg_reg
	set_buf[1] = 0x00;											//None
	
	CS_L;
	HAL_SPI_TransmitReceive(&hspi1, set_buf, read_buf, 2, 0xff);
	CS_H;
	
	return read_buf[1];		//返回读取值
}


//ad777x 对应寄存器写入值
uint8_t add7779_write_cmd(uint8_t reg_addr, uint8_t data)
{
	uint8_t set_buf[2];
	uint8_t read_buf[2];
	
	set_buf[0] = (reg_addr & 0x7F);			//write_reg
	set_buf[1] = data;						 			//reg data

	CS_L;
	HAL_SPI_TransmitReceive(&hspi1, set_buf, read_buf, 2, 0xff);
	CS_H;

	return set_buf[1]; //返回写入值	
}

//ad777x 设置SPI 工作模式
void AD7779_Set_SPIOperationMode(ad7779_spi_op_mode mode)
{
	uint8_t gen2_data = ad7779_read_cmd(AD7779_REG_GENERAL_USER_CONFIG_2);
	uint8_t gen3_data = ad7779_read_cmd(AD7779_REG_GENERAL_USER_CONFIG_3);

	switch (mode)
	{
	case AD7779_INT_REG:
		gen2_data &= ~AD7779_SAR_DIAG_MODE_EN;
		gen3_data &= ~AD7779_SPI_SLAVE_MODE_EN;
		break;
	case AD7779_SD_CONV:
		gen2_data &= ~AD7779_SAR_DIAG_MODE_EN;
		gen3_data |= AD7779_SPI_SLAVE_MODE_EN;
		break;
	case AD7779_SAR_CONV:
		gen2_data |= AD7779_SAR_DIAG_MODE_EN;
		gen3_data &= ~AD7779_SPI_SLAVE_MODE_EN;
		break;
	}

	add7779_write_cmd(AD7779_REG_GENERAL_USER_CONFIG_2, gen2_data);
	add7779_write_cmd(AD7779_REG_GENERAL_USER_CONFIG_3, gen3_data);
}


//ad777x 设置电源模式
void AD7779_Set_PowerMode(ad7779_pwr_mode mode)
{
	uint8_t config = ad7779_read_cmd(AD7779_REG_GENERAL_USER_CONFIG_1);

	if (mode == AD7779_HIGH_RES) config |= AD7779_MOD_POWERMODE;
	else config &= ~AD7779_MOD_POWERMODE;

	add7779_write_cmd(AD7779_REG_GENERAL_USER_CONFIG_1, config);
}


//ad777x	过去电源模式
ad7779_pwr_mode AD7779_Get_PowerMode(void)
{
	uint8_t config = ad7779_read_cmd(AD7779_REG_GENERAL_USER_CONFIG_1);

	if (AD7779_FLAGCHECK(config, AD7779_MOD_POWERMODE)) return AD7779_HIGH_RES;
	else return AD7779_LOW_PWR;
}


//ad777x	设置输出速率
void AD7779_Set_OutputRate(double odr_kHz)
{
	uint8_t src_n_lower = 0x00, src_n_upper = 0x00;
	uint8_t src_if_lower = 0x00, src_if_upper = 0x00;

	uint16_t src_n = 0x0000, src_if = 0x0000;
	
	// Get the fmod from mode register
	// High-precision = 2048, Low-power = 512
	double f_mod = 512.0;
	if (AD7779_Get_PowerMode() == AD7779_HIGH_RES) f_mod = 2048.0;

	// Calculate the SRC value
	double n = f_mod / odr_kHz;

	src_n = (uint16_t)n; // Convert to integer for floor
	src_if = (uint16_t)((n - (double)src_n) * 65536.0); // Convert the decimal part

	// Split lower/upper bits
	src_n_lower = src_n & 0x00FF;
	src_n_upper = (src_n & 0xFF00) >> 8;

	src_if_lower = src_if & 0x00FF;
	src_if_upper = (src_if & 0xFF00) >> 8;

	// Write registers
	add7779_write_cmd(AD7779_REG_SRC_N_MSB, src_n_upper);
	add7779_write_cmd(AD7779_REG_SRC_N_LSB, src_n_lower);
	add7779_write_cmd(AD7779_REG_SRC_IF_MSB, src_if_upper);
	add7779_write_cmd(AD7779_REG_SRC_IF_LSB, src_if_lower);

	// Update
	add7779_write_cmd(AD7779_REG_SRC_UPDATE, 0x01);
	HAL_Delay(1);
	add7779_write_cmd(AD7779_REG_SRC_UPDATE, 0x00);
}


//ad777x 设置输出格式
void AD7779_Set_OutputFormat(ad7779_dout_format format)
{
	uint8_t config = ad7779_read_cmd(AD7779_REG_DOUT_FORMAT);

	config &= ~AD7779_DOUT_FORMAT(0x03);
	config |= AD7779_DOUT_FORMAT(format);

	add7779_write_cmd(AD7779_REG_DOUT_FORMAT, config);
}

//ad777x 设置输出头格式
void AD7779_Set_OutputHeaderFormat(ad7779_dout_header_format format)
{
	uint8_t config = ad7779_read_cmd(AD7779_REG_DOUT_FORMAT);

	if (format == AD7779_HEADER_STATUS) config &= ~AD7779_DOUT_HEADER_FORMAT;
	else config |= AD7779_DOUT_HEADER_FORMAT;

	add7779_write_cmd(AD7779_REG_DOUT_FORMAT, config);
}

//ad777x 设置DCLK分频
void AD7779_Set_DCLKDivision(ad7779_dclk_div div)
{
	uint8_t config = ad7779_read_cmd(AD7779_REG_DOUT_FORMAT);

	config &= ~AD7779_DCLK_CLK_DIV(0x07);
	config |= AD7779_DCLK_CLK_DIV(div);

	add7779_write_cmd(AD7779_REG_DOUT_FORMAT, config);
}


//ad777x  设置REF MUX参考
void AD7779_Set_SigmaDelta_ReferenceMultiplexing(ad7779_ref_mux mux)
{
	uint8_t config = ad7779_read_cmd(AD7779_REG_ADC_MUX_CONFIG);

	config &= ~AD7779_REF_MUX_CTRL(0x03);
	config |= AD7779_REF_MUX_CTRL(mux);

	add7779_write_cmd(AD7779_REG_ADC_MUX_CONFIG, config);
}


//ad777x  设置MTR MUX参考
void AD7779_Set_SigmaDelta_ADCMultiplexing(ad7779_mtr_mux mux)
{
	uint8_t config = ad7779_read_cmd(AD7779_REG_ADC_MUX_CONFIG);

	config &= ~AD7779_MTR_MUX_CTRL(0x0F);
	config |= AD7779_MTR_MUX_CTRL(mux);

	add7779_write_cmd(AD7779_REG_ADC_MUX_CONFIG, config);
}

//ad777x  设置SAR MUX参考
void AD7779_Set_SAR_Multiplexing(ad7779_sar_mux mux)
{
	uint8_t config = ad7779_read_cmd(AD7779_REG_GLOBAL_MUX_CONFIG);

	config &= ~AD7779_GLOBAL_MUX_CTRL(0x1F);
	config |= AD7779_GLOBAL_MUX_CTRL(mux);

	add7779_write_cmd(AD7779_REG_GLOBAL_MUX_CONFIG, config);
}


//ad777x  设置GPIO模式
void AD7779_GPIO_SetMode(uint8_t gpio_pin, uint8_t gpio_mode)
{
	uint8_t config = ad7779_read_cmd(AD7779_REG_GPIO_CONFIG);

	if (gpio_mode == GPIO_MODE_OUTPUT_PP) config |= gpio_pin;
	else if (gpio_mode == GPIO_MODE_INPUT) config &= ~gpio_pin;

	add7779_write_cmd(AD7779_REG_GPIO_CONFIG, config);
}


//ad777x	写GPIO引脚
void AD7779_GPIO_WritePin(uint8_t gpio_pin, GPIO_PinState state)
{
	uint8_t config = ad7779_read_cmd(AD7779_REG_GPIO_DATA);

	if (state == GPIO_PIN_SET) config |= gpio_pin;
	else config &= ~gpio_pin;

	add7779_write_cmd(AD7779_REG_GPIO_CONFIG, config);
}

//ad777x	读GPIO引脚高低
GPIO_PinState AD7779_GPIO_ReadPin(uint8_t gpio_pin)
{
	uint8_t config = ad7779_read_cmd(AD7779_REG_GPIO_CONFIG);

	if (AD7779_FLAGCHECK(config, gpio_pin << 3)) return GPIO_PIN_SET;
	else return GPIO_PIN_RESET;
}


//ad777x	获取自检寄存器值
uint16_t AD7779_Get_GeneralError(void)
{
	uint8_t gen1 = ad7779_read_cmd(AD7779_REG_GEN_ERR_REG_1);
	uint8_t gen2 = ad7779_read_cmd(AD7779_REG_GEN_ERR_REG_2);

	return ((uint16_t)gen2 << 8) + gen1;
}


//ad777x	设置LDO clolck检测寄存器
void AD7779_Set_GeneralErrorCheckEnable(uint16_t error_en, ad7779_ldo_psm_test_en psm_en, ad7779_ldo_psm_trip_test_en psm_trip_en)
{
	uint8_t gen1 = error_en & 0x00FF;
	uint8_t gen2 = (error_en & 0x2000) >> 8;

	gen2 |= AD7779_LDO_PSM_TEST_EN(psm_en);
	gen2 |= AD7779_LDO_PSM_TRIP_TEST_EN(psm_trip_en);

	add7779_write_cmd(AD7779_REG_GEN_ERR_REG_1, gen1);
	add7779_write_cmd(AD7779_REG_GEN_ERR_REG_2, gen2);
}


//ad777x	使能对应输出通道 时钟
void AD7779_EnableChannel(ad7779_ch channel)
{
	uint8_t ch_disable = ad7779_read_cmd(AD7779_REG_CH_DISABLE);

	ch_disable &= ~AD7779_CH_DISABLE(channel);

	add7779_write_cmd(AD7779_REG_CH_DISABLE, ch_disable);
}

//ad777x	关闭对应输出通道 时钟
void AD7779_DisableChannel(ad7779_ch channel)
{
	uint8_t ch_disable = ad7779_read_cmd(AD7779_REG_CH_DISABLE);

	ch_disable |= AD7779_CH_DISABLE(channel);

	add7779_write_cmd(AD7779_REG_CH_DISABLE, ch_disable);
}

//ad777x 设置对应通道状态
void AD7779_Set_ChannelEnable(ad7779_ch channel, ad7779_state state)
{
	if (state == AD7779_ENABLE) AD7779_EnableChannel(channel);
	else AD7779_DisableChannel(channel);
}

//ad777x 设置对应通道增益
void AD7779_Set_ChannelPGAGain(ad7779_ch channel, ad7779_gain gain)
{
	uint8_t config = ad7779_read_cmd(AD7779_REG_CH_CONFIG(channel));

	config &= ~AD7779_CH_GAIN(0x03);
	config |= AD7779_CH_GAIN(gain);

	add7779_write_cmd(AD7779_REG_CH_CONFIG(channel), config);
}

//ad777x 设置通道参考监听
void AD7779_Set_ChannelReferenceMonitor(ad7779_ch channel, ad7779_state state)
{
	uint8_t config = ad7779_read_cmd(AD7779_REG_CH_CONFIG(channel));

	if (state == AD7779_ENABLE) config |= AD7779_CH_REF_MONITOR;
	else config &= ~AD7779_CH_REF_MONITOR;

	add7779_write_cmd(AD7779_REG_CH_CONFIG(channel), config);
}

//ad777x 设置对应通道MUX RX
void AD7779_Set_ChannelMuxRxMode(ad7779_ch channel, ad7779_state state)
{
	uint8_t config = ad7779_read_cmd(AD7779_REG_CH_CONFIG(channel));

	if (state == AD7779_ENABLE) config |= AD7779_CH_RX;
	else config &= ~AD7779_CH_RX;

	add7779_write_cmd(AD7779_REG_CH_CONFIG(channel), config);
}

//ad777x 设置对应偏移的通道
void AD7779_Set_ChannelSyncOffset(ad7779_ch channel, uint8_t offset)
{
	add7779_write_cmd(AD7779_REG_CH_SYNC_OFFSET(channel), offset);
}

//ad777x	设置对应通道的漂移值
void AD7779_Set_ChannelOffset(ad7779_ch channel, uint32_t offset)
{
	uint8_t lower, mid, upper;
	lower = offset & 0x0000FF;
	mid = (offset & 0x00FF00) >> 8;
	upper = (offset & 0xFF0000) >> 16;

	add7779_write_cmd(AD7779_REG_CH_OFFSET_UPPER_BYTE(channel), upper);
	add7779_write_cmd(AD7779_REG_CH_OFFSET_MID_BYTE(channel), mid);
	add7779_write_cmd(AD7779_REG_CH_OFFSET_LOWER_BYTE(channel), lower);
}

//ad777x  设置对应通道的增益
void AD7779_Set_ChannelGain(ad7779_ch channel, uint32_t gain)
{
	uint8_t lower, mid, upper;
	lower = gain & 0x0000FF;
	mid = (gain & 0x00FF00) >> 8;
	upper = (gain & 0xFF0000) >> 16;

	add7779_write_cmd(AD7779_REG_CH_GAIN_UPPER_BYTE(channel), upper);
	add7779_write_cmd(AD7779_REG_CH_GAIN_MID_BYTE(channel), mid);
	add7779_write_cmd(AD7779_REG_CH_GAIN_LOWER_BYTE(channel), lower);
}

//ad777x		获取通道错误信息
uint8_t AD7779_Get_ChannelError(ad7779_ch channel)
{
	return ad7779_read_cmd(AD7779_REG_CH_ERR_REG(channel));
}

//ad777x		获取通道SAT DSP错误信息
uint8_t AD7779_Get_ChannelDSPError(ad7779_ch channel)
{
	switch (channel)
	{
	case 0:
	case 1:
		return ad7779_read_cmd(AD7779_REG_CH0_1_SAT_ERR);
		break;
	case 2:
	case 3:
		return ad7779_read_cmd(AD7779_REG_CH2_3_SAT_ERR);
		break;
	case 4:
	case 5:
		return ad7779_read_cmd(AD7779_REG_CH4_5_SAT_ERR);
		break;
	case 6:
	case 7:
		return ad7779_read_cmd(AD7779_REG_CH6_7_SAT_ERR);
		break;
	}
}

//ad777x  0-7错误通道使能
void AD7779_Set_ChannelErrorCheckEnable(uint8_t error_en)
{
	add7779_write_cmd(AD7779_REG_CHX_ERR_REG_EN, error_en);
}






































uint8_t reg_buf;
void eCon_ad7779_init()
{
	//读取GEN_ERR_REG_2的RESET位
	if (AD7779_FLAGCHECK(ad7779_read_cmd(AD7779_REG_GEN_ERR_REG_2), AD7779_GEN2_ERR_RESET_DETECTED))
		// 设置SPI模式位注册操作模式
		AD7779_Set_SPIOperationMode(AD7779_INT_REG);
	
	AD7779_Set_PowerMode(AD7779_HIGH_RES);		//设置电源模式为高分辨率
	AD7779_Set_OutputRate(1);									//设置采样率1Khz，计算参考P55 . 使能REF_OUT会降低一半，实际为 2/1 = 1000Hz
		
	AD7779_Set_OutputFormat(AD7779_DOUT_FORMAT_1LINE);		//Dout 1线输出
	AD7779_Set_OutputHeaderFormat(AD7779_HEADER_STATUS);	//输出头为status
	AD7779_Set_DCLKDivision(AD7779_DCLK_DIV_4);						//DCLK分频4   8.192/4 = 2.048Mhz

	reg_buf = ad7779_read_cmd(AD7779_REG_GENERAL_USER_CONFIG_1);
	add7779_write_cmd(AD7779_REG_GENERAL_USER_CONFIG_1, 0x74);					//使能REFout 用于供电REF+ REF- 做为 0-3 4-7通道输入参考		
	AD7779_Set_SigmaDelta_ReferenceMultiplexing(AD7779_REFMUX_EXTREF);		//参考P44页，用万用表点取看是否2.5V

	
	add7779_write_cmd(AD7779_REG_CH_DISABLE, 0x00);			//使能8个通道时钟
	for(uint8_t i=0; i<8; i++)
	{
		add7779_write_cmd(AD7779_REG_CH_CONFIG(i), 0x00);				//Gain = 1; 关闭REF_MON和RX
		add7779_write_cmd(AD7779_REG_CH_SYNC_OFFSET(i), 0x00);	//offset = 0,所有通道偏移置0
		AD7779_Set_ChannelOffset(i, 0);													//所有通道漂移值也改为0
	}
	
	
}


/* USER HANDLE END 0 */


uint8_t ReadBuf[32];
int boardChannelDataInt[9] = {0xAABBCCDD};

void HAL_GPIO_EXTI_Callback(uint16_t	GPIO_Pin)
{
	if(GPIO_Pin == DRDY_Pin)
	{
		HAL_SPI_Receive(&hspi4, ReadBuf, 32, 0xff);

		for(uint8_t i=1; i<9; i++)
		{
			for (int j = 0; j < 3; j++)
			{ //  read 24 bits of channel data in 8 3 byte chunks
				uint8_t inByte;
				inByte = ReadBuf[j+1 + (i-1)*4];
				boardChannelDataInt[i] = (boardChannelDataInt[i] << 8) | inByte; // int data goes here
			}
		}

		
		for(uint8_t i=1; i<9; i++)
		{
			if ((boardChannelDataInt[i] & 0x00800000) == 0x00800000)
			{
				boardChannelDataInt[i] |= 0xFF000000;
			}else
			{
				boardChannelDataInt[i] &= 0x00FFFFFF;
			}
		}

		for(uint8_t i=1; i<9; i++)
		{
			boardChannelDataInt[i]  = boardChannelDataInt[i] * 0.298023223;  //298023223   3933906
		}	

		if(HAL_UART_Transmit(&huart3, boardChannelDataInt, 36,  1) != HAL_OK)
		{
			Error_Handler();
		}

	}

}




















