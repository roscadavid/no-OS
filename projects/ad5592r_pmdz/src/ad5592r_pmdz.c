/***************************************************************************//**
 *   @file   ad469x_fmcz.c
 *   @brief  Implementation of Main Function.
 *   @author Cristian Pop (cristian.pop@analog.com)
 ********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdio.h>
#include <inttypes.h>
#include <xil_cache.h>
#include "spi_engine.h"
#include "no_os_pwm.h"
#include "axi_pwm_extra.h"

#include "no_os_error.h"
#include "clk_axi_clkgen.h"
#include "no_os_gpio.h"
#include "xilinx_gpio.h"
#include "parameters.h"

#include "xilinx_uart.h"
#include "xil_cache.h"


#include "ad5592r.h"
#include "ad5592r-base.h"
#include "axi_adc_core.h"


/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
//#define AD5592R_EVB_SAMPLE_NO		1000
//#define TOTAL_CH					3

#define SPI_DEVICE_ID				0


//#define AXI_ADC_DATA_CHANNEL_0		0x0042C
//#define AXI_ADC_DATA_CHANNEL_1		0x0046C
//#define AXI_ADC_DATA_CHANNEL_2		0x004AC
//#define AXI_ADC_DATA_CHANNEL_3		0x004EC
//#define AXI_ADC_DATA_CHANNEL_4		0x0052C
//#define AXI_ADC_DATA_CHANNEL_5		0x0056C


#define RX_CORE_BASEADDR            0x44a00000

int main(void)
{
	printf("\n\n!!!\tStarting...\t!!!\n\n");
	//uint16_t value, value1, value2, value3, value4, value5;
	int ret;
	uint32_t *buf = ADC_DDR_BASEADDR;
	uint16_t samples_0[6][3200];
	uint16_t samples_1[6][3200];
//	uint32_t 		*data;
//	uint32_t* buf = malloc(1024 * sizeof(uint32_t));
//	uint32_t buf[6];
//	uint32_t buf0[1024];
//	uint32_t buf1[1024];
//	uint32_t buf2[1024];
//	uint32_t buf3[1024];
//	uint32_t buf4[1024];
//	uint32_t buf5[1024];

    // Definirea parametrilor pentru SPI Engine
    struct spi_engine_offload_init_param spi_engine_offload_init_param = {
        .offload_config = OFFLOAD_RX_EN,
        .rx_dma_baseaddr = AD5592R_DMA_BASEADDR,

    };

    struct spi_engine_init_param spi_eng_init_param = {
            .ref_clk_hz = AD5592R_SPI_ENG_REF_CLK_FREQ_HZ,
            .type = SPI_ENGINE,
            .spi_engine_baseaddr = AD5592R_SPI_ENGINE_BASEADDR,
            .cs_delay = 0,
            .data_width = 16,
        };
    struct axi_clkgen_init clkgen_init = {
    			.name = "rx_clkgen",
    			.base = RX_CLKGEN_BASEADDR,
    			.parent_rate = 100000000,
    		};
    struct axi_pwm_init_param axi_pwm_init = {
    			.base_addr = AXI_PWMGEN_BASEADDR,
    			.ref_clock_Hz = 160000000,
    			.channel = 0,
    		};
	struct no_os_pwm_init_param trigger_pwm_init = {
		.period_ns = 5000,//1000	/* 1Mhz */ //f=400KSP
		.duty_cycle_ns = 10,
		.polarity = NO_OS_PWM_POLARITY_HIGH,
		.platform_ops = &axi_pwm_ops,
		.extra = &axi_pwm_init,
	};
    struct no_os_spi_init_param spi_init = {
            .chip_select = AD5592R_SPI_CS,
            .max_speed_hz = 20000000,    //Sclk max din dataSheet in loc de 80Mz vom pune 20Mz
            .mode = NO_OS_SPI_MODE_3,
            .platform_ops = &spi_eng_platform_ops,
			.extra = (void*)&spi_eng_init_param,
        };

	struct ad5592r_dev *my_ad5592;

	struct ad5592r_init_param default_init_param = {
			.spi_init = &spi_init,
			.offload_init_param = &spi_engine_offload_init_param,
			.trigger_pwm_init = &trigger_pwm_init,
			.clkgen_init = &clkgen_init,
			.axi_clkgen_rate = 160000000,
			.reg_access_speed = 20000000,   //Sclk max din DataSheet
			.reg_data_width = 8,
			.capture_data_width = 16,  //posibil 12, dar nu sigur
			.int_ref = true,
			.dcache_invalidate_range = (void (*)(uint32_t, uint32_t))Xil_DCacheInvalidateRange,

	    };

	Xil_ICacheEnable();
	Xil_DCacheEnable();

	ret = ad5592r_init(&my_ad5592, &default_init_param);
	if (ret < 0) {
		printf("Couldn't initialize ad5592r driver with SPI engine!\n");
		return ret;
	}

	printf("SUCCES!\n");

	//momentan alocate static
		memset(buf, 0, 3200 * sizeof(uint32_t));
			// Citește câte 1024 samples de pe fiecare canal (0–5)
		ret = ad5592r_read_data_spi_engine_offload(my_ad5592, 0, buf, 3200);
		if (ret < 0) {
			printf("Error reading ADC data from channel 0!\n");
			return ret;
		}
		for ( int i = 0; i < 3200 ; i++){
				samples_0[0][i] = buf [i] & 0xFFF;
				samples_1[0][i] = (buf[i]>>16) & 0xFFF;

			}
		memset(buf, 0, 3200 * sizeof(uint32_t));
			// Citește câte 1024 samples de pe fiecare canal (0–5)
		ret = ad5592r_read_data_spi_engine_offload(my_ad5592, 1, buf, 3200);
		if (ret < 0) {
			printf("Error reading ADC data from channel 1!\n");
			return ret;
		}
		for ( int i = 0; i < 3200 ; i++){
				samples_0[1][i] = buf [i] & 0xFFF;
				samples_1[1][i] = (buf[i]>>16) & 0xFFF;

			}
		memset(buf, 0, 3200 * sizeof(uint32_t));
			// Citește câte 1024 samples de pe fiecare canal (0–5)
		ret = ad5592r_read_data_spi_engine_offload(my_ad5592, 2, buf, 3200);
		if (ret < 0) {
			printf("Error reading ADC data from channel 2!\n");
			return ret;
		}
		for ( int i = 0; i < 3200 ; i++){
				samples_0[2][i] = buf [i] & 0xFFF;
				samples_1[2][i] = (buf[i]>>16) & 0xFFF;

			}
		memset(buf, 0, 3200 * sizeof(uint32_t));
			// Citește câte 1024 samples de pe fiecare canal (0–5)
		ret = ad5592r_read_data_spi_engine_offload(my_ad5592, 3, buf, 3200);
		if (ret < 0) {
			printf("Error reading ADC data from channel 3!\n");
			return ret;
		}
		for ( int i = 0; i < 3200 ; i++){
				samples_0[3][i] = buf [i] & 0xFFF;
				samples_1[3][i] = (buf[i]>>16) & 0xFFF;

			}
		memset(buf, 0, 3200 * sizeof(uint32_t));
			// Citește câte 1024 samples de pe fiecare canal (0–5)
		ret = ad5592r_read_data_spi_engine_offload(my_ad5592, 4, buf, 3200);
		if (ret < 0) {
			printf("Error reading ADC data from channel 4!\n");
			return ret;
		}
		for ( int i = 0; i < 3200 ; i++){
				samples_0[4][i] = buf [i] & 0xFFF;
				samples_1[4][i] = (buf[i]>>16) & 0xFFF;

			}
		memset(buf, 0, 3200 * sizeof(uint32_t));
			// Citește câte 1024 samples de pe fiecare canal (0–5)
		ret = ad5592r_read_data_spi_engine_offload(my_ad5592, 5, buf, 3200);
		if (ret < 0) {
			printf("Error reading ADC data from channel 5!\n");
			return ret;
		}
		for ( int i = 0; i < 3200 ; i++){
				samples_0[5][i] = buf [i] & 0xFFF;
				samples_1[5][i] = (buf[i]>>16) & 0xFFF;
			}

		for ( int i = 0; i < 3200 ; i++){
			    printf("ADC0 sample: %d, %d, %d, %d, %d, %d \n", samples_0[0][i], samples_0[1][i], samples_0[2][i], samples_0[3][i], samples_0[4][i], samples_0[5][i] );
			    printf("ADC1 sample: %d, %d, %d, %d, %d, %d \n", samples_1[0][i], samples_1[1][i], samples_1[2][i], samples_1[3][i], samples_1[4][i], samples_1[5][i] );
			}

//	for (int i = 0; i < 5; i++)
//	{
//		ad5592r_read_adc(my_ad5592, 0, &value);
//		ad5592r_read_adc(my_ad5592, 1, &value1);
//		ad5592r_read_adc(my_ad5592, 2, &value2);
//		ad5592r_read_adc(my_ad5592, 3, &value3);
//		ad5592r_read_adc(my_ad5592, 4, &value4);
//		ad5592r_read_adc(my_ad5592, 5, &value5);
//
//		printf("ADC sample:  %d,%d,%d,%d,%d,%d\n ", value & 0x0fff, value1 & 0x0fff, value2 & 0x0fff, value3 & 0x0fff, value4 & 0x0fff, value5 & 0x0fff);
//	}

	printf("\n\n!!!\tEnd...\t!!!\n\n");

	return 0;
}




