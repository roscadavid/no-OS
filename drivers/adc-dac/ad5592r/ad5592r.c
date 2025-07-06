/***************************************************************************//**
 *   @file   ad5592r.c
 *   @brief  Implementation of AD5592R driver.
 *   @author Mircea Caprioru (mircea.caprioru@analog.com)
********************************************************************************
 * Copyright 2018, 2020(c) Analog Devices, Inc.
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
#include "no_os_error.h"
#include "ad5592r-base.h"
#include "ad5592r.h"
#include "spi_engine.h"
#include "no_os_delay.h"
#include "no_os_util.h"
#include "no_os_alloc.h"
#include "no_os_pwm.h"
#include "clk_axi_clkgen.h"
#include "xil_cache.h"

const struct ad5592r_rw_ops ad5592r_rw_ops = {
	.write_dac = ad5592r_write_dac,
	.read_adc = ad5592r_read_adc,
	.multi_read_adc = ad5592r_multi_read_adc,
	.reg_write = ad5592r_reg_write,
	.reg_read = ad5592r_reg_read,
	.gpio_read = ad5592r_gpio_read,
};


/**
 * Write NOP and read value.
 *
 * @param dev - The device structure.
 * @param buf - buffer where to read
 * @return 0 in case of success, negative error code otherwise
 */
static int32_t ad5592r_spi_wnop_r16(struct ad5592r_dev *dev, uint16_t *buf)
{
	int32_t ret;
	uint16_t spi_msg_nop = 0; /* NOP */
	ret = spi_engine_set_transfer_width(dev->spi, dev->reg_data_width);
		if (ret != 0)
			return ret;

		spi_engine_set_speed(dev->spi, dev->reg_access_speed);

	ret = no_os_spi_write_and_read(dev->spi, (uint8_t *)&spi_msg_nop,
				       sizeof(spi_msg_nop));
	if (ret < 0)
		return ret;
	ret = spi_engine_set_transfer_width(dev->spi,
						    dev->capture_data_width);
		if (ret != 0)
			return ret;

		spi_engine_set_speed(dev->spi, dev->spi->max_speed_hz);
		*buf = swab16(spi_msg_nop);
	return ret;
}



/**
 * Write DAC channel.
 *
 * @param dev - The device structure.
 * @param chan - The channel number.
 * @param value - DAC value
 * @return 0 in case of success, negative error code otherwise
 */
int32_t ad5592r_write_dac(struct ad5592r_dev *dev, uint8_t chan,
			  uint16_t value)
{
	int32_t ret;
	if (!dev)
		return -1;
	ret = spi_engine_set_transfer_width(dev->spi, dev->reg_data_width);
	if (ret != 0)
			return ret;

	spi_engine_set_speed(dev->spi, dev->reg_access_speed);

	dev->spi_msg = swab16( NO_OS_BIT(15) | (uint16_t)(chan << 12) | value);

	ret = no_os_spi_write_and_read(dev->spi, (uint8_t *)&dev->spi_msg,
					sizeof(dev->spi_msg));
	ret = spi_engine_set_transfer_width(dev->spi,
						    dev->capture_data_width);
	if (ret != 0)
			return ret;

	spi_engine_set_speed(dev->spi, dev->spi->max_speed_hz);

	return ret;
}

/**
 * Read ADC channel.
 *
 * @param dev - The device structure.
 * @param chan - The channel number.
 * @param value - ADC value
 * @return 0 in case of success, negative error code otherwise
 */
int32_t ad5592r_read_adc(struct ad5592r_dev *dev, uint8_t chan,
			 uint16_t *value)
{
	int32_t ret;

	if (!dev)
		return -1;
	ret = spi_engine_set_transfer_width(dev->spi, dev->reg_data_width);
	if (ret != 0)
		return ret;

	spi_engine_set_speed(dev->spi, dev->reg_access_speed);

	dev->spi_msg = swab16((uint16_t)(AD5592R_REG_ADC_SEQ << 11) |
				      NO_OS_BIT(chan));

	ret = no_os_spi_write_and_read(dev->spi, (uint8_t *)&dev->spi_msg,
					       sizeof(dev->spi_msg));
	if (ret < 0)
		return ret;
	ret = spi_engine_set_transfer_width(dev->spi,
						    dev->capture_data_width);
		if (ret != 0)
			return ret;

		spi_engine_set_speed(dev->spi, dev->spi->max_speed_hz);

	/*
	 * Invalid data:
	 * See Figure 40. Single-Channel ADC Conversion Sequence
	 */
	ret = ad5592r_spi_wnop_r16(dev, &dev->spi_msg);
	if (ret < 0)
		return ret;

	ret = ad5592r_spi_wnop_r16(dev, &dev->spi_msg);
	if (ret < 0)
		return ret;

	*value = dev->spi_msg;
//	printf("%d\n", *value);
//		uint16_t temp_reg_val;
//		ad5592r_reg_write(dev, 0x1, 0x02);
//		ad5592r_reg_write(dev, 0x0, 0x00);
//		ad5592r_reg_read(dev, 0x0, &temp_reg_val);

	return 0;
}

/**
 * Read Multiple ADC Channels.
 *
 * @param dev - The device structure.
 * @param chans - The ADC channels to be readback
 * @param values - ADC value array
 * @return 0 in case of success, negative error code otherwise
 */
int32_t ad5592r_multi_read_adc(struct ad5592r_dev *dev, uint16_t chans,
			       uint16_t *values)
{
	int32_t ret;
	uint8_t samples;
	uint8_t i;

	if (!dev)
		return -1;
	ret = spi_engine_set_transfer_width(dev->spi, dev->reg_data_width);
		if (ret != 0)
			return ret;

	spi_engine_set_speed(dev->spi, dev->reg_access_speed);
	samples = no_os_hweight16(chans);

	dev->spi_msg = swab16((uint16_t)(AD5592R_REG_ADC_SEQ << 11) | chans);

	ret = no_os_spi_write_and_read(dev->spi, (uint8_t *)&dev->spi_msg,
				       sizeof(dev->spi_msg));
	if (ret < 0)
		return ret;
	ret = spi_engine_set_transfer_width(dev->spi,
					    dev->capture_data_width);
	if (ret != 0)
		return ret;

	spi_engine_set_speed(dev->spi, dev->spi->max_speed_hz);
	/*
	 * Invalid data:
	 * See Figure 40. Single-Channel ADC Conversion Sequence
	 */
	ret = ad5592r_spi_wnop_r16(dev, &dev->spi_msg);
	if (ret < 0)
		return ret;

	for (i = 0; i < samples; i++) {
		ret = ad5592r_spi_wnop_r16(dev, &dev->spi_msg);
		if (ret < 0)
			return ret;
		values[i] = dev->spi_msg;
	}

	return 0;
}

/**
 * Write register.
 *
 * @param dev - The device structure.
 * @param reg - The register address.
 * @param value - register value
 * @return 0 in case of success, negative error code otherwise
 */

int32_t ad5592r_reg_write(struct ad5592r_dev *dev, uint8_t reg, uint16_t value)
{
	int32_t ret;

	if (!dev)
		return -1;

	ret = spi_engine_set_transfer_width(dev->spi, dev->reg_data_width);
	if (ret != 0)
		return ret;

	spi_engine_set_speed(dev->spi, dev->reg_access_speed);

	dev->spi_msg = swab16((reg << 11) | value);
	ret = no_os_spi_write_and_read(dev->spi, (uint8_t *)&dev->spi_msg,
					sizeof(dev->spi_msg));

	ret = spi_engine_set_transfer_width(dev->spi, dev->capture_data_width);
	if (ret != 0)
		return ret;

	spi_engine_set_speed(dev->spi, dev->spi->max_speed_hz);

	return ret;
}

/**
 * Read register.
 *
 * @param dev - The device structure.
 * @param reg - The register address.
 * @param value - register value
 * @return 0 in case of success, negative error code otherwise
 */
int32_t ad5592r_reg_read(struct ad5592r_dev *dev, uint8_t reg, uint16_t *value)
{
	int32_t ret;

	if (!dev)
		return -1;
	ret = spi_engine_set_transfer_width(dev->spi, dev->reg_data_width);
		if (ret != 0)
			return ret;

	spi_engine_set_speed(dev->spi, dev->reg_access_speed);
	dev->spi_msg = swab16((AD5592R_REG_LDAC << 11) |
			      AD5592R_LDAC_READBACK_EN | (reg << 2) | dev->ldac_mode);

	ret = no_os_spi_write_and_read(dev->spi, (uint8_t *)&dev->spi_msg,
				       sizeof(dev->spi_msg));
	if (ret < 0)
		return ret;
	ret = spi_engine_set_transfer_width(dev->spi,
					    dev->capture_data_width);
	if (ret != 0)
		return ret;

	spi_engine_set_speed(dev->spi, dev->spi->max_speed_hz);
	ret = ad5592r_spi_wnop_r16(dev, &dev->spi_msg);
	if (ret < 0)
		return ret;

	*value = dev->spi_msg;

	return 0;
}

/**
 * Read GPIOs.
 *
 * @param dev - The device structure.
 * @param value - GPIOs value.
 * @return 0 in case of success, negative error code otherwise
 */
int32_t ad5592r_gpio_read(struct ad5592r_dev *dev, uint8_t *value)
{
	int32_t ret;

	if (!dev)
		return -1;
	ret = spi_engine_set_transfer_width(dev->spi, dev->reg_data_width);
	if (ret != 0)
		return ret;
	spi_engine_set_speed(dev->spi, dev->reg_access_speed);

	ret = ad5592r_reg_write(dev, AD5592R_REG_GPIO_IN_EN,
				AD5592R_GPIO_READBACK_EN | dev->gpio_in);
	if (ret < 0)
		return ret;

	ret = spi_engine_set_transfer_width(dev->spi,
					    dev->capture_data_width);
	if (ret != 0)
		return ret;

	spi_engine_set_speed(dev->spi, dev->spi->max_speed_hz);

	ret = ad5592r_spi_wnop_r16(dev, &dev->spi_msg);
	if (ret < 0)
		return ret;

	*value = (uint8_t)dev->spi_msg;

	return 0;
}

int32_t ad5592r_read_data_spi_engine_offload(struct ad5592r_dev *dev,
			 uint8_t chan,
			 uint32_t *buf,
			 uint16_t samples)
{
	int32_t ret;

	if (!dev || !buf || chan >= dev->num_channels)
		return -1;

	uint32_t adc_seq_cmd = ((AD5592R_REG_ADC_SEQ << 11) | NO_OS_BIT(chan));
	uint32_t commands_data[3] = {
		adc_seq_cmd, // selectează canalul dorit
		0x0000,      // nop
		0x0000       // nop
	};

	struct spi_engine_offload_message msg;
	uint32_t spi_eng_msg_cmds[11] = {
			CS_LOW,
			WRITE(2),
			CS_HIGH,
			SLEEP(20),
			CS_LOW,
			WRITE(2),
		    CS_HIGH,
			SLEEP(20),
			CS_LOW,
		    WRITE_READ(2),
			CS_HIGH
	};

	// Pornește trigger-ul
	no_os_pwm_enable(dev->trigger_pwm_desc);

	// Inițializează offload-ul
	ret = spi_engine_offload_init(dev->spi, dev->offload_init_param);
	if (ret != 0)
		return ret;

	msg.commands = spi_eng_msg_cmds;
	msg.no_commands = NO_OS_ARRAY_SIZE(spi_eng_msg_cmds);
	msg.rx_addr = (uint32_t)buf;
	msg.commands_data = commands_data;

	ret = spi_engine_offload_transfer(dev->spi, msg, samples);
	if (ret != 0)
		return ret;

	no_os_mdelay(100);

	if (dev->dcache_invalidate_range)
		dev->dcache_invalidate_range(msg.rx_addr, samples * sizeof(uint32_t));

	return 0;
}


/**
 * Initialize AD5593r device.
 *
 * @param dev - The device structure.
 * @param init_param - The initial parameters of the device.
 * @return 0 in case of success, negative error code otherwise
 */

int32_t ad5592r_init(struct ad5592r_dev **device,
                     struct ad5592r_init_param *init_param)
{
    struct ad5592r_dev *dev;
	int32_t ret;
    uint16_t temp_reg_val;

    // Alocarea memoriei pentru dispozitivul AD5592R
    dev = (struct ad5592r_dev *)no_os_malloc(sizeof(*dev));
    if (!dev)
        return -1;

    // Verificăm dacă nu folosim SPI standard și configurăm axi_clkgen
    ret = axi_clkgen_init(&dev->clkgen, init_param->clkgen_init);
    if (ret != 0) {
        printf("error: %s: axi_clkgen_init() failed\n", init_param->clkgen_init->name);
        goto error_dev;
    }

    ret = axi_clkgen_set_rate(dev->clkgen, init_param->axi_clkgen_rate);  //axi_clkgen_rate daca avem 20M max ar trebui sa avem 40M
    if (ret != 0) {
        printf("error: %s: axi_clkgen_set_rate() failed\n", init_param->clkgen_init->name);
        goto error_clkgen;
    }
    printf("axi_clkgen_set_rate() set to %ld Hz successfully.\n", init_param->axi_clkgen_rate);


   // Inițializarea SPI engine pentru AD5592R
       // ret = spi_engine_init(&dev->spi, init_param->spi_init);
    ret = no_os_spi_init(&dev->spi, init_param->spi_init);
        if (ret < 0) {
            no_os_free(dev);
            printf("SPI Engine initialization failed!\n");
            return ret;
        } else {
            printf("SPI Engine initialized successfully.\n");
        }
        // Inițializarea SPI Engine offload (folosind parametrii din init_param)
    ret = spi_engine_offload_init(dev->spi, init_param->offload_init_param);
        if (ret < 0) {
                    printf("SPI Engine Offload initialization failed!\n");
                    no_os_free(dev);
                    return ret;
                } else{
                printf("SPI Engine Offload initialized successfully.\n");
                }

        // Continuăm cu restul inițializărilor AD5592R
    	dev->offload_init_param = init_param->offload_init_param;
    	dev->reg_access_speed = init_param->reg_access_speed;
    	dev->reg_data_width = init_param->reg_data_width;
    	dev->capture_data_width = init_param->capture_data_width;
    	dev->dcache_invalidate_range = init_param->dcache_invalidate_range;
        // Restul inițializărilor AD5592R
        dev->ops = &ad5592r_rw_ops;
    // Resetare software a dispozitivului AD5592R
    ret = ad5592r_software_reset(dev);
    if (ret < 0){
    	goto error_clkgen;}

	no_os_mdelay(1000);
    // Setăm modurile canalelor

	dev->num_channels=8;
	dev->channel_modes[0] = 1;
	dev->channel_modes[1] = 1;
	dev->channel_modes[2] = 1;
	dev->channel_modes[3] = 1;
	dev->channel_modes[4] = 1;
	dev->channel_modes[5] = 1;
	dev->channel_modes[6] = 1;
	dev->channel_modes[7] = 1;

	    ret = ad5592r_set_channel_modes(dev);
	    if (ret < 0)
	        goto error_clkgen;

	no_os_mdelay(1000);

    // Dacă se utilizează referință internă, actualizăm registrul PD (Power Down)
    if (init_param->int_ref) {
        ret = ad5592r_reg_read(dev, AD5592R_REG_PD, &temp_reg_val);
        if (ret < 0)
            goto error_clkgen;

        // Activăm referința internă
        temp_reg_val |= AD5592R_REG_PD_EN_REF;

   no_os_mdelay(1000);
        // Scriem valoarea în registrul PD
        ret = ad5592r_reg_write(dev, AD5592R_REG_PD, temp_reg_val);
        if (ret < 0)
            goto error_clkgen;

    	ret = no_os_pwm_init(&dev->trigger_pwm_desc, init_param->trigger_pwm_init);
    	if (ret != 0)
    		goto error_spi;
    }
    *device = dev;

    no_os_mdelay(1000);
     //Dacă ajungem aici, inițializarea a avut succes
    return 0;
error_spi:
       no_os_spi_remove(dev->spi);
error_clkgen:
#if !defined(USE_STANDARD_SPI)
    axi_clkgen_remove(dev->clkgen);
#endif
error_dev:
    no_os_free(dev);

    return -1;
}

