/****************************************************************************
 * include/nuttx/sensors/sht10.h
 *
 *   Copyright (C) 2011-2012, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_SHT10_H
#define __INCLUDE_NUTTX_SENSORS_SHT10_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <stdint.h>

#if defined(CONFIG_I2C) && defined(CONFIG_I2C_SHT10)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************
 * CONFIG_I2C - Enables support for I2C drivers
 * CONFIG_I2C_SHT10 - Enables support for the SHT10 driver
 */
/* SHT10 Register Definitions ***********************************************/
/* SHT10 Registers addresses */

#define SHT10_CMD_TEMP 		0x03 //0000 0011
#define SHT10_CMD_HUMID 	0x05 //0000 0101
#define SHT10_CMD_WSTAT 	0x06 //0000 0110
#define SHT10_CMD_RSTAT 	0x07 //0000 0111
#define SHT10_CMD_RESET 	0x1E //0001 1110

/* Configuration Register Bit Definitions */

#define SHT10_NOACK 		0
#define SHT10_ACK 			1
#define SHT10_RES_HIGH 		0x00// Cau hinh nhiet do 14bit, do am 12bit
#define SHT10_RES_LOW 		0x01// Cau hinh nhiet do 12bit, do am 8bit
#define SHT10_RESOLUTION SHT10_RES_HIGH
/* NOTE: When temperature values are read, they are return as b16_t, fixed
 * precision integer values (see include/fixedmath.h).
 */
/*IOCTL commands*/
#define SNIOC_READT    		_SNIOC(0x0001) /* Arg: uint8_t* pointer */
#define SNIOC_READH    		_SNIOC(0x0002)
#define SNIOC_INIT			_SNIOC(0x0003)
#define SNIOC_RESET			_SNIOC(0x0004)
#define SNIOC_START_MEASURE	_SNIOC(0x0005)
#define SNIOC_MEASURE		_SNIOC(0x0006)
#define SNIOC_FINISH		_SNIOC(0x0007)
#define SNIOC_TEST			_SNIOC(0x0008)

#define SHT10_UNAVAIL      	-32768
#define SHT10_CRC_FAIL     	-32767

#define SHT10_STATE_READY           0
#define SHT10_STATE_MEASURE_TMP     1
#define SHT10_STATE_CALC_TMP        2
#define SHT10_STATE_MEASURE_HUM     3
#define SHT10_STATE_CALC_HUM        4

#define SHT10_AT_5V         4010
#define SHT10_AT_4V         3980
#define SHT10_AT_3_5V       3970
#define SHT10_AT_3V         3960
#define SHT10_AT_2_5V       3940

#define SHT10_TEMP_V_COMP   SHT10_AT_3V

  
static uint8_t 	crc_value;
static uint8_t  sht10_state;
static uint8_t test = 100;
static uint16_t sht10_hum_raw=0,
				sht10_tmp_raw=0;
static int16_t  sht10_hum=0,
				sht10_tmp=0;

static uint8_t sht10_valid(v){
	return ((v) > -32000);
}

static void crc8(uint8_t b)
{
	uint8_t i;
	for (i = 0; i < 8; ++i) {
		if ((crc_value ^ b) & 0x80) {
			crc_value <<= 1;
			crc_value ^= 0x31;
		} else
			crc_value <<= 1;
		b <<= 1;
	}
}
/****************************************************************************
 * Public Types
 ****************************************************************************/


struct i2c_master_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: sht10_register
 *
 * Description:
 *   Register the sht10 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c - An instance of the I2C interface to use to communicate with sht10
 *   addr - The I2C address of the sht10.  The base I2C address of the sht10
 *   is 0x48.  Bits 0-3 can be controlled to get 8 unique addresses from 0x48
 *   through 0x4f.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sht10_register(FAR const char *devpath, FAR struct i2c_master_s *i2c, uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_I2C_SHT10 */
#endif /* __INCLUDE_NUTTX_SENSORS_SHT10_H */
