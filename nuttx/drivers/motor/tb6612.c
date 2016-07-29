/****************************************************************************
 * drivers/motor/tb6612.c
 * Character driver for the STMicro LM-75 Temperature Sensor
 *
 *   Copyright (C) 2011, 2013, 2016 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <arch/board/board.h>

#if defined(CONFIG_MOTOR) && defined(CONFIG_TB6612)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int tb6612_gpioconfig(void);

 /****************************************************************************
 * Private Data
 ****************************************************************************/

#  define GPIO_MOTOR_AIN1 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN9)
#  define GPIO_MOTOR_AIN2 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN10)
#  define GPIO_MOTOR_BIN1 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN11)
#  define GPIO_MOTOR_BIN2 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN12)
#  define GPIO_MOTOR_STBY (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN15)

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int tb6612_gpioconfig(void){
	int ret;
	ret = stm32_configgpio(GPIO_MOTOR_AIN1);

	return ret;
}


#endif
