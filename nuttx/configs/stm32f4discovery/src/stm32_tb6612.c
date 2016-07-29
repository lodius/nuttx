/****************************************************************************
 * stm32_tb6612.c
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
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <arch/board/board.h>

#include "stm32.h"
#include "stm32f4discovery.h"

#if defined(CONFIG_TB6612)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

void seta1(void);
void reseta1(void);
void seta2(void);
void reseta2(void);

void setb1(void);
void resetb1(void);
void setb2(void);
void resetb2(void);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void tb6612_gpioconfig(void);

void tb6612_stbyon(void);
void tb6612_stbyoff(void);

void tb6612_amotor_cw(void);
void tb6612_amotor_ccw(void);
void tb6612_amotor_stop(void);

void tb6612_bmotor_cw(void);
void tb6612_bmotor_ccw(void);
void tb6612_bmotor_stop(void);

void config_fan(void);
void fan_on(void);
void fan_off(void);


 /****************************************************************************
 * Private Data
 ****************************************************************************/

#  define GPIO_MOTOR_AIN1 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN9)
#  define GPIO_MOTOR_AIN2 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN10)
#  define GPIO_MOTOR_BIN1 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN11)
#  define GPIO_MOTOR_BIN2 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN12)
#  define GPIO_MOTOR_STBY (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN15)

 //PWM0 = D13
 //PWM1 = A5

/****************************************************************************
 * Private Functions
 ****************************************************************************/

//Fan GPIO

#  define GPIO_FAN (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_PORTB|GPIO_PIN13)

void config_fan(void){
	stm32_configgpio(GPIO_FAN);
}

void fan_on(void){
	stm32_gpiowrite(GPIO_FAN,1);
}

void fan_off(void){
	stm32_gpiowrite(GPIO_FAN,0);
}

//Manipualte AIN GPIOs

void seta1(void){
	stm32_gpiowrite(GPIO_MOTOR_AIN1,1);
}

void reseta1(void){
	stm32_gpiowrite(GPIO_MOTOR_AIN1,0);
}

void seta2(void){
	stm32_gpiowrite(GPIO_MOTOR_AIN2,1);
}

void reseta2(void){
	stm32_gpiowrite(GPIO_MOTOR_AIN2,0);
}

//Manipulate BIN GPIOs

void setb1(void){
	stm32_gpiowrite(GPIO_MOTOR_BIN1,1);
}

void resetb1(void){
	stm32_gpiowrite(GPIO_MOTOR_BIN1,0);
}

void setb2(void){
	stm32_gpiowrite(GPIO_MOTOR_BIN2,1);
}

void resetb2(void){
	stm32_gpiowrite(GPIO_MOTOR_BIN2,0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

//Configure TB6612 GPIOs

void tb6612_gpioconfig(void){
	stm32_configgpio(GPIO_MOTOR_AIN1);
	stm32_configgpio(GPIO_MOTOR_AIN2);
	stm32_configgpio(GPIO_MOTOR_BIN1);
	stm32_configgpio(GPIO_MOTOR_BIN2);
	stm32_configgpio(GPIO_MOTOR_STBY);
}

//Manipulate STBY GPIO

void tb6612_stbyon(void){
	stm32_gpiowrite(GPIO_MOTOR_STBY,0);
}

void tb6612_stbyoff(void){
	stm32_gpiowrite(GPIO_MOTOR_STBY,1);
}

//Control motor A

void tb6612_amotor_cw(void){
	seta1();
	reseta2();
}

void tb6612_amotor_ccw(void){
	reseta1();
	seta2();
}

void tb6612_amotor_stop(void){
	reseta1();
	reseta2();
}

//Control motor B

void tb6612_bmotor_cw(void){
	setb1();
	resetb2();
}

void tb6612_bmotor_ccw(void){
	resetb1();
	setb2();
}

void tb6612_bmotor_stop(void){
	resetb1();
	resetb2();
}

#endif
