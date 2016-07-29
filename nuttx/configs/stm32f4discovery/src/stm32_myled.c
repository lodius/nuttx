#include <nuttx/config.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <sched.h>

#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32f4discovery.h"

#ifndef CONFIG_ARCH_LEDS

/* This array maps a LED number to GPIO pin configuration */

static uint32_t led_map[BOARD_NLEDS] ={ GPIO_LED1, GPIO_LED2, GPIO_LED3, GPIO_LED4};

/****************************************************************************
 * LEDs: Fileops Prototypes and Structures
 ****************************************************************************/

typedef FAR struct file   file_t;

static int     leds_open(file_t *filep);
static int     leds_close(file_t *filep);
static ssize_t leds_read(file_t *filep, FAR char *buffer, size_t buflen);
static ssize_t leds_write(file_t *filep, FAR const char *buf, size_t buflen);

static const struct file_operations leds_ops = {
  leds_open,    /* open */
  leds_close,   /* close */
  leds_read,    /* read */
  leds_write,   /* write */
  0,            /* seek */
  0,            /* ioctl */
};

/****************************************************************************
 * LEDs: Fileops
 ****************************************************************************/

static int leds_open(file_t *filep)
{
  return OK;
}

static int leds_close(file_t *filep)
{
  return OK;
}

static ssize_t leds_read(file_t *filep, FAR char *buf, size_t buflen)
{
  register uint8_t reg;

  if(buf == NULL || buflen < 1)
    /* Well... nothing to do */
    return -EINVAL;

  /* These LEDs are actived by low signal (common anode), then invert signal we read*/
  reg = ~(stm32_gpioread(GPIO_LED4));
  reg = (reg << 1) | ~(stm32_gpioread(GPIO_LED3));
  reg = (reg << 1) | ~(stm32_gpioread(GPIO_LED2));
  reg = (reg << 1) | ~(stm32_gpioread(GPIO_LED1));
  reg = reg & 0x0F;

  *buf = (char) reg;

  return 1;
}

static ssize_t leds_write(file_t *filep, FAR const char *buf, size_t buflen)
{
  register uint8_t reg;

  if(buf == NULL || buflen < 1)
    /* Well... nothing to do */
    return -EINVAL;

  reg = (uint8_t) *buf;

  printf("Trying to write %d\n", reg);

  /* These LEDs are actived by low signal (common anode), invert the boolean value */
  stm32_gpiowrite(GPIO_LED1, !(reg & BOARD_LED1_BIT));
  stm32_gpiowrite(GPIO_LED2, !(reg & BOARD_LED2_BIT));
  stm32_gpiowrite(GPIO_LED3, !(reg & BOARD_LED3_BIT));
  stm32_gpiowrite(GPIO_LED4, !(reg & BOARD_LED4_BIT));//!

  return 1;
}

/* Configure LED1-4 GPIOs for output */
void led_init(void){
   stm32_configgpio(GPIO_LED1);
   stm32_configgpio(GPIO_LED2);
   stm32_configgpio(GPIO_LED3);
   stm32_configgpio(GPIO_LED4);
   (void)register_driver("/dev/myled", &leds_ops, 0444, NULL);
}



/*My function*/

/*Control 1 led*/
void led_set(int led, bool ledon){
  if ((unsigned)led < BOARD_NLEDS)
    {
      stm32_gpiowrite(led_map[led], ledon);
    }
}

/*Control 4 led*/
void led_allset(uint8_t ledset){
    stm32_gpiowrite(GPIO_LED1, (ledset & BOARD_LED1_BIT) == 0);
    stm32_gpiowrite(GPIO_LED2, (ledset & BOARD_LED2_BIT) == 0);
  	stm32_gpiowrite(GPIO_LED3, (ledset & BOARD_LED3_BIT) == 0);
  	stm32_gpiowrite(GPIO_LED4, (ledset & BOARD_LED4_BIT) == 0);
}



#endif /* !CONFIG_ARCH_LEDS */
