/****************************************************************************
 * drivers/sensors/sht10.c
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
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/config.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sht10.h>

#if defined(CONFIG_I2C) && defined(CONFIG_I2C_SHT10)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SHT10_I2C_FREQUENCY
#  define CONFIG_SHT10_I2C_FREQUENCY 100000
#endif

/****************************************************************************
 * Private
 ****************************************************************************/

struct sht10_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
};
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* I2C Helpers */

/*static int     sht10_i2c_write(FAR struct sht10_dev_s *priv, FAR const uint8_t *buffer, int buflen);
static int     sht10_i2c_read(FAR struct sht10_dev_s *priv, FAR uint8_t *buffer, int buflen);
static int     sht10_readtemp(FAR struct sht10_dev_s *priv, FAR b16_t *temp);*/
static void start(FAR struct file *filep);
static uint8_t send(FAR struct file *filep,uint16_t b);
static uint8_t recv_data(FAR struct file *filep);
static uint8_t recv_crc(FAR struct file *filep);
static uint8_t sht10_start_temp(FAR struct file *filep);
static uint8_t sht10_start_humid(FAR struct file *filep);
static uint8_t sht10_ready(FAR struct file *filep);
static int16_t result(FAR struct file *filep);
static int16_t sht10_result_temp(FAR struct file *filep);
static int16_t sht10_result_humid(FAR struct file *filep);
static void sht10_init(FAR struct file *filep);
static void sht10_start_measure(FAR struct file *filep);
static uint8_t sht10_measure(FAR struct file *filep);
static uint8_t sht10_measure_finish(FAR struct file *filep);
static int16_t sht10_get_tmp(void);
static int16_t sht10_get_hum(void);
//static uint16_t sht10_get_tmp_raw(void);
//static uint16_t sht10_get_hum_raw(void);
/* Character driver methods */

static int     sht10_open(FAR struct file *filep);
static int     sht10_close(FAR struct file *filep);
static ssize_t sht10_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t sht10_write(FAR struct file *filep,FAR const char *buffer, size_t buflen);    
static int sht10_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_sht10fops =
{
  sht10_open,
  sht10_close,
  sht10_read,
  sht10_write,
  NULL,
  sht10_ioctl
#ifndef CONFIG_DISABLE_POLL
  , NULL
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*Functions*/

void delay(void);
void scl_out(FAR struct i2c_master_s *dev);
void scl_hi(FAR struct i2c_master_s *dev);
void scl_lo(FAR struct i2c_master_s *dev);
void sda_hi(FAR struct i2c_master_s *dev);
void sda_lo(FAR struct i2c_master_s *dev);
uint8_t  sda_val(FAR struct i2c_master_s *dev);
void scl_pulse(FAR struct i2c_master_s *dev);
void sda_reset(FAR struct i2c_master_s *dev);
void sda_set(FAR struct i2c_master_s *dev);


static ssize_t sht10_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  sndbg("Function empty");
  return OK;
}

static ssize_t sht10_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  sndbg("Function empty");
  return -ENOSYS;
}

static int sht10_open(FAR struct file *filep)
{
  sht10_init(filep);
  sndbg("Opened and initialized device sht10\n");
  return OK;
}

static int sht10_close(FAR struct file *filep)
{
  return OK;
}

static uint8_t send(FAR struct file *filep,uint16_t b)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct sht10_dev_s *priv   = inode->i_private;
  FAR struct i2c_master_s *dev   = priv->i2c;

  crc8(b);
  // data
  uint8_t i;
  
  for (i = 0; i < 8; ++i) {
    if (b & 0x80)
      sda_hi(dev);
    else
      sda_lo(dev);
    b <<= 1;
    delay();
    scl_pulse(dev);
  }
  // acknowledge
  sda_hi(dev);
  delay();
  uint8_t ack = sda_val(dev);
  scl_pulse(dev);
  return ack;
}

static uint8_t recv_data(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct sht10_dev_s *priv   = inode->i_private;
  FAR struct i2c_master_s *dev   = priv->i2c;
  // data
  uint8_t b = 0;
  uint8_t i;
  for (i = 0; i < 8; ++i) {
   // data is transmitted MSB first
    b <<= 1;
    if (sda_val(dev))
      b |= 1;
    scl_pulse(dev);
    delay();
  }
  // send acknowledge bit
  sda_lo(dev);
  
  delay();

  scl_pulse(dev);
  
  sda_hi(dev);
  delay();
  
  crc8(b);
  return b;
}

static uint8_t recv_crc(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct sht10_dev_s *priv   = inode->i_private;
  FAR struct i2c_master_s *dev   = priv->i2c;
  // data
  uint8_t b = 0;
  uint8_t i;
  for (i = 0; i < 8; ++i) {
   // CRC is transmitted LSB first
    b >>= 1;
    if (sda_val(dev))
      b |= 0x80;
    scl_pulse(dev);
    delay();
  }
 // hi acknowledge
  sda_hi(dev);
  
  delay();

  scl_pulse(dev);
  delay();
  return b;
}

static void start(FAR struct file *filep) {
  FAR struct inode        *inode = filep->f_inode;
  FAR struct sht10_dev_s *priv   = inode->i_private;
  FAR struct i2c_master_s *dev   = priv->i2c;

  uint8_t i;
 
  // connection reset sequence

  for (i = 0; i < 10; ++i) {
    scl_pulse(dev);
    delay();
  }

  // start sequence
  

  scl_hi(dev); delay();

  sda_lo(dev); delay(); //data low

  scl_lo(dev); delay();
  scl_hi(dev); delay();

  sda_hi(dev); delay();//data high

  scl_lo(dev); delay();
  
}

static uint8_t sht10_start_temp(FAR struct file *filep){
  crc_value = SHT10_RESOLUTION << 7; // bit-reversed
  start(filep);
  //usleep(500000);
  return send(filep,SHT10_CMD_TEMP) == 0;
  
}

static uint8_t sht10_start_humid(FAR struct file *filep){
  crc_value = SHT10_RESOLUTION << 7; // bit-reversed
  start(filep);
  //usleep(500000);
  return send(filep,SHT10_CMD_HUMID) == 0;
  
}

static uint8_t sht10_ready(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct sht10_dev_s *priv   = inode->i_private;
  FAR struct i2c_master_s *dev   = priv->i2c;
  return sda_val(dev) == 0;
}

//
//Get Result Functions
//


static int16_t result(FAR struct file *filep){
  
  usleep(80500);

  if (!sht10_ready(filep))
    return SHT10_UNAVAIL;

  int16_t ret = recv_data(filep) << 8; 
  ret |= recv_data(filep);
  uint8_t crc = recv_crc(filep);

  if (crc != crc_value)
    return SHT10_CRC_FAIL;

  return ret;
}

static int16_t sht10_result_temp(FAR struct file *filep){

  int16_t ret = result(filep);
  if (sht10_valid(ret))
  {
    sht10_tmp_raw=(uint16_t)ret;
    if(SHT10_RESOLUTION)
      ret = ret * 4 - SHT10_TEMP_V_COMP;
    else
      ret -= SHT10_TEMP_V_COMP;
  }
  return ret;
}
         
static int16_t sht10_result_humid(FAR struct file *filep){

  int16_t ret = result(filep);
  if (sht10_valid(ret)){
    sht10_hum_raw=(uint16_t)ret;
    if(SHT10_RESOLUTION){
    // inspired by Werner Hoch, modified for low resolution mode
      const int32_t C1 = (int32_t)(-4.0 * 100);
      const int32_t C2 = (int32_t)(0.648 * 100 * (1L<<24));
      const int32_t C3 = (int32_t)(-7.2e-4 * 100 * (1L<<24));
      const int32_t T1 = (uint32_t)(0.01 * (1L<<30));
      const int32_t T2 = (uint32_t)(0.00128 * (1L<<30));
      ret = (int16_t)((((C3 * ret + C2) >> 7) * ret + (1L<<16)) >> 17) + C1;
      ret = (int16_t)( ((( ((int32_t)sht10_tmp-2500) * (int32_t)( (T1 + T2*((int32_t)sht10_hum_raw)) >>13))>>17) + ((int32_t)ret)) );
    
    }
    else{
    // inspired by Werner Hoch, high res mode
      const int32_t C1 = (int32_t)(-4.0 * 100);
      const int32_t C2 = (int32_t)(0.0405 * 100 * (1L<<28));
      const int32_t C3 = (int32_t)(-2.8e-6 * 100 * (1L<<30));
      const int32_t T1 = (uint32_t)(0.01 * (1L<<30));
      const int32_t T2 = (uint32_t)(0.00008 * (1L<<30));
      ret = (int16_t)((((((C3 * ret) >> 2) + C2) >> 11) * ret + (1L<<16)) >> 17) + C1;
      ret = (int16_t)( ((( ((int32_t)sht10_tmp-2500) * (int32_t)( (T1 + T2*((int32_t)sht10_hum_raw)) >>13))>>17) + ((int32_t)ret)) );
      
    }
    // implemented by whitejack (temp compensation)
    
  }

  if(ret>9999)
    ret=9999;

  if(ret<0001)
    ret=0001;
  return ret;
}

////////////////////////////////////////////////////////////////////
//User + Init functions
////////////////////////////////////////////////////////////////////

static void sht10_init(FAR struct file *filep){
  FAR struct inode      *inode = filep->f_inode;
  FAR struct sht10_dev_s *priv   = inode->i_private;
  FAR struct i2c_master_s *dev   = priv->i2c;

  scl_out(dev);
  scl_lo(dev);

  sda_lo(dev);
  //sda_reset(dev);

  delay();

  start(filep);
  //usleep(500000);
  send(filep,SHT10_CMD_RESET);
  usleep(11000);

  start(filep);
  //usleep(500000);
  send(filep,SHT10_CMD_WSTAT);
  
  send(filep,SHT10_RESOLUTION);
  sht10_state = SHT10_STATE_MEASURE_TMP;
}

static void sht10_start_measure(FAR struct file *filep){
  if(sht10_state==SHT10_STATE_READY)
  {
    sht10_state = SHT10_STATE_MEASURE_TMP;
  }
}

static uint8_t sht10_measure(FAR struct file *filep){
  switch(sht10_state)
  {

    case SHT10_STATE_MEASURE_TMP:
    {
      sndbg("Measuring temperature\n");
      sht10_start_temp(filep);
      sht10_state = SHT10_STATE_CALC_TMP;
    }
    break;

    case SHT10_STATE_CALC_TMP:
    {
      while(!sht10_ready(filep)){

      }//Wait for ready signal

      if(sht10_ready(filep))
      {
        sndbg("Calculating temperature\n");
        sht10_tmp = sht10_result_temp(filep);
        sht10_state = SHT10_STATE_MEASURE_HUM;
      }
    }
    break;

    case SHT10_STATE_MEASURE_HUM:
    {
      sndbg("Measuring humidity\n");
      sht10_start_humid(filep);
      sht10_state = SHT10_STATE_CALC_HUM;
    }
    break;

    case SHT10_STATE_CALC_HUM:
    {
      while(!sht10_ready(filep)){
        
      }//Wait for ready signal

      if(sht10_ready(filep))
      {
        sndbg("Calculating humidity\n");
        sht10_hum = sht10_result_humid(filep);
        sht10_state = SHT10_STATE_READY;
      }
    }
    break;

  }    
  return sht10_state;
}

static uint8_t sht10_measure_finish(FAR struct file *filep){
  if(sht10_measure(filep)==SHT10_STATE_READY){
    return 1;
  }
  else{
    return 0;
  }
}

static uint8_t get_test(void){
  return test;
}

/*static uint16_t sht10_get_tmp_raw(void){
  return sht10_tmp_raw;
}*/

/*static uint16_t sht10_get_hum_raw(void){
  return sht10_hum_raw;
}*/

static int16_t sht10_get_tmp(void){
  return sht10_tmp;
}

static int16_t sht10_get_hum(void){
  return sht10_hum;
}

static int sht10_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  //FAR struct inode      *inode = filep->f_inode;
  //FAR struct sht10_dev_s *priv  = inode->i_private;
  int ret   = OK;

  switch (cmd)
    {
      case SNIOC_READT:
        {
          ret = sht10_get_tmp();
          sndbg("Temperature: %d\n", ret);
          
        }
      break;

      case SNIOC_READH:
        {
          ret = sht10_get_hum();
          sndbg("Humidity: %d\n", ret);

        }
      break;

      case SNIOC_INIT:
        {
          sht10_init(filep);
          sndbg("Initialized the device\n");

        }
      break;

      case SNIOC_START_MEASURE:
        {
          sht10_start_measure(filep);
          sndbg("Started measuring\n");

        }
      break;

      case SNIOC_MEASURE:
        {
          ret = sht10_measure(filep);
          switch(ret)
          {
            case SHT10_STATE_READY:
              //sndbg("Ready : %d\n",ret);
            break;

            case SHT10_STATE_MEASURE_TMP:
              //sndbg("Measuring temperature : %d\n",ret);
            break;

            case SHT10_STATE_CALC_TMP:
              //sndbg("Calculating temperature : %d\n",ret);
            break;

            case SHT10_STATE_MEASURE_HUM:
              //sndbg("Measuring humidity : %d\n",ret);
            break;

            case SHT10_STATE_CALC_HUM:
              //sndbg("Calculating humidity : %d\n",ret);
            break;

            default:
              sndbg("Unrecognized : %d\n",ret);
          }
        }
      break;

      case SNIOC_FINISH:
        {
          ret = sht10_measure_finish(filep);
          if(ret)
            sndbg("Finished measuring : %d\n",ret);
        }
      break;

      case SNIOC_TEST:
        {

          ret = get_test();
          sndbg("test %d\n",ret);
        }
      break;

      default:
        sndbg("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
      break;
    }

  return ret;
}





/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sht10_register
 *
 * Description:
 *   Register the LM-75 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c - An instance of the I2C interface to use to communicate with sht10
 *   addr - The I2C address of the LM-75.  The base I2C address of the sht10
 *   is 0x48.  Bits 0-3 can be controlled to get 8 unique addresses from 0x48
 *   through 0x4f.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sht10_register(FAR const char *devpath, FAR struct i2c_master_s *i2c, uint8_t addr)
{
  FAR struct sht10_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the SHT10 device structure */

  priv = (FAR struct sht10_dev_s *)kmm_malloc(sizeof(struct sht10_dev_s));
  if (priv == NULL)
    {
      sndbg("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;

  /* Register the character driver */

  ret = register_driver(devpath, &g_sht10fops, 0666, priv);
  if (ret < 0)
    {
      sndbg("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_I2C && CONFIG_I2C_SHT10 */
