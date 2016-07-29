/****************************************************************************
 * MYAPPS/adc/adc_main.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/boardctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/analog/adc.h>

#include "adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Use CONFIG_MYAPPS_ADC_NSAMPLES == 0 to mean to collect samples
 * indefinitely.
 */

#ifndef CONFIG_MYAPPS_GSENSOR_NSAMPLES
#  define CONFIG_MYAPPS_GSENSOR_NSAMPLES 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct adc_state_s g_adcstate;

static bool g_gsensor_started;
bool cont_gsensor = true;
int g_gsensor_pid;
FILE *fp;
int fd;
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_devpath
 ****************************************************************************/

static void adc_devpath(FAR struct adc_state_s *adc, FAR const char *devpath)
{
  /* Get rid of any old device path */

  if (adc->devpath)
    {
      free(adc->devpath);
    }

  /* Then set-up the new device path by copying the string */

  adc->devpath = strdup(devpath);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_main
 ****************************************************************************/


static int button_handler(int irq, FAR void *context){
  printf("\n User button pressed exiting...\n");
  cont_gsensor = false;
  g_gsensor_started = false;
}
//
static int gsensor_daemon(int argc, char *argv[]){
  g_gsensor_started = true;
  struct adc_msg_s sample[CONFIG_MYAPPS_GSENSOR_GROUPSIZE];
  size_t readsize;
  ssize_t nbytes;
  int errval = 0;
  int ret;
  int i,j;
  int am_channel,am_data;

  

  

  /* Check if we have initialized */

  if (!g_adcstate.initialized)
  {
      /* Initialization of the ADC hardware is performed by logic external to
       * this test.
       */

       printf("adc_main here: Initializing external ADC device\n");
       ret = board_gsensoradc_setup();
       if (ret < 0)
       {
        printf("adc_main here: boardctl failed: %d\n", errno);
        errval = 1;
        goto errout;
      }

      /* Set the default values */

      adc_devpath(&g_adcstate, CONFIG_MYAPPS_GSENSOR_DEVPATH);

#if CONFIG_MYAPPS_GSENSOR_NSAMPLES > 0
      g_adcstate.count = CONFIG_MYAPPS_GSENSOR_NSAMPLES;
#else
      g_adcstate.count = 1;
#endif
      g_adcstate.initialized = true;
    }

  /* If this example is configured as an NX add-on, then limit the number of
   * samples that we collect before returning.  Otherwise, we never return
   */

#if defined(CONFIG_NSH_BUILTIN_APPS) || CONFIG_MYAPPS_GSENSOR_NSAMPLES > 0
   printf("adc_main: g_adcstate.count: %d\n", g_adcstate.count);
#endif

  /* Open the ADC device for reading */

   printf("adc_main: Hardware initialized. Opening the ADC device: %s\n",
     g_adcstate.devpath);

   fd = open(g_adcstate.devpath, O_RDONLY);
   if (fd < 0)
   {
    printf("adc_main: open %s failed: %d\n", g_adcstate.devpath, errno);
    errval = 2;
    goto errout;
  }

  fp = fopen("/sdcard/TEST.txt", "w");
  if (fp < 0) {
    printf("error opening TEST.TXT");
    return 1;
  }
  
  board_button_initialize();

  xcpt_t oldhandler = board_button_irq(0, button_handler);

      /* Use lowsyslog() for compatibility with interrupt handler output. */

  lowsyslog(LOG_INFO, "Attached handler at %p to button 0 [User button], oldhandler:%p\n",
    button_handler, oldhandler);

  /* Now loop the appropriate number of times, displaying the collected
   * ADC samples.
   */
/*
#if defined(CONFIG_NSH_BUILTIN_APPS)
  for (; g_adcstate.count > 0; g_adcstate.count--)
#elif CONFIG_MYAPPS_GSENSOR_NSAMPLES > 0
  for (g_adcstate.count = 0;
       g_adcstate.count < CONFIG_MYAPPS_GSENSOR_NSAMPLES;
       g_adcstate.count++)
#else
  for (;;)
#endif*/

    while(cont_gsensor)
    {
    /* Flush any output before the loop entered or from the previous pass
     * through the loop.
     */
    //fflush(stdout);
     
     printf("\nSample number %d\n",++j);
     fprintf(fp,"\nSample number %d\n",j);


#ifdef CONFIG_MYAPPS_GSENSOR_SWTRIG
    /* Issue the software trigger to start ADC conversion */

     ret = ioctl(fd, ANIOC_TRIGGER, 0);
     if (ret < 0)
     {
      int errcode = errno;
      printf("adc_main: ANIOC_TRIGGER ioctl failed: %d\n", errcode);
    }
#endif

    /* Read up to CONFIG_MYAPPS_ADC_GROUPSIZE samples */

    readsize = CONFIG_MYAPPS_GSENSOR_GROUPSIZE * sizeof(struct adc_msg_s);
    nbytes = read(fd, sample, readsize);

    /* Handle unexpected return values */

    if (nbytes < 0)
    {
      errval = errno;
      if (errval != EINTR)
      {
        printf("adc_main: read %s failed: %d\n",
         g_adcstate.devpath, errval);
        errval = 3;
        goto errout_with_dev;
      }

      printf("adc_main: Interrupted read...\n");
    }
    else if (nbytes == 0)
    {
      printf("adc_main: No data read, Ignoring\n");
    }

    /* Print the sample data on successful return */

    else
    {
      int nsamples = nbytes / sizeof(struct adc_msg_s);
      if (nsamples * sizeof(struct adc_msg_s) != nbytes)
      {
        printf("adc_main: read size=%ld is not a multiple of sample size=%d, Ignoring\n",
         (long)nbytes, sizeof(struct adc_msg_s));
      }
      else
      {
        printf("Sample:\n");
        for (i = 0; i < nsamples; i++)
        {
          am_channel = sample[i].am_channel;
          am_data = sample[i].am_data;

          printf("%d: channel: %d value: %d\n",
           i+1,am_channel , am_data);

          fprintf(fp,"%d: channel: %d value: %d\n",
           i+1, am_channel, am_data);

          if(am_data >3500){
            printf("Environment in channel %d too dry\n",am_channel);
          }else if(am_data <1000){
            printf("Environment in channel %d too wet\n",am_channel);
          }else{
            printf("Environment in channel %d is perfect\n",am_channel);
          }

          fflush(fp);
        }
      }
    }

    usleep(150000);
  }

  (void)board_button_irq(0, NULL);
  fclose(fp);
  close(fd);
  task_delete(g_gsensor_pid);

  /* Error exits */

  errout_with_dev:
  (void)board_button_irq(0, NULL);
  fclose(fp);
  close(fd);
  task_delete(g_gsensor_pid);

  errout:
  printf("Terminating!\n");
  fflush(stdout);
  task_delete(g_gsensor_pid);
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int gsensor_main(int argc, char *argv[])
#endif
{
  FAR char *gsensorargv[2];
  if(!g_gsensor_started){
    gsensorargv[0] = "gsensor_daemon";
    gsensorargv[1] = NULL;

    g_gsensor_pid = task_create("gsensor_daemon", CONFIG_MYAPPS_GSENSOR_PRIORITY,
      CONFIG_MYAPPS_GSENSOR_STACKSIZE, gsensor_daemon, (FAR char * const *)gsensorargv);
    if(g_gsensor_pid < 0){
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to start gsensor_daemon: %d\n",errcode);
      return -errcode;
      printf("gsensor_daemon started");
    }
  }
  else{
    printf("gsensor_daemon already started");
  }
  return 0;
}
