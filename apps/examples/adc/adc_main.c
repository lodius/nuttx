/****************************************************************************
 * examples/adc/adc_main.c
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

/* Use CONFIG_EXAMPLES_ADC_NSAMPLES == 0 to mean to collect samples
 * indefinitely.
 */

#ifndef CONFIG_EXAMPLES_ADC_NSAMPLES
#  define CONFIG_EXAMPLES_ADC_NSAMPLES 0
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
 * Name: adc_help
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static void adc_help(FAR struct adc_state_s *adc)
{
  printf("Usage: adc [OPTIONS]\n");
  printf("\nArguments are \"sticky\".  For example, once the ADC device is\n");
  printf("specified, that device will be re-used until it is changed.\n");
  printf("\n\"sticky\" OPTIONS include:\n");
  printf("  [-p devpath] selects the ADC device.  "
         "Default: %s Current: %s\n",
         CONFIG_EXAMPLES_ADC_DEVPATH, g_adcstate.devpath ? g_adcstate.devpath : "NONE");
  printf("  [-n count] selects the samples to collect.  "
         "Default: 1 Current: %d\n", adc->count);
  printf("  [-h] shows this messagea and exits\n");
}
#endif

/****************************************************************************
 * Name: arg_string
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static int arg_string(FAR char **arg, FAR char **value)
{
  FAR char *ptr = *arg;

  if (ptr[2] == '\0')
    {
      *value = arg[1];
      return 2;
    }
  else
    {
      *value = &ptr[2];
      return 1;
    }
}
#endif

/****************************************************************************
 * Name: arg_decimal
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static int arg_decimal(FAR char **arg, FAR long *value)
{
  FAR char *string;
  int ret;

  ret = arg_string(arg, &string);
  *value = strtol(string, NULL, 10);
  return ret;
}
#endif

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static void parse_args(FAR struct adc_state_s *adc, int argc, FAR char **argv)
{
  FAR char *ptr;
  FAR char *str;
  long value;
  int index;
  int nargs;

  for (index = 1; index < argc; )
    {
      ptr = argv[index];
      if (ptr[0] != '-')
        {
          printf("Invalid options format: %s\n", ptr);
          exit(0);
        }

      switch (ptr[1])
        {
          case 'n':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 0)
              {
                printf("Count must be non-negative: %ld\n", value);
                exit(1);
              }

            adc->count = (uint32_t)value;
            index += nargs;
            break;

          case 'p':
            nargs = arg_string(&argv[index], &str);
            adc_devpath(adc, str);
            index += nargs;
            break;

          case 'h':
            adc_help(adc);
            exit(0);

          default:
            printf("Unsupported option: %s\n", ptr);
            adc_help(adc);
            exit(1);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_main
 ****************************************************************************/

/*
r Opens an existing text file for reading purpose.

w Opens a text file for writing. If it does not exist, then a new file is created. 
Here your program will start writing content from the beginning of the file.

a Opens a text file for writing in appending mode. If it does not exist, then a new file is created. 
Here your program will start appending content in the existing file content.

r+  Opens a text file for both reading and writing.

w+  Opens a text file for both reading and writing. 
It first truncates the file to zero length if it exists, otherwise creates a file if it does not exist.
a+  Opens a text file for both reading and writing. 

It creates the file if it does not exist. The reading will start from the beginning but writing can only be appended.
*/

bool cont = true;

static int button_handler(int irq, FAR void *context){
  printf("\n User button pressed exiting...\n");
  cont = false;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int adc_main(int argc, char *argv[])
#endif
{
  struct adc_msg_s sample[CONFIG_EXAMPLES_ADC_GROUPSIZE];
  size_t readsize;
  ssize_t nbytes;
  int fd;
  int errval = 0;
  int ret;
  int i,j;

  int am_channel,am_data;

  char buf[80];
  char *retp;
  FILE *fp;

  

  /* Check if we have initialized */

  if (!g_adcstate.initialized)
  {
      /* Initialization of the ADC hardware is performed by logic external to
       * this test.
       */

       printf("adc_main here: Initializing external ADC device\n");
       ret = boardctl(BOARDIOC_ADCTEST_SETUP, 0);
       if (ret < 0)
       {
        printf("adc_main here: boardctl failed: %d\n", errno);
        errval = 1;
        goto errout;
      }

      /* Set the default values */

      adc_devpath(&g_adcstate, CONFIG_EXAMPLES_ADC_DEVPATH);

#if CONFIG_EXAMPLES_ADC_NSAMPLES > 0
      g_adcstate.count = CONFIG_EXAMPLES_ADC_NSAMPLES;
#else
      g_adcstate.count = 1;
#endif
      g_adcstate.initialized = true;
    }

    
  /* Parse the command line */

#ifdef CONFIG_NSH_BUILTIN_APPS
  parse_args(&g_adcstate, argc, argv);
#endif

  /* If this example is configured as an NX add-on, then limit the number of
   * samples that we collect before returning.  Otherwise, we never return
   */

#if defined(CONFIG_NSH_BUILTIN_APPS) || CONFIG_EXAMPLES_ADC_NSAMPLES > 0
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
#elif CONFIG_EXAMPLES_ADC_NSAMPLES > 0
  for (g_adcstate.count = 0;
       g_adcstate.count < CONFIG_EXAMPLES_ADC_NSAMPLES;
       g_adcstate.count++)
#else
  for (;;)
#endif*/
    
  for(;cont == true;)
  {
    /* Flush any output before the loop entered or from the previous pass
     * through the loop.
     */
    //fflush(stdout);
     
    printf("\nSample number %d\n",++j);
    fprintf(fp,"\nSample number %d\n",j);
    

#ifdef CONFIG_EXAMPLES_ADC_SWTRIG
    /* Issue the software trigger to start ADC conversion */

    ret = ioctl(fd, ANIOC_TRIGGER, 0);
    if (ret < 0)
      {
        int errcode = errno;
        printf("adc_main: ANIOC_TRIGGER ioctl failed: %d\n", errcode);
      }
#endif

    /* Read up to CONFIG_EXAMPLES_ADC_GROUPSIZE samples */

    readsize = CONFIG_EXAMPLES_ADC_GROUPSIZE * sizeof(struct adc_msg_s);
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

                fflush(fp);
                }
          }
      }

      usleep(150000);
  }

  (void)board_button_irq(0, NULL);
  fclose(fp);
  close(fd);
  return OK;

  /* Error exits */

errout_with_dev:
  (void)board_button_irq(0, NULL);
  fclose(fp);
  close(fd);

errout:
  printf("Terminating!\n");
  fflush(stdout);
  return errval;
}
