/****************************************************************************
 * MYAPPS/alarm/alarm_main.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <sys/ioctl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/timers/rtc.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_alarm2_daemon_started;
static pid_t g_alarm2_daemon_pid;
static bool g_alarm2_received[CONFIG_RTC_NALARMS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: alarm_handler
 ****************************************************************************/

static void alarm2_handler(int signo, FAR siginfo_t *info, FAR void *ucontext)
{
  int almndx = info->si_value.sival_int;
  if (almndx >= 0 && almndx < CONFIG_RTC_NALARMS)
    {
      g_alarm2_received[almndx] = true;
      printf("%d",almndx);
    }
  
}

/****************************************************************************
 * Name: alarm_daemon
 ****************************************************************************/

static int alarm2_daemon(int argc, FAR char *argv[])
{
  struct sigaction act;
  sigset_t set;
  int ret;
  int i;

  /* Indicate that we are running */

  g_alarm2_daemon_started = true;
  printf("alarm2_daemon: Running testtt\n");
  /* Make sure that the alarm signal is unmasked */

  sigemptyset(&set);

  sigaddset(&set, CONFIG_MYAPPS_ALARM2_SIGNO);
  ret = sigprocmask(SIG_UNBLOCK, &set, NULL);

  if (ret != OK)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: sigprocmask failed: %d\n",
              errcode);
      goto errout;
    }
  

  /* Register alarm signal handler */

  act.sa_sigaction = alarm2_handler;
  act.sa_flags     = SA_SIGINFO;

  sigfillset(&act.sa_mask);
  sigdelset(&act.sa_mask, CONFIG_MYAPPS_ALARM2_SIGNO);

  ret = sigaction(CONFIG_MYAPPS_ALARM2_SIGNO, &act, NULL);
  if (ret < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: sigaction failed: %d\n",
              errcode);
      goto errout;
    }
  

  /* Now loop forever, waiting for alarm signals */

  for (; ; )
    {
      /*
       * Check if any alarms fired.
       *
       * NOTE that there are race conditions here... if we missing an alarm,
       * we will just report it a half second late.
       */
       
      for (i = 0; i < CONFIG_RTC_NALARMS; i++)
        {
          if (g_alarm2_received[i])
            {
              printf("alarm_demon: alarm %d received\n", i)  ;
              g_alarm2_received[i] = false;
            }
        }

      /* Now wait a little while and poll again.  If a signal is received
       * this should cuase us to awken earlier.
       */

      usleep(500*1000L);
    }

errout:
  g_alarm2_daemon_started = false;

  printf("alarm2_daemon: Terminating\n");
  return EXIT_FAILURE;
}

/****************************************************************************
 * Name: start_daemon
 ****************************************************************************/

static int start_daemon(void)
{
  FAR char *alarm2argv[2];

  if (!g_alarm2_daemon_started)
    {
      alarm2argv[0] = "alarm2_daemon";
      alarm2argv[1] = NULL;

      g_alarm2_daemon_pid =
        task_create("alarm2_daemon", CONFIG_MYAPPS_ALARM2_PRIORITY,
                    CONFIG_MYAPPS_ALARM2_STACKSIZE, alarm2_daemon,
                    (FAR char * const *)alarm2argv);
      if (g_alarm2_daemon_pid < 0)
        {
          int errcode = errno;
          fprintf(stderr, "ERROR: Failed to start alarm2_daemon: %d\n",
                 errcode);
          return -errcode;
        }

      printf("alarm2_daemon started\n");
      usleep(1000000);
    }

  return OK;
}

/****************************************************************************
 * Name: show_usage
 ****************************************************************************/

static void show_usage(FAR const char *progname)
{
  fprintf(stderr, "USAGE:\n");
#if CONFIG_RTC_NALARMS > 1
  fprintf(stderr, "\t%s [-a <alarmid>] <seconds>\n", progname);
  fprintf(stderr, "Where:\n");
  fprintf(stderr, "\t-a <alarmid>\n");
  fprintf(stderr, "\t\t<alarmid> selects the alarm: 0..%d\n",
          CONFIG_RTC_NALARMS - 1);
#else
  fprintf(stderr, "\t%s <seconds>\n", progname);
  fprintf(stderr, "Where:\n");
#endif
  fprintf(stderr, "\t<seconds>\n");
  fprintf(stderr, "\t\tThe number of seconds until the alarm expires.\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * alarm_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int alarm2_main(int argc, FAR char *argv[])
#endif
{
  struct rtc_setrelative_s setrel;
  unsigned long seconds;
  int alarm2id = 0;
  int secndx;
  int fd;
  int ret;

  /*struct rtc_time date;
  fd = open(CONFIG_MYAPPS_ALARM2_DEVPATH, O_WRONLY);
  ret = ioctl(fd,RTC_RD_TIME,&date);
  if(ret){
    printf("%d\n",date.tm_sec);
    printf("%d\n",date.tm_min);
    printf("%d\n",date.tm_hour);
    printf("%d\n",date.tm_mday);
    printf("%d\n",date.tm_mon);
    printf("%d\n",date.tm_year);
  }
  (void)close(fd);*/

  
  /*struct timespec t1;
  struct timespec t2;
  uint32_t dt_nsec;
  #define RDELAY 300
  clock_gettime(CLOCK_REALTIME, &t1);
  printf("%d\n",t1.tv_nsec);

  do{
    clock_gettime(CLOCK_REALTIME, &t2);
    dt_nsec = (t2.tv_nsec - t1.tv_nsec);
  }while(dt_nsec<RDELAY);

  
  printf("%d\n",t2.tv_nsec);
  
  
  printf("\n%d\n",dt_nsec);*/
  


  ret = start_daemon();
    if (ret < 0)
      {
        fprintf(stderr, "ERROR: Unrecognized option: %s\n", argv[1]);
        show_usage(argv[0]);
        return EXIT_FAILURE;
      }

#if CONFIG_RTC_NALARMS > 1

  if (argc == 4)
    {
      if (strcmp(argv[1], "-a") != 0)
        {
          fprintf(stderr, "ERROR: Unrecognized option: %s\n", argv[1]);
          show_usage(argv[0]);
          return EXIT_FAILURE;
        }

      alarm2id = atoi(argv[2]);
      secndx = 3;
    }
  else
#endif

  if (argc == 2)
    {
       secndx = 1;
    }
  else
    {
      fprintf(stderr, "ERROR: Invalid number of arguments: %d\n", argc - 1);
      show_usage(argv[0]);
      return EXIT_FAILURE;
    }

  printf("Get number of seconds\n");
  seconds = strtoul(argv[secndx], NULL, 10);
  if (seconds < 1)
    {
      fprintf(stderr, "ERROR: Invalid number of seconds: %lu\n", seconds);
      show_usage(argv[0]);
      return EXIT_FAILURE;
    }


  printf("Opening %s\n", CONFIG_MYAPPS_ALARM2_DEVPATH);
  fd = open(CONFIG_MYAPPS_ALARM2_DEVPATH, O_WRONLY);
  if (fd < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to open %s: %d\n",
              CONFIG_MYAPPS_ALARM2_DEVPATH, errcode);
      return EXIT_FAILURE;
    }

  printf("Set the alarm\n");
  setrel.id      = alarm2id;
  setrel.signo   = CONFIG_MYAPPS_ALARM2_SIGNO;
  setrel.pid     = g_alarm2_daemon_pid;
  setrel.reltime = (time_t)seconds;

  setrel.sigvalue.sival_int = alarm2id;

  fd = ioctl(fd, RTC_SET_RELATIVE, (unsigned long)((uintptr_t)&setrel));
  if (fd < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: RTC_SET_RELATIVE ioctl failed: %d\n",
              errcode);
      (void)close(fd);
      return EXIT_FAILURE;
    }

  printf("Alarm set in %lu seconds\n", seconds);
  
  (void)close(fd);
  

  return EXIT_SUCCESS;
}
