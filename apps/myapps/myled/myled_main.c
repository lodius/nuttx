
#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/arch.h>

#include <sys/ioctl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/leds/userled.h>

static bool g_led_daemon_started;

static int led_daemon(int argc, char *argv[])
{
  userled_set_t supported;
  userled_set_t ledset;
  bool incrementing;
  int ret;
  int fd;

  /* Indicate that we are running */

  g_led_daemon_started = true;
  printf("led_daemon: Running\n");

  /* Open the LED driver */

  printf("led_daemon: Opening %s\n", CONFIG_MYAPPS_MYLED_DEVPATH);
  fd = open(CONFIG_MYAPPS_MYLED_DEVPATH, O_WRONLY);
  if (fd < 0)
    {
      int errcode = errno;
      printf("led_daemon: ERROR: Failed to open %s: %d\n",
             CONFIG_MYAPPS_MYLED_DEVPATH, errcode);
      goto errout;
    }

  /* Now loop forever, changing the LED set */

  ledset       = 0;
  incrementing = true;
  supported    = 0x0F;
  for (; ; )
    {
      userled_set_t newset;
      userled_set_t tmp;

      if (incrementing)
        {
          tmp = ledset;
          do
            {
              tmp++;
              newset = tmp & supported;
            }
          while (newset == ledset);

          /* REVISIT: There are flaws in this logic.  It would not work
           * correctly if there were spaces in the supported mask.
           */

          if (newset == 0)
            {
              incrementing = false;
              continue;
            }
        }
      else
        {
          /* REVISIT: There are flaws in this logic.  It would not work
           * correctly if there were spaces in the supported mask.
           */

          if (ledset == 0)
            {
              incrementing = true;
              continue;
            }

          tmp = ledset;
          do
            {
              tmp--;
              newset = tmp & supported;
            }
          while (newset == ledset);
        }

      ledset = newset;
      printf("led_daemon: LED set 0x%02x\n", (unsigned int)ledset);

      ret = write(fd, &ledset, 1);
      if (ret < 0)
        {
          int errcode = errno;
          printf("led_daemon: ERROR: write(ULEDIOC_SUPPORTED) failed: %d\n",
                 errcode);
          goto errout_with_fd;
        }

      usleep(500*1000L);
    }

errout_with_fd:
  (void)close(fd);

errout:
  g_led_daemon_started = false;

  printf("led_daemon: Terminating\n");
  return EXIT_FAILURE;
}

/****************************************************************************
*
****************************************************************************/


#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int myled_main(int argc, FAR char *argv[])
#endif
{
  FAR char *ledargv[2];
  int ret;
  led_init();
  printf("leds_main: Starting the led_daemon\n");
  if (g_led_daemon_started)
    {
      printf("leds_main: led_daemon already running\n");
      return EXIT_SUCCESS;
    }

  ledargv[0] = "led_daemon";
  ledargv[1] = NULL;

  ret = task_create("led_daemon", CONFIG_MYAPPS_MYLED_PRIORITY,
                    CONFIG_MYAPPS_MYLED_STACKSIZE, led_daemon,
                    (FAR char * const *)ledargv);
  if (ret < 0)
    {
      int errcode = errno;
      printf("leds_main: ERROR: Failed to start led_daemon: %d\n",
             errcode);
      return EXIT_FAILURE;
    }

  printf("leds_main: led_daemon started\n");
  return EXIT_SUCCESS;
}
