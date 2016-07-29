#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/boardctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

void config_fan(void);
void fan_on(void);
void fan_off(void);

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int fan_main(int argc, char *argv[])
#endif
{
    int input;

    if(argc<2){
      printf("Enter 0 to stop and 1 to run fan\n");
      return 0;
    }
    config_fan();

    input = strtoul(argv[1],NULL,10);

    if(input){
      fan_on();
      printf("Started fan\n");
    }
    else if(!input){
      fan_off();
      printf("Stopped fan\n");
    }

    return 0;
  }

