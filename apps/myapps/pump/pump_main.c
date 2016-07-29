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

#include <nuttx/pwm.h>

int pwm1;

void start_pump(int duty,uint32_t freq){

  struct pwm_info_s info;
  int ret;

  if(duty<1||duty >99)
  {
    printf("Duty must be in range 1-99\n");
    return;
  }
  else if(freq <1||freq>100000)
  {
    printf("Frequency must be in range 1-100K\n");
    return;
  }

  info.duty = ((uint32_t)duty << 16) / 100; //Ranged:1-99
  info.frequency = freq; //Max 100KHz

  //pwm1  = open("/dev/pwm1", O_RDONLY);
  
  //Configure device
  ret = ioctl(pwm1, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
  if (ret < 0)
  {
    printf("ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
    return;
  }

  //Start device
  ret = ioctl(pwm1, PWMIOC_START, 0);
  if (ret < 0)
  {
    printf("ioctl(PWMIOC_START) failed: %d\n", errno);
    return;
  }

  tb6612_bmotor_cw();
}

void stop_pump(void){
  int ret;
  //pwm1  = open("/dev/pwm1", O_RDONLY);
  tb6612_bmotor_stop();
  ret = ioctl(pwm1, PWMIOC_STOP, 0);
  if (ret < 0)
  {
    printf("pwm_main: ioctl(PWMIOC_STOP) failed: %d\n", errno);
    return;
  }
  //close(pwm1);
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int pump_main(int argc, char *argv[])
#endif
{
  int input;

  if(argc<2){
    printf("Enter number of seconds to run\n");
    printf("pump <seconds>\n");
    return 0;
  }
  pwm1 = open("/dev/pwm1",O_RDWR);
  tb6612_gpioconfig();
  tb6612_stbyoff();

  input = strtoul(argv[1],NULL,10);

  printf("Started pump for %d secs\n",input);

  start_pump(40,40000);
  sleep(input);
  stop_pump();


  return 0;
}

