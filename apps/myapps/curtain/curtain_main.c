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

void tb6612_gpioconfig(void);

void tb6612_stbyon(void);
void tb6612_stbyoff(void);

void tb6612_amotor_cw(void);
void tb6612_amotor_ccw(void);
void tb6612_amotor_stop(void);

void tb6612_bmotor_cw(void);
void tb6612_bmotor_ccw(void);
void tb6612_bmotor_stop(void);

void open_curtain(int duty,uint32_t freq,int time){

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
  else if(time < 1)
  {
    printf("Time must be positive\n");
    return;
  }

  info.duty = ((uint32_t)duty << 16) / 100; //Ranged:1-99
  info.frequency = freq; //Max 100KHz
  int pwm0  = open("/dev/pwm0", O_RDONLY);
  //Configure device
  ret = ioctl(pwm0, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
  if (ret < 0)
  {
    printf("ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
    return;
  }

  //Start device
  ret = ioctl(pwm0, PWMIOC_START, 0);
  if (ret < 0)
  {
    printf("ioctl(PWMIOC_START) failed: %d\n", errno);
    return;
  }

  tb6612_amotor_cw();
  sleep(time);
  tb6612_amotor_stop();

  ret = ioctl(pwm0, PWMIOC_STOP, 0);
  if (ret < 0)
  {
    printf("pwm_main: ioctl(PWMIOC_STOP) failed: %d\n", errno);
    return;
  }
  close(pwm0);
}

void close_curtain(int duty,uint32_t freq,int time){
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
  else if(time < 1)
  {
    printf("Time must be positive\n");
    return;
  }

  info.duty = ((uint32_t)duty << 16) / 100; //Ranged:1-99
  info.frequency = freq; //Max 100KHz
  int pwm0  = open("/dev/pwm0", O_RDONLY);
  //Configure device
  ret = ioctl(pwm0, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
  if (ret < 0)
  {
    printf("ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
    return;
  }

  //Start device
  ret = ioctl(pwm0, PWMIOC_START, 0);
  if (ret < 0)
  {
    printf("ioctl(PWMIOC_START) failed: %d\n", errno);
    return;
  }

  tb6612_amotor_ccw();
  sleep(time);
  tb6612_amotor_stop();
  ret = ioctl(pwm0, PWMIOC_STOP, 0);
  if (ret < 0)
  {
    printf("pwm_main: ioctl(PWMIOC_STOP) failed: %d\n", errno);
    return;
  }
  close(pwm0);
}


#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int curtain_main(int argc, char *argv[])
#endif
{
    int input,sec;
    if(argc<3){
      printf("Enter 0 to close and 1 to open curtain\n");
      printf("Enter seconds to run\n");
      return 0;
    }
    tb6612_gpioconfig();
    tb6612_stbyoff();

    input = strtoul(argv[1],NULL,10);
    sec = strtoul(argv[2],NULL,10);

    if(input){
      open_curtain(30,30000,sec);
      printf("Curtain openned\n");
    }
    else if(!input){
      close_curtain(30,30000,sec);
      printf("Curtain closed\n");
    }

    return 0;
  }

