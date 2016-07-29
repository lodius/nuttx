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


void run_motor(int duty,uint32_t freq,int time){

  struct pwm_info_s info;
  int fd1,fd2;
  int ret;

  if(duty<1||duty >99)
  {
    printf("Duty must be in range 1-99\n");
    return;
  }
  else if(freq <1||freq>100000)
  {
    printf("Frequency must be in range 1-100K");
    return;
  }
  else if(time <1)
  {
    printf("Time invalid");
    return;
  }

  info.duty = ((uint32_t)duty << 16) / 100; //Ranged:1-99
  info.frequency = freq; //Max 100KHz

  //Open device
  fd1 = open("/dev/pwm0", O_RDONLY);
  fd2 = open("/dev/pwm1", O_RDONLY);
  if (fd1 < 0|fd2 < 0)
  {
    printf("Open /dev/pwm0 failed: %d\n", errno);
    return;
  }

  printf("Starting output with frequency: %u duty: %08x\n",
         info.frequency, info.duty);

  //Configure device
  ret = ioctl(fd1, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
  ret = ioctl(fd2, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
  if (ret < 0)
  {
    printf("ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
    return;
  }

  //Start device
  ret = ioctl(fd1, PWMIOC_START, 0);
  ret = ioctl(fd2, PWMIOC_START, 0);
  if (ret < 0)
  {
    printf("ioctl(PWMIOC_START) failed: %d\n", errno);
    return;
  }

  tb6612_gpioconfig();

  tb6612_stbyoff();
  tb6612_amotor_cw();
  tb6612_bmotor_cw();

  sleep(time);

  tb6612_amotor_ccw();
  //tb6612_bmotor_ccw(); may bom khong chay nguoc chieu duoc

  sleep(time);

  //Stop device
  tb6612_amotor_stop();
  tb6612_bmotor_stop();
  tb6612_stbyon();

  printf("Stopping device\n");
  ret = ioctl(fd1, PWMIOC_STOP, 0);
  if (ret < 0)
  {
    printf("pwm_main: ioctl(PWMIOC_STOP) failed: %d\n", errno);
    return;
  }

  close(fd1);
  close(fd2);
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int motor_main(int argc, char *argv[])
#endif
{
  
  run_motor(30,30000,7);
  return 0;
}

