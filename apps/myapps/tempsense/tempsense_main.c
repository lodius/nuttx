#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <nuttx/sensors/sht10.h>




#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int tempsense_main(int argc, char *argv[])
#endif
{
  int fd,ret,finish=1;
  uint16_t temp,humi;
  fd = open("/dev/sht10",O_RDWR);
  
  ioctl(fd,SNIOC_START_MEASURE,0);
  finish = ioctl(fd,SNIOC_FINISH,0);

  do{
  	ioctl(fd,SNIOC_MEASURE,0);
  	finish = ioctl(fd,SNIOC_FINISH,0);
  }while(!finish);

  temp = ioctl(fd,SNIOC_READT,0);
  printf("Temperature : %d\n",temp);

  humi = ioctl(fd,SNIOC_READH,0);
  printf("Humidity : %d\n",humi);

  close(fd);
  return 0;
}
