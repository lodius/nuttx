#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <nuttx/sensors/bh1750fvi.h>

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int lsense_main(int argc, char *argv[])
#endif
{
  int fd;
  uint8_t buf1[2];
  //uint16_t buf2;
  fd = open("/dev/light0",O_RDWR);
  while(1){
    read(fd,&buf1,2);
    printf("Light : %u lx\n",((buf1[1]<<8) | (buf1[0])) * 10/12  );
    
    /*ioctl(fd,SNIOC_READ16,&buf2);
    printf("Light : %u lx\n",buf2*10/12);*/
    usleep(500000);
  }
  close(fd);
  
  return 0;
}
