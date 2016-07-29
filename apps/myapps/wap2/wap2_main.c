#include <nuttx/config.h>

#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>


void show_usage2(void){
  printf("***\nError : wrong input detected\n");
  printf("Application usage is:\n");
  printf("wpa SSID\n***\n");
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int wap2_main(int argc, char *argv[])
#endif
{
  char SSID[64];
  int i = 2;
  if(argc < 2){
    show_usage2();
    return 0;
  }

  strcpy(SSID,argv[1]);

  while(i < argc){
    strcat(SSID," ");
    strcat(SSID,argv[i]);
    i++;
  }

  printf("public ssid is %s\n",SSID);

  int fd = open("/dev/ttyS1",O_RDWR);

  if(connect_ap(fd,SSID,"")){
    printf("Connect to WPA success\n");
    sleep(1);
  }
  else
    printf("Connect to WAP failed\n");

  return 0;
}