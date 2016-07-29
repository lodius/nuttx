#include <nuttx/config.h>

#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

void show_usage(void);

void show_usage(void){
  printf("***\nError : wrong input detected\n");
  printf("Application usage is:\n");
  printf("wpa SSID Password\n***\n");
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int wap_main(int argc, char *argv[])
#endif
{
  char SSID[64];
  char PASS[64];
  int i = 2;
  if(argc < 2){
    show_usage();
    return 0;
  }

  strcpy(SSID,argv[1]);
  strcpy(PASS,argv[argc-1]);

  while(i < (argc - 1)){
    strcat(SSID," ");
    strcat(SSID,argv[i]);
    i++;
  }

  printf("ssid is %s\n",SSID);
  printf("pass is %s\n",argv[argc-1]);

  int fd = open("/dev/ttyS1",O_RDWR);

  if(connect_ap(fd,SSID,PASS)){
    printf("Connect to WPA success\n");
  }
  else
    printf("Connect to WAP failed\n");

  return 0;
}