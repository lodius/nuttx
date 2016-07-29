#include <nuttx/config.h>

#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

static char buffer_console[512];

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int console_main(int argc, char *argv[])
#endif
{
	int ret,size,i=2;
	memset(&buffer_console[0], 0, sizeof(buffer_console));


	if(argc < 1){
		printf("Please enter command to send\n");
		printf("Structure : console <command>\n");
		return 0;
	}

	printf("%d\n",argc);
	char cmd[256] = "AT+";
	strcat(cmd,argv[1]);

	while(i <= argc-1){
		strcat(cmd," ");
		strcat(cmd,argv[i]);
		i++;
	}

	strcat(cmd,"\r\n");


	int fd = open("/dev/ttyS1",O_RDWR);
	if(fd)
		printf("Device ttyS1 opened\n");
	else
		printf("Error opening\n");

	printf("Writing to serial\n");

	ret = write(fd,cmd,strlen(cmd));
	printf("Sent : %d\n",ret);

	sleep(3);

	ioctl(fd,FIONREAD,&size);

	if(size) {
		read(fd,&buffer_console,size);
	}		

	printf("%s\n",buffer_console);

	//connect_ap2(fd);
	//connect_site(fd);

	/*if(start_module(fd))
		printf("Start module succeed\n");
	else
		printf("Failed to start module\n");*/

	//stm32_esp8266send(fd,"temp",35,"humi", 69);
	

	return 0;
}