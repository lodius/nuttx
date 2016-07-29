#include <nuttx/config.h>

#include <stdio.h>

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>




int connect_site3(int fd, FAR char *inURL);

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int connect_main(int argc, char *argv[])
#endif
{
    int fd = ("/dev/ttyS1",O_RDWR);
    if(argc < 2){
    	printf("Usage : connect <site>\n");
    	return 0;
    }
    if(connect_site3(fd,argv[1]))
      printf("Connect to site successfully\n");
    else
      printf("Failed to connect to site\n");

    return 0;
}

