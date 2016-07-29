#include <nuttx/config.h>

#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/timers/rtc.h>

#define ESPDEBUG 1

#define SSID	"Tien Dung"
#define PASS	"Quachhoang091194"

#define URL   "dweet.io"//"api.thingspeak.com"
#define URL2  "noniscope.hoxty.com"
#define URL3  "noniscope.localtunnel.me"

#define PORT        "80"//443 is for secure socket which seems not to be supported at this point on the ESP8266

static char inURL[32];

#define thing_name  "quake"
#define MAX_SERVER_CONNECT_ATTEMPTS 5 

static char ok[] = "OK\r\n";

struct data_s{
  int value;
  char key[16];
};

int length(int x);
int match(const char *str, const char *sub);
int wait(int fd, int timeout, char* term);
int reset_module(int fd);
int connect_ap(int fd,char *netssid,char *password);
int connect_site(int fd);
int stm32_esp8266start(int fd,char *netssid,char *password);

void stm32_esp8266send(int fd,	char const *key1, int datain1,
								char const *key2, int datain2);

int stm32_esp8266initialize(void);
void stm32_esp8266send_multi(int fd, const struct data_s data[], int num);
int stm32_esp8266send_data(int fd, int id, int temp, int humi, int light, int ghumi,int curtain, int pump, int fan);


int length(int x) {
   if(x>=100000) {
        if(x>=10000000) {
            if(x>=1000000000) return 10;
            if(x>=100000000) return 9;
            return 8;
        }
        if(x>=1000000) return 7;
        return 6;
    } else {
        if(x>=1000) {
            if(x>=10000) return 5;
            return 4;
        } else {
            if(x>=100) return 3;
            if(x>=10) return 2;
            return 1;
        }
    }
}

int match(const char *str, const char *sub)
{
    int length = strlen(sub);
    if (length == 0) return 0;
    int count = 0;
    for (str = strstr(str, sub); str; str = strstr(str + length, sub))
        ++count;
    return count;
}

int wait(int fd, int timeout, char* term) {
	struct timespec t1;
	struct timespec t2;

  //char buf_int[256];
  char *buf_int;
  buf_int = (char *)malloc(512*sizeof(char));

	int size;
	int found=0;

	clock_gettime(CLOCK_REALTIME,&t1);
 	
	do{
		ioctl(fd,FIONREAD,(unsigned long)&size);
		
		if(size) {
			read(fd,buf_int,size);

			if(ESPDEBUG)
				printf("%s\n",buf_int);
		
			if(match(buf_int, term)) {
				found = 1;
				break;
			}	
		}
		clock_gettime(CLOCK_REALTIME,&t2);
	}while(timeout > (t2.tv_sec - t1.tv_sec));
	free(buf_int);

	return found;
}

int reset_module(int fd){
	int respond;
	
	printf("Starting ESP8266 module\n");
	while(!respond){
    //software reset
    	write(fd,"AT+RST\r\n",8);//reset module (works with both ESP-01 and ESP-03 module)

    	if (wait(fd, 5, ok)){ //watch out for the case of the r in ready - varies with ESP8266 firmware version
    		printf("Module is responding\n");
    		respond = 1;
    	}
    	else{
    		printf("Module not responding to reset\n");
    		sleep(1);
    	}
    }

    return respond;
}

int connect_ap(int fd,FAR char *netssid,FAR char *password){
	int connected; 
	//append char array
	char cmd[256]="AT+CWJAP=\"";
	strcat(cmd,netssid);
	strcat(cmd,"\"");

  if(password){
    strcat(cmd,",\"");
    strcat(cmd,password);
    strcat(cmd,"\"");
  }

	strcat(cmd,"\r\n");

	write(fd,"AT+GMR\r\n",8);
	wait(fd,1,ok);

	write(fd,"AT+CWMODE=1\r\n",13);
	wait(fd,1,ok);

	write(fd,"AT+CIPMUX=0\r\n",13);
	wait(fd,1,ok);

	printf("Connecting to WiFi access point...\n");

	//asprintf(&cmd,"%c%c%c%c%c","AT+CWJAP=\"",SSID,"\",\"",PASS,"\"\r\n");

	write(fd, cmd, 20 + strlen(netssid) + strlen(password));

	printf(cmd);
	connected = wait(fd, 9, "K");

	if(!connected){
		printf("Attempt to connect to access point failed. Restarting module.\n");
		return 0; 
	}
	else
	{
		printf("\nCONNECTED TO ACCESS POINT\n");
	}

	return connected;
}

int connect_site(int fd){
	//append the char array
	char cmd[128]="AT+CIPSTART=\"TCP\",\"";
	strcat(cmd,URL);
  strcat(cmd,"\",");
  strcat(cmd,PORT);
  strcat(cmd,"\r\n");

	int connected_to_dweet = 0;
	int connection_attempts = 0;

	printf("Connecting to dweet...\n");

	while((!connected_to_dweet)&&(connection_attempts < MAX_SERVER_CONNECT_ATTEMPTS)){

		//asprintf(&cmd,"%c%c%c%c%c","AT+CIPSTART=\"TCP\",\"",URL,"\",",PORT,"\r\n");
		write(fd,cmd,27+sizeof(URL)-1+sizeof(PORT)-1);
		connected_to_dweet = wait(fd,9,"K");

    	//this needs to change - look for  something in server response that indicates valid connection
		connection_attempts += 1;
		if (!connected_to_dweet)
		{
			printf("Attempt to connect to %s did not succeed\n",URL);

		}
		else
		{
			printf("\nCONNECTED TO DWEET\n");
		}
	}

  return connected_to_dweet;
}

int connect_site2(int fd){
  //append the char array
  char cmd[128]="AT+CIPSTART=\"TCP\",\"";
  strcat(cmd,URL2);
    strcat(cmd,"\",");
    strcat(cmd,PORT);
    strcat(cmd,"\r\n");

  int connected = 0;
  int connection_attempts = 0;

  printf("Connecting to %s...\n",URL2);

  while((!connected)&&(connection_attempts < MAX_SERVER_CONNECT_ATTEMPTS)){

    //asprintf(&cmd,"%c%c%c%c%c","AT+CIPSTART=\"TCP\",\"",URL,"\",",PORT,"\r\n");
    write(fd,cmd,27+sizeof(URL2)-1+sizeof(PORT)-1);
    connected = wait(fd,9,"K");

      //this needs to change - look for  something in server response that indicates valid connection
    connection_attempts += 1;
    if (!connected)
    {
      printf("Attempt to connect to %s did not succeed\n",URL2);

    }
    else
    {
      printf("\nCONNECTED TO %s\n",URL2);
    }
  }

  return connected;
}

int connect_site3(int fd, FAR char *inputURL){
  //append the char array
  strcpy(inURL,inputURL);
  char cmd[128]="AT+CIPSTART=\"TCP\",\"";
  strcat(cmd,inURL);
    strcat(cmd,"\",");
    strcat(cmd,PORT);
    strcat(cmd,"\r\n");

  int connected = 0;
  int connection_attempts = 0;

  printf("Connecting to %s...\n",inURL);

  while((!connected)&&(connection_attempts < MAX_SERVER_CONNECT_ATTEMPTS)){

    //asprintf(&cmd,"%c%c%c%c%c","AT+CIPSTART=\"TCP\",\"",URL,"\",",PORT,"\r\n");
    write(fd,cmd,27+strlen(inURL)+strlen(PORT)-1);
    connected = wait(fd,9,"K");

      //this needs to change - look for  something in server response that indicates valid connection
    connection_attempts += 1;
    if (!connected)
    {
      printf("Attempt to connect to %s did not succeed\n",inURL);

    }
    else
    {
      printf("\nCONNECTED TO %s\n",inURL);
    }
  }

  return connected;
}

int stm32_esp8266start(int fd,char *netssid,char *password)
{
  //Reset and test if the module is ready

  reset_module(fd);

  //Connect to access point
  
  connect_ap(fd,netssid,password);

  //Connect to dweet
  
  return connect_site(fd);
}

void stm32_esp8266send(int fd,	char const *key1, int datain1,
								char const *key2, int datain2)
{

	int len = 73;
	
	int datalen1 = length(datain1);
	int datalen2 = length(datain2);

	char 	data1[datalen1],
			data2[datalen2];

	sprintf(data1,"%ld",datain1);
	sprintf(data2,"%ld",datain2);


  	len += (sizeof(thing_name) - 1) + (datalen1) + (datalen2) + strlen(key1) + strlen(key2);
  	

  	
  	//sizeof will return 11 for a 10 character thing name because of null terminator so subtract 1
  	char 	datalen[length(len)];
	sprintf(datalen,"%ld",len);
  	//form and send the HTTP POST message
  	write(fd,"AT+CIPSEND=",11);
  	write(fd,datalen,strlen(datalen));
  	write(fd,"\r\n",2);
  	
  	if (!wait(fd,9,"> ")){
  		write(fd,"AT+CIPCLOSE",11);
  		printf("Send timeout please reconnect wifi module");
  		return;
  	}

  	write(fd,"POST /dweet/for/",16);  
  	//write(fd,"GET /dweet/for/",15);// both POST and GET work

  	write(fd,thing_name,sizeof(thing_name)-1);
 
  	write(fd,"?",1);
  	write(fd,key1,strlen(key1));
  	write(fd,"=",1);
  	write(fd,data1,datalen1);

  	write(fd,"&",1);
  	write(fd,key2,strlen(key2));
  	write(fd,"=",1);
  	write(fd,data2,datalen2);

  	write(fd," HTTP/1.1\r\n",11);
  	write(fd,"Host: dweet.io\r\n",16);

  	//write(fd,"Connection: close\r\n",19);//in some cases connection needs to be closed after POST
  	write(fd,"Connection: keep-alive\r\n",24);//in this case we want to keep the connection alive
  	write(fd,"\r\n",2);
  
  	if(wait(fd,9,"OK"))//}}}\r\n
  		printf("Sent data successfully\n");
  	else
  		printf("Send data fail\n");

}

void stm32_esp8266send_multi(int fd, const struct data_s data[], int num)
{
  int len = 0;
  int i;
  char inputdata[16];

  if(num < 1){
    printf("Illegal number of data\n");
    return;
  }


  char cmd[256] = "POST /dweet/for/";
  strcat(cmd,thing_name);
  for(i = 0;i < num;i++){
    if(i==0)
      strcat(cmd,"?");
    else
      strcat(cmd,"&");

    sprintf(inputdata,"%ld",data[i].value);

    strcat(cmd,data[i].key);
    strcat(cmd,"=");
    strcat(cmd,inputdata);
  }
  strcat(cmd," HTTP/1.1\r\n");
  strcat(cmd,"Host: dweet.io\r\n");

  strcat(cmd,"Connection: keep-alive\r\n");
  strcat(cmd,"\r\n");

  len += strlen(cmd);

  //sizeof will return 11 for a 10 character thing name because of null terminator so subtract 1
  char  datalen[length(len)];
  sprintf(datalen,"%ld",len);
  //form and send the HTTP POST message
  write(fd,"AT+CIPSEND=",11);
  write(fd,datalen,strlen(datalen));
  write(fd,"\r\n",2);

  if (!wait(fd,9,">")){
    write(fd,"AT+CIPCLOSE",11);
    printf("\nSend timeout please reconnect wifi module\n");
    return;
  }

  write(fd,cmd,strlen(cmd));

  if(wait(fd,9,"OK"))//}}}\r\n
    printf("Sent data successfully\n");
  else
    printf("Send data fail\n");

}

int get_response(int fd, int timeout,char *term) {
  struct timespec t1;
  struct timespec t2;

  //char buf_ret[512];
  char *buf_ret;
  buf_ret = (char*)malloc(512*sizeof(char));

  int size;
  int ret = 0x000;

  clock_gettime(CLOCK_REALTIME,&t1);
  
  do{
    ioctl(fd,FIONREAD,(unsigned long)&size);
    
    if(size) {
      read(fd,buf_ret,size);//with or without &

      if(ESPDEBUG)
        printf("%s\n",buf_ret);
    
      if(match(buf_ret, term)) {
        break;
      } 
    }
    clock_gettime(CLOCK_REALTIME,&t2);
  }while(timeout > (t2.tv_sec - t1.tv_sec));
  
  if(match(buf_ret,"!"))
    ret=ret|0b001;
  
  if(match(buf_ret,"#"))
    ret=ret|0b010;

  if(match(buf_ret,"%"))
    ret=ret|0b100;

  free(buf_ret);

  return ret;
}

int stm32_esp8266send_data(int fd, int id, int temp, int humi, int light, int ghumi,int curtain, int pump, int fan)
{
  int len = 0;
  char inputdata[16];
  int buf;
  char cmd[256] = "GET /dataupdate.php";

  strcat(cmd,"?");
  strcat(cmd,"user_id");
  strcat(cmd,"=");
  sprintf(inputdata,"%ld",id);
  strcat(cmd,inputdata);
  
  strcat(cmd,"&");
  strcat(cmd,"temp");
  strcat(cmd,"=");
  sprintf(inputdata,"%ld",temp);
  strcat(cmd,inputdata);

  strcat(cmd,"&");
  strcat(cmd,"humi");
  strcat(cmd,"=");
  sprintf(inputdata,"%ld",humi);
  strcat(cmd,inputdata);

  strcat(cmd,"&");
  strcat(cmd,"light");
  strcat(cmd,"=");
  sprintf(inputdata,"%ld",light);
  strcat(cmd,inputdata);

  strcat(cmd,"&");
  strcat(cmd,"ghumi");
  strcat(cmd,"=");
  sprintf(inputdata,"%ld",ghumi);
  strcat(cmd,inputdata);

  strcat(cmd,"&");
  strcat(cmd,"curtain");
  strcat(cmd,"=");
  sprintf(inputdata,"%ld",curtain);
  strcat(cmd,inputdata);

  strcat(cmd,"&");
  strcat(cmd,"pump");
  strcat(cmd,"=");
  sprintf(inputdata,"%ld",pump);
  strcat(cmd,inputdata);

  strcat(cmd,"&");
  strcat(cmd,"fan");
  strcat(cmd,"=");
  sprintf(inputdata,"%ld",fan);
  strcat(cmd,inputdata);
  
  strcat(cmd," HTTP/1.1\r\n");
  strcat(cmd,"Host: ");
  strcat(cmd,inURL);
  strcat(cmd,"\r\n");

  strcat(cmd,"Connection: keep-alive\r\n");
  strcat(cmd,"\r\n");

  len += strlen(cmd);

  //int to char conversion
  char  datalen[length(len)];
  sprintf(datalen,"%ld",len);
  //form and send the HTTP POST message
  write(fd,"AT+CIPSEND=",11);
  write(fd,datalen,strlen(datalen));
  write(fd,"\r\n",2);

  if (!wait(fd,9,">")){
    write(fd,"AT+CIPCLOSE",11);
    printf("\nSend timeout please reconnect wifi module\n");
    return 0;
  }
  
  write(fd,cmd,strlen(cmd));

  buf = get_response(fd,5,"K&*");
  return buf;
}

int stm32_esp8266initialize(void){
	int fd = open("/dev/ttyS1",O_RDWR);

	if(reset_module(fd)){
		printf("Started ESP8266 wireless module successfully\n");
		return 1;
	}
	else{
		printf("Failed to start ESP8266 module\n");
		return -ENODEV;
	}

	return fd;
}
