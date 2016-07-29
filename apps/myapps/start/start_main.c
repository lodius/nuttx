#include <nuttx/config.h>

#include <sys/mount.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/boardctl.h>

#include <stdio.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <sched.h>

#include <nuttx/analog/adc.h>
#include <nuttx/sensors/sht10.h>
#include <nuttx/sensors/bh1750fvi.h>
#include <nuttx/pwm.h>

//Requirements :
//CONFIG_NFILE_DESCRIPTORS=12
//CONFIG_NFILE_STREAMS=12

//Data declaration


static int temp,humi,light,ghumi;
static bool curtain = false,pump = false,fan = false;

static pthread_t thread_data_s;
static pthread_cond_t cond_data = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t lock_data = PTHREAD_MUTEX_INITIALIZER;
static int timeout_data = 10;
static int signal_data = 0;

static int 	signal_cur_op,	signal_cur_cl,
			signal_pump_on,		signal_pump_off,
			signal_fan_on,		signal_fan_off;

static pthread_t thread_cur_s;
static pthread_cond_t cond_cur = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t lock_motor = PTHREAD_MUTEX_INITIALIZER;
static int signal_motor = 0;
#define OPEN_CUR 	1
#define CLOSE_CUR 	2

static int pwm1;

//Functions declaration

void tb6612_gpioconfig(void);

void tb6612_stbyon(void);
void tb6612_stbyoff(void);

void tb6612_amotor_cw(void);
void tb6612_amotor_ccw(void);
void tb6612_amotor_stop(void);

void tb6612_bmotor_cw(void);
void tb6612_bmotor_ccw(void);
void tb6612_bmotor_stop(void);

void config_fan(void);
void fan_on(void);
void fan_off(void);


int match(const char *str, const char *sub);
int stm32_esp8266send_data(int fd, int id, int temp, int humi, int light, int ghumi,int curtain, int pump, int fan);


int gsensor_get(int gsensor);
void motor_a_cw(int duty,uint32_t freq,int time);
void motor_a_ccw(int duty,uint32_t freq,int time);
void motor_b_run(int duty,uint32_t freq);
void motor_b_stop(void);
void fan_run(void);
void fan_stop(void);

//Private Functions

void motor_a_cw(int duty,uint32_t freq,int time){

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
  int pwm0 	= open("/dev/pwm0", O_RDONLY);
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

void motor_a_ccw(int duty,uint32_t freq,int time){

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
  int pwm0 	= open("/dev/pwm0", O_RDONLY);
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

void motor_b_run(int duty,uint32_t freq){

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

  //pwm1 	= open("/dev/pwm1", O_RDONLY);
  
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

void motor_b_stop(void){
	int ret;
	//pwm1 	= open("/dev/pwm1", O_RDONLY);
	tb6612_bmotor_stop();
	ret = ioctl(pwm1, PWMIOC_STOP, 0);
	if (ret < 0)
	{
		printf("pwm_main: ioctl(PWMIOC_STOP) failed: %d\n", errno);
		return;
	}
	//close(pwm1);
}

int gsensor_get(int gsensor){
	struct adc_msg_s sample[CONFIG_MYAPPS_GSENSOR_GROUPSIZE];
	size_t readsize;
	size_t nbytes;
	//int am_channel;
	//int am_data;

	ioctl(gsensor, ANIOC_TRIGGER, 0);

    /* Read up to 4 samples for 4 sensors*/

	readsize = 4 * sizeof(struct adc_msg_s);
	nbytes = read(gsensor, sample, readsize);

    /* Handle unexpected return values */

	if (nbytes < 0)
	{
		printf("adc_main: read failed\n");
	}
	else if (nbytes == 0)
	{
		printf("adc_main: No data read, Ignoring\n");
	}

    /* Print the sample data on successful return */

	else
	{
		int nsamples = nbytes / sizeof(struct adc_msg_s);
		if (nsamples * sizeof(struct adc_msg_s) != nbytes)
		{
			printf("adc_main: read size=%ld is not a multiple of sample size=%d, Ignoring\n",
				(long)nbytes, sizeof(struct adc_msg_s));
		}
	}
	return (4095 - (sample[0].am_data+sample[1].am_data+sample[2].am_data+sample[3].am_data)/4);
}

void fan_run(void){
	fan_on();
}

void fan_stop(void){
	fan_off();
}

void log_sd(int log_temp, int log_humi, int log_light, int log_ghumi)
{
	/*char inputdata[16];
	char cmd[256]= "******************************\n";
	

	strcat(cmd,"Temperature       : ");
	sprintf(inputdata,"%ld",log_temp);
	strcat(cmd,inputdata);
	strcat(cmd," celcius\n");

	strcat(cmd,"Humidity          : ");
	sprintf(inputdata,"%ld",log_humi);
	strcat(cmd,inputdata);
	strcat(cmd," RH percent\n");

	strcat(cmd,"Light             : ");
	sprintf(inputdata,"%ld",log_light);
	strcat(cmd,inputdata);
	strcat(cmd," lx\n");

	strcat(cmd,"Ground Humidity   : ");
	sprintf(inputdata,"%ld",log_ghumi);
	strcat(cmd,inputdata);
	strcat(cmd," percent\n");

	write(sd,cmd,strlen(cmd));*/
	FILE *sd;
	sd = fopen("/sdcard/data.txt", "a");
	
	fprintf(sd,"******************************\n");
	fprintf(sd,"Temperature       : %d celcius\n",log_temp);
	fprintf(sd,"Humidity          : %d RH percent\n",log_humi);
	fprintf(sd,"Light             : %d lx\n",log_light);
	fprintf(sd,"Ground Humidity   : %d precent\n",log_ghumi);

	fflush(sd);
	fclose(sd);
	
}

void *thread_data(void *arg)
{
	int esp8266 = open("/dev/ttyS1",O_RDWR);
	

	if(esp8266 < 0){
		printf("Failed to open esp8266\n");
		return 0;
	}

	/*int sd = open("/sdcard/data.txt",O_WRONLY);
	if(sd < 0){
		printf("Failed to open sd card\n");
		return;
	}*/
	
	int buf;

	int buf1,buf2,buf3,buf4,buf5,buf6;
	int uid = (int)arg;
	int buf_temp,buf_humi,buf_light,buf_ghumi;

	bool buf_cur,buf_pump,buf_fan;

	while(1){
		pthread_mutex_lock(&lock_data);
		while(!signal_data){
			pthread_cond_wait(&cond_data,&lock_data);
		}
		signal_data = 0;
		buf_temp = temp;
		buf_humi = humi;
		buf_light = light;
		buf_ghumi = ghumi;

		pthread_mutex_unlock(&lock_data);

		buf_cur = curtain;
		buf_pump = pump;
		buf_fan = fan;

		buf = stm32_esp8266send_data(esp8266,uid,buf_temp,buf_humi,buf_light,buf_ghumi,buf_cur,buf_pump,buf_fan);

		printf("Data sent\n");
		printf("\n***\nPolled command : %d\n***\n",buf);

		log_sd(buf_temp,buf_humi,buf_light,buf_ghumi);

		printf("Data logged\n");

		if((buf & 0b001)==0b001){
			printf("Openning curtain\n");
			buf1 = 1;
		}
		else {
			printf("Closing curtain\n");
			buf2 = 1;
		}

		if((buf & 0b010)==0b010){
			printf("Turning pump on\n");
			buf3 = 1;
		}
		else {
			printf("Turning pump off\n");
			buf4 = 1;
		}

		if((buf & 0b100)==0b100){
			printf("Turning fan on\n");
			buf5 = 1;
		}
		else {
			printf("Turning fan off\n");
			buf6 = 1;
		}

		//pthread_mutex_lock(&lock_poll);
		signal_cur_op = buf1;
		signal_cur_cl = buf2;
		signal_pump_on = buf3;
		signal_pump_off = buf4;
		signal_fan_on = buf5;
		signal_fan_off = buf6;
		//pthread_mutex_unlock(&lock_poll);

		buf1 = 0;
		buf2 = 0;
		buf3 = 0;
		buf4 = 0;
		buf5 = 0;
		buf6 = 0;
	}
}

void *thread_cur(void *arg){
	int buf_signal;

	/*struct sched_param param;
	param.sched_priority = 100;
	if (sched_setscheduler(getpid(), SCHED_RR, & param) != 0) {
		perror("sched_setscheduler");
		exit(EXIT_FAILURE);  
	}*/

	while(1){
		pthread_mutex_lock(&lock_motor);
		while(!signal_motor){
			pthread_cond_wait(&cond_cur,&lock_motor);
		}
		buf_signal = signal_motor;
		signal_motor = 0;
		pthread_mutex_unlock(&lock_motor);

		if(buf_signal == OPEN_CUR)
			motor_a_cw(30,30000,5);
		else if(buf_signal == CLOSE_CUR)
			motor_a_ccw(30,30000,5);
	}
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int start_main(int argc, char *argv[])
#endif
{
	int uid;
	int gsensor,sht10,bh1750;
	struct timespec t1;
	struct timespec t2;

	int buf_temp,buf_humi,buf_light,buf_ghumi;
	int buf_cur_op,buf_cur_cl,buf_pump_on,buf_pump_off,buf_fan_on ,buf_fan_off;

	uint8_t buf_2[2];

	if(argc<2){
		printf("Enter user id\n");

		return 0;
	}
	else{
		uid = strtoul(argv[1],NULL,10);
	}

	gsensor = open("/dev/gsensor",O_RDONLY);
	sht10 	= open("/dev/sht10",O_RDWR);
	bh1750 	= open("/dev/light0",O_RDWR);
	pwm1 	= open("/dev/pwm1",O_WRONLY);
	
	if(sht10 < 0){
		printf("Failed to open the sht10\n");
		return 0;
	} 
	else if(gsensor < 0){
		printf("Failed to open th gsensor\n");
		return 0;
	} 
	else if(bh1750 < 0){
		printf("Failed to open the bh1750\n");
		return 0;
	}

	struct sched_param param;
	param.sched_priority = 100;
	if (sched_setscheduler(getpid(), SCHED_RR, & param) != 0) {
		perror("sched_setscheduler");
		exit(EXIT_FAILURE);  
	}

	
	pthread_attr_t	attr_data;
	pthread_attr_t	attr_cur;

	attr_data.priority = 100;
	attr_cur.priority = 100;

	pthread_attr_init(&attr_data);
	pthread_attr_init(&attr_cur);

	if(pthread_attr_setschedpolicy(&attr_data,SCHED_RR)){
		printf("Set sched send failed\n");
	}
	if(pthread_attr_setschedpolicy(&attr_cur,SCHED_RR)){
		printf("Set sched cur failed\n");
	}

	printf("Starting thread data \n");
	if ( (pthread_create(&thread_data_s,&attr_data, (void*)thread_data, (void*)uid)) != 0)
	{
		fprintf(stderr, "Error in thread data creation\n");
		return 0;
	}

	printf("Starting thread curtain \n");
	if ( (pthread_create(&thread_cur_s, &attr_cur, (void*)thread_cur, NULL)) != 0)
	{
		fprintf(stderr, "Error in thread curtain creation\n");
		return 0;
	}

	printf("Setup\n");
	signal_data = 0;
	tb6612_gpioconfig();
	tb6612_stbyoff();
	config_fan();

	clock_gettime(CLOCK_REALTIME,&t1);
	clock_gettime(CLOCK_REALTIME,&t2);

	while(1){
    //Read sensor
		printf("***************************\n");
		printf("Reading sensors...\n");
		ioctl(sht10,SNIOC_START_MEASURE,0);
		do{
			ioctl(sht10,SNIOC_MEASURE,0);
		}while(!ioctl(sht10,SNIOC_FINISH,0));

		buf_temp = ioctl(sht10,SNIOC_READT,0);
		printf("Temperature : %d\n",buf_temp);
		buf_temp = (int)buf_temp/100;

		buf_humi = ioctl(sht10,SNIOC_READH,0);
		printf("Humidity : %d\n",buf_humi);
		buf_humi = (int)buf_humi/100;

		read(bh1750,&buf_2,2);
		printf("Light : %u lx\n",((buf_2[1]<<8) | (buf_2[0])) * 10/12  );
		buf_light = (int)( ( (buf_2[1]<<8) | (buf_2[0]) ) * 10/12  );

		buf_ghumi = gsensor_get(gsensor);
		buf_ghumi = (int)buf_ghumi*100/4095;
		printf("Ground humidity : %d\n",buf_ghumi);

		printf("***************************\n");
		
		pthread_mutex_lock(&lock_data);
		temp = buf_temp;
		humi = buf_humi;
		light = buf_light;
		ghumi = buf_ghumi;
		pthread_mutex_unlock(&lock_data);

		buf_cur_op = signal_cur_op;
		buf_cur_cl = signal_cur_cl;
		buf_pump_on = signal_pump_on;
		buf_pump_off = signal_pump_off;
		buf_fan_on = signal_fan_on;
		buf_fan_off = signal_fan_off;
	
    	//Check timeout whether to signal or not

		clock_gettime(CLOCK_REALTIME,&t2);
		if((t2.tv_sec-t1.tv_sec)>timeout_data){
			pthread_mutex_lock(&lock_data);
			signal_data = 1;
			pthread_mutex_unlock(&lock_data);

			pthread_cond_signal(&cond_data);
			printf("%d seconds passed\n",timeout_data);
			clock_gettime(CLOCK_REALTIME,&t1);
		}

    	//Automatic action
		
		
		/*if(!pump){
			if(buf_pump_on && !(buf_ghumi>70)){
				motor_b_run(50,50000);
				pump = true;
				printf("Started pump\n");
			}
			else if(buf_ghumi<10){//
				motor_b_run(50,50000);
				pump = true;
				printf("Started pump\n");
			}
		}
		else if(pump){
			if(buf_pump_off && !(buf_ghumi<10)){
				motor_b_stop();
				pump = false;
				printf("Stopped pump\n");
			}
			else if(buf_ghumi>80){//
				motor_b_stop();
				pump = false;
				printf("Stopped pump\n");
			}
		}


		if(curtain){
			if(buf_cur_cl){
				pthread_mutex_lock(&lock_motor);
				signal_motor = CLOSE_CUR;
				pthread_mutex_unlock(&lock_motor);

				pthread_cond_signal(&cond_cur);
				curtain = false;
				printf("Closed curtain\n");
			}
			else if(buf_light>20000){//
				pthread_mutex_lock(&lock_motor);
				signal_motor = CLOSE_CUR;
				pthread_mutex_unlock(&lock_motor);

				pthread_cond_signal(&cond_cur);
				curtain = false;
				printf("Closed curtain\n");
			}
		}
		else if(!curtain){
			if((buf_cur_op && !(buf_light>15000))){
				pthread_mutex_lock(&lock_motor);
				signal_motor = OPEN_CUR;
				pthread_mutex_unlock(&lock_motor);

				pthread_cond_signal(&cond_cur);
				curtain = true;
				printf("Opened curtain\n");
			}
			else if(buf_light<50){//
				pthread_mutex_lock(&lock_motor);
				signal_motor = OPEN_CUR;
				pthread_mutex_unlock(&lock_motor);

				pthread_cond_signal(&cond_cur);
				curtain = true;
				printf("Opened curtain\n");
			}	
		}
		
		if(fan){
			if(buf_fan_off && !(buf_humi>75)){
				fan_stop();
				fan = false;
				printf("Stopped fan\n");
			}
			else if(buf_humi<35){
				fan_stop();
				fan = false;
				printf("Stopped fan\n");
			}
		}
		else if(!fan){
			if(buf_fan_on && !(buf_humi<45)){
				fan_run();
				fan = true;
				printf("Started fan\n");
			}
			else if(buf_humi>85 || buf_temp>35){
				fan_run();
				fan = true;
				printf("Started fan\n");
			}
		}*/
		
		

		//Manual Action

		/*if((ghumi>80)&(pump)){
			motor_b_stop();
			pump = false;
			printf("Ground humidity over 80, stopped pump\n");
		}*/
		if((buf_pump_on) & (!pump)){//
			
			motor_b_run(40,40000);
			pump = true;
			printf("Started pump\n");
			
		}
		else if((buf_pump_off) & (pump)){//

			motor_b_stop();
			pump = false;
			printf("Stopped pump\n");
		}

		if(buf_cur_cl){//
			if(curtain==true){
				pthread_mutex_lock(&lock_motor);
				signal_motor = CLOSE_CUR;
				pthread_mutex_unlock(&lock_motor);

				pthread_cond_signal(&cond_cur);
				curtain = false;
				printf("Closed curtain\n");
			}
			
		}
		else if(buf_cur_op){//
			if(curtain==false){
				pthread_mutex_lock(&lock_motor);
				signal_motor = OPEN_CUR;
				pthread_mutex_unlock(&lock_motor);

				pthread_cond_signal(&cond_cur);
				curtain = true;
				printf("Opened curtain\n");
			}
		}

		if((buf_fan_on & !fan)){//
			fan_run();
			fan = true;
			printf("Started fan\n");
		}
		else if((buf_fan_off & fan)){//
			fan_stop();
			fan = false;
			printf("Stopped fan\n");
		}

		/*buf_cur_op = 0;
		buf_cur_cl = 0;
		buf_pump_on = 0;
		buf_pump_off = 0;
		buf_fan_on = 0;
		buf_fan_off = 0;*/

		printf("***************************\n");
	}

	return 0;
}
