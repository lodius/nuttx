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

static bool start2_started;
int start2_pid;

int temp,humi,light,ghumi;

pthread_t thread_log_s;
pthread_cond_t cond_log = PTHREAD_COND_INITIALIZER;
pthread_mutex_t lock_log = PTHREAD_MUTEX_INITIALIZER;

pthread_t thread_sensor_s;

int signal_log = 0;

//Functions declaration

int gsensor_getdata(int gsensor);

//Private Functions

int gsensor_getdata(int gsensor){
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

void log_sdcard(int log_temp, int log_humi, int log_light, int log_ghumi){
	char inputdata[16];
	char cmd[256]= "******************************\n";
	/*int sd = open("/sdcard/data.txt",O_WRONLY);
	if(sd < 0){
		printf("Failed to open sd card\n");
		return;
	}
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

	write(sd,cmd,strlen(cmd));
	close(sd);
	*/
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

void *thread_log(void *arg)
{
	int buf_temp,buf_humi,buf_light,buf_ghumi;

	while(1){
		pthread_mutex_lock(&lock_log);
		while(!signal_log){
			pthread_cond_wait(&cond_log,&lock_log);
		}
		signal_log = 0;
		buf_temp = temp;
		buf_humi = humi;
		buf_light = light;
		buf_ghumi = ghumi;

		pthread_mutex_unlock(&lock_log);

		log_sdcard(buf_temp,buf_humi,buf_light,buf_ghumi);
		printf("Data logged\n");
	}
}

void *thread_sensor(void *arg)
{
	int gsensor,sht10,bh1750;
	struct timespec t1;
	struct timespec t2;

	int buf_temp,buf_humi,buf_light,buf_ghumi;

	uint8_t buf_2[2];

	gsensor = open("/dev/gsensor",O_RDONLY);
	sht10 	= open("/dev/sht10",O_RDWR);
	bh1750 	= open("/dev/light0",O_RDWR);
	
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

		buf_ghumi = gsensor_getdata(gsensor);
		buf_ghumi = (int)buf_ghumi*100/4095;
		printf("Ground humidity : %d\n",buf_ghumi);

		printf("***************************\n");
		
		pthread_mutex_lock(&lock_log);
		temp = buf_temp;
		humi = buf_humi;
		light = buf_light;
		ghumi = buf_ghumi;
		pthread_mutex_unlock(&lock_log);
	
    	//Check timeout whether to signal or not

		clock_gettime(CLOCK_REALTIME,&t2);
		if((t2.tv_sec-t1.tv_sec)>10){
			pthread_mutex_lock(&lock_log);
			signal_log = 1;
			pthread_mutex_unlock(&lock_log);

			pthread_cond_signal(&cond_log);
			printf("%d seconds passed\n",10);
			clock_gettime(CLOCK_REALTIME,&t1);
		}
    	
		printf("***************************\n");
	}
}

static int start2_daemon(int argc, char *argv[]){
	pthread_attr_t	attr_log;
	pthread_attr_t	attr_sensor;
	
	printf("Setup\n");
	signal_log = 0;
	


	attr_log.priority = 100;
	attr_sensor.priority = 110;

	pthread_attr_init(&attr_log);
	pthread_attr_init(&attr_sensor);

	if(pthread_attr_setschedpolicy(&attr_log,SCHED_RR)){
		printf("Set sched data failed\n");
	}

	if(pthread_attr_setschedpolicy(&attr_sensor,SCHED_RR)){
		printf("Set sched data failed\n");
	}

	printf("Starting thread data \n");
	if ( (pthread_create(&thread_log_s,&attr_log, (void*)thread_log, NULL)) != 0)
	{
		fprintf(stderr, "Error in thread data creation\n");
		return 0;
	}

	printf("Starting thread sensor \n");
	if ( (pthread_create(&thread_sensor_s,&attr_sensor, (void*)thread_sensor, NULL)) != 0)
	{
		fprintf(stderr, "Error in thread data creation\n");
		return 0;
	}

}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int start2_main(int argc, char *argv[])
#endif
{
	FAR char *start2argv[2];
    if(!start2_started){
      start2argv[0] = "start2_daemon";
      start2argv[1] = NULL;

      start2_pid = task_create("start2_daemon", CONFIG_MYAPPS_START2_PRIORITY,
                    CONFIG_MYAPPS_GSENSOR_STACKSIZE, start2_daemon, (FAR char * const *)start2argv);
      if(start2_pid < 0){
        int errcode = errno;
            fprintf(stderr, "ERROR: Failed to start start2_daemon: %d\n",errcode);
            return -errcode;
            printf("start2_daemon started");
      }
    }
    else{
      printf("start2_daemon already started");
    }
	

	return 0;
}
