#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mount.h>
#include <sys/boardctl.h>


#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>


#include <nuttx/sensors/sht10.h>
#include <nuttx/analog/adc.h>


int ghumi,temp,humi,light;
int gsensor,sht10,esp8266,bh1750;

struct data_s{
  int value;
  char key[16];
};

int gsensor_read(){
  struct adc_msg_s sample[CONFIG_MYAPPS_GSENSOR_GROUPSIZE];
  size_t readsize;
  size_t nbytes;
  int am_channel,am_data,ret,i;

  ret = ioctl(gsensor, ANIOC_TRIGGER, 0);
  if (ret < 0)
  {
    printf("adc_main: ANIOC_TRIGGER ioctl failed: %d\n", errno);
  }

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
    else
    {
      printf("Sample:\n");
      for (i = 0; i < nsamples; i++)
      {
        am_channel = sample[i].am_channel;
        am_data = sample[i].am_data;

        am_data = (int)(100 - (am_data*100/4095));

        /*printf("%d: channel: %d value: %d\n",
          i+1,am_channel , am_data);

        if(am_data < 20){
          printf("Channel : %d too dry\n",am_channel);
        }else if(am_data > 80){
          printf("Channel : %d too wet\n",am_channel);
        }else{
          printf("Channel : %d is perfect\n",am_channel);
        }*/
      }
    }
  }
  return (100 - (sample[0].am_data+sample[1].am_data+sample[2].am_data+sample[3].am_data)*100/(4095*4));
}


#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int ssend_main(int argc, char *argv[])
#endif
{
  struct data_s data[4] = {
    {0,"temp"},
    {0,"humi"},
    {0,"light"},
    {0,"ghumi"}
  };
  uint8_t buf[2];
  sht10 = open("/dev/sht10",O_RDWR);
  esp8266 = open("/dev/ttyS1",O_RDWR);
  bh1750 = open("/dev/light0",O_RDWR);
  gsensor = open("/dev/gsensor",O_RDONLY);
  
  if(sht10 < 0 || esp8266 < 0 || bh1750 < 0 || gsensor < 0){
    printf("Failed to open one of the devices\n");
    return 0;
  }

  while(1){

    ioctl(sht10,SNIOC_START_MEASURE,0);
    do{
      ioctl(sht10,SNIOC_MEASURE,0);
      
    }while(!ioctl(sht10,SNIOC_FINISH,0));

    temp = ioctl(sht10,SNIOC_READT,0);
    printf("Temperature : %d\n",temp);

    humi = ioctl(sht10,SNIOC_READH,0);
    printf("Humidity : %d\n",humi);

    read(bh1750,&buf,2);
    printf("Light : %u lx\n",((buf[1]<<8) | (buf[0])) * 10/12  );

    ghumi = gsensor_read();
    printf("Ground humidity : %d pecentage\n",ghumi);

    temp = (int)temp/100;
    humi = (int)humi/100;
    light = (int)( ( (buf[1]<<8) | (buf[0]) ) * 10/12  );

    data[0].value = temp;
    data[1].value = humi;
    data[2].value = light;
    data[3].value = ghumi;

    stm32_esp8266send_multi(esp8266,data,4);
    //stm32_esp8266send_test(esp8266,temp);
    
  }

  close(sht10);
  close(esp8266);
  close(bh1750);
  return 0;
}
