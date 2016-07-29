#include <nuttx/config.h>

#include <stdio.h>
#include <pthread.h>

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <nuttx/timers/rtc.h>

#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#include <apps/netutils/cJSON.h>




static bool test_started;

static pthread_t threadaction;
static pthread_t threadaction2;
static pthread_t threadmain;
static pthread_mutex_t lock1;
static pthread_mutex_t lock2;
static pthread_cond_t cond1 = PTHREAD_COND_INITIALIZER;
static pthread_cond_t cond2 = PTHREAD_COND_INITIALIZER;
static int cond_signal = 0;
static int cond_signal2 = 0;
static int thread_timeout = 5;


void *thread_action(void *arg) { 
  int pid = getpid();
  struct sched_param param;
  param.sched_priority = 100;
  if (sched_setscheduler(pid, SCHED_RR, & param) != 0) {
    perror("sched_setscheduler");
    exit(EXIT_FAILURE);  
  }
  while(1){
    pthread_mutex_lock(&lock1);
    while (!cond_signal) {
      pthread_cond_wait(&cond1, &lock1);
    }
    cond_signal = 0;
    pthread_mutex_unlock(&lock1);
    printf("Thread action %d pid %d signaled\n",(int)arg,pid);
  }

  pthread_exit(NULL);
}

void *thread_action2(void *arg) { 
  int pid = getpid();
  struct sched_param param;
  param.sched_priority = 100;
  if (sched_setscheduler(pid, SCHED_RR, & param) != 0) {
    perror("sched_setscheduler");
    exit(EXIT_FAILURE);  
  }
  while(1){
    /*pthread_mutex_lock(&lock2);
    while ((cond_signal2)!=3) {
      pthread_cond_wait(&cond2, &lock2);
    }
    cond_signal2 = 0;
    pthread_mutex_unlock(&lock2);*/
    printf("Thread action %d pid: %d signaled\n",(int)arg,pid);
    sleep(2);
  }

  pthread_exit(NULL);
}

void *thread_main2(void *arg){
  struct timespec t1;
  struct timespec t2;
  int pid = getpid();
  printf("Main thread pid: %d\n",pid);

  printf("Setup\n");

  struct sched_param param;
  param.sched_priority = 255;
  if (sched_setscheduler(pid, SCHED_RR, & param) != 0) {
    perror("sched_setscheduler");
    exit(EXIT_FAILURE);  
  }

  clock_gettime(CLOCK_REALTIME,&t1);
  clock_gettime(CLOCK_REALTIME,&t2);

  while(1){
    sleep(2);
    printf("Sensor data read\n");
    pthread_mutex_lock(&lock2);
    cond_signal2++;
    pthread_mutex_unlock(&lock2);

    clock_gettime(CLOCK_REALTIME,&t2);
    if((t2.tv_sec-t1.tv_sec)>thread_timeout){
      pthread_mutex_lock(&lock1);
      cond_signal = 1;
      pthread_mutex_unlock(&lock1);
      pthread_cond_signal(&cond1);
      printf("%d seconds passed\n",thread_timeout);
      clock_gettime(CLOCK_REALTIME,&t1);
    }

    if(cond_signal2==3){
      pthread_mutex_lock(&lock2);
      cond_signal2 = 0;
      pthread_mutex_unlock(&lock2);
      pthread_cond_signal(&cond2);
      printf("Condition 2 was met\n");
    }

  }
}

static int pthread_test(int argc, char *argv[])
{
  pthread_mutex_init(&lock1, NULL);
  pthread_mutex_init(&lock2, NULL);

  printf("Starting thread main\n");
  
  if ( (pthread_create(&threadmain, NULL, (void*)thread_main2, NULL)) != 0)
  {
    fprintf(stderr, "Error in thread main creation\n");
  }

  printf("Starting thread action 1\n");
  if ( (pthread_create(&threadaction, NULL, (void*)thread_action, (void*) 1)) != 0)
  {
    fprintf(stderr, "Error in thread 1 creation\n");
  }

  printf("Starting thread action \n");
  if ( (pthread_create(&threadaction2, NULL, (void*)thread_action2, (void*) 2)) != 0)
  {
    fprintf(stderr, "Error in thread 2 creation\n");
  }

  printf("Waiting for threads\n");
  
  pthread_join(threadaction, NULL);
  pthread_join(threadaction2, NULL);
  pthread_join(threadmain, NULL);

  printf("\nAll threads stopped\n");
  return 0;
}

int match(const char *str, const char *sub);

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int test_main(int argc, char *argv[])
#endif
{
  /*FAR char *testargv[2];
  cond_signal = 0;

  if(argc == 2){
    thread_timeout = strtoul(argv[1],NULL,10);
    printf("Sleep for %d seconds\n",thread_timeout);
  }
  else
    printf("Sleep for 3 seconds default\n");

  if(!test_started){
    testargv[0] = "pthread_test";
    testargv[1] = NULL;

    int test_pid = task_create("pthread_test", 100, 2048, pthread_test, (FAR char * const *)testargv);
    if(test_pid < 0){
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to start test process: %d\n",errcode);
      return -errcode;
      printf("Test task started\n");
    }

  }
*/

    /*char buf[128];
    int esp = open("/dev/ttyS1",O_RDWR) ;
    stm32_esp8266send_test2(esp,2,69,96,69,96,1,1,0,buf);
    printf("\n*****\n");
    printf("%s\n",buf);

    if(match(buf,"!"))
      printf("motor on\n");
    else if(match(buf,"@"))
      printf("motor off\n");

    if(match(buf,"#"))
      printf("pump on\n");
    else if(match(buf,"$"))
      printf("pump off\n");

    if(match(buf,"%"))
      printf("fan on\n");
    else if(match(buf,"^"))
      printf("fan off\n");
*/

    config_fan();
    fan_on();
    sleep(5);
    fan_off();
    
    return 0;
  }

