#include <nuttx/config.h>

#include <stdio.h>
#include <pthread.h>

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

static bool test_started;

static pthread_t thread1;
static pthread_t thread2;
static pthread_mutex_t lock;
static pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
static int cont = 0;
static int thread_timeout = 3;

void *thread_1(void *arg) {
  /* thread code blocks here until MAX_COUNT is reached */
 
  while(1){
    pthread_mutex_lock(&lock);
    while (!cont) {
      pthread_cond_wait(&cond, &lock);
    }
    cont = 0;
    pthread_mutex_unlock(&lock);
    printf("Thread 1 signaled\n");
  }
  /* proceed with thread execution */

  pthread_exit(NULL);
}

/* some other thread code that signals a waiting thread that the condition has been reached */
void *thread_2(void *arg) {
  while(1){
    pthread_mutex_lock(&lock);

    sleep(thread_timeout);

    cont = 1;
    pthread_mutex_unlock(&lock);
    pthread_cond_signal(&cond);
  }

  pthread_exit(NULL);
} 

static int pthread_test(int argc, char *argv[])
{
  pthread_mutex_init(&lock, NULL);

  printf("Starting thread 1\n");
  
  if ( (pthread_create(&thread1, NULL, (void*)thread_1, (void*) 1)) != 0)
  {
    fprintf(stderr, "Error in thread 1 creation\n");
  }

  printf("Starting thread 2\n");
  if ( (pthread_create(&thread2, NULL, (void*)thread_2, (void*) 2)) != 0)
  {
    fprintf(stderr, "Error in thread 2 creation\n");
  }

  printf("Waiting for threads\n");
  
  pthread_join(thread1, NULL);
  pthread_join(thread2, NULL);

  printf("All threads stopped\n");
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int test_main(int argc, char *argv[])
#endif
{
  FAR char *testargv[2];
  cont = 0;

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
      printf("hello task started");
    }

  }
  

  return 0;
}

