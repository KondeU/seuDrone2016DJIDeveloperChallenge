#include <stdio.h>
#include <iostream>
#include <setjmp.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <getopt.h>
#include <pthread.h>
#include <errno.h>
#include <ctype.h>
#include <fstream>
#include <iomanip>
#include "DJIHardDriverManifold.h"
#include "math.h"
#include "conboardsdktask.h"
#include "APIThread.h"
#include <sys/time.h>
#include <semaphore.h>
using namespace std;

//#define flightmove

//extern april_res_all april_result_all;
sem_t main_april;

int iDataFromMobile;
void GetDataFromMobile(DJI::onboardSDK::CoreAPI * API, Header * pHeader, void * pData);

void SearchInArea(/*const */ConboardSDKScript * pScript);

void * get_images_loop(void * data);
void * main_guidance(void * data);
void land_in_car(ConboardSDKScript* script, double height);
double tic() {
	struct timeval t;
	gettimeofday(&t, NULL);
	return ((double)t.tv_sec + ((double)t.tv_usec) / 1000000.);
}

HardDriverManifold driver("/dev/ttyTHS1", 230400);
CoreAPI api(&driver);
ConboardSDKScript sdkScript(&api);
//BroadcastData bd;
ScriptThread st(&sdkScript);

int main(int argc, char **argv)
{
    pthread_attr_t attr;
    struct sched_param schedparam;
    pthread_t read_thread;
    pthread_t guidance_thread;

    if(0 != geteuid())
    {
        printf("Please run ./test as root!\n");
        return -1;
    }

    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy((pthread_attr_t *)&attr, SCHED_FIFO);
    schedparam.sched_priority = 90;
    pthread_attr_setschedparam(&attr,&schedparam);
    pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);


    if (pthread_create(&read_thread, &attr, get_images_loop, NULL) != 0)
    {
        perror ("usbRead_thread create");
        assert(0);
    }
    if (pthread_create(&guidance_thread, NULL, main_guidance, NULL) != 0)       ////////////////
    {
        perror ("guidance_thread create");
        assert(0);
    }
    if(pthread_attr_destroy(&attr) != 0)
    {
        perror("pthread_attr_destroy error");
    }	

	driver.init();

	APIThread send(&api, 1);
	APIThread read(&api, 2);
	send.createThread();
	read.createThread();	
	loadSS(st.script, (UserData)"--SS load ./key.txt");
	sleep(1);
	st.script->getApi()->activate(&st.script->adata);
	sleep(1);
	st.script->getApi()->setControl(true);
	sleep(1);  


	iDataFromMobile = 0;
	uint8_t * recdata = NULL;
	CallBackHandler FromMobileEntrance;
	FromMobileEntrance.callback = GetDataFromMobile;
	FromMobileEntrance.userData = recdata;
	api.setFromMobileCallback(FromMobileEntrance);

	while(iDataFromMobile != 1)
	{
		cout << "Waiting for take off..." << endl;
		sleep(1);
	}
	
#ifdef flightmove
	int flag;
	flag = 4;
	st.script->getFlight()->task((DJI::onboardSDK::Flight::TASK)flag);
	sleep(8);
#endif 

	SearchInArea(st.script);
	sleep(3);

	//land_in_car(st.script,6.0);

#ifdef flightmove
	st.script->getApi()->setControl(true);
	sleep(1);
	flag = 6;
	st.script->getFlight()->task((DJI::onboardSDK::Flight::TASK)flag);
	sleep(5);
#endif 
	st.script->getApi()->setControl(false);
	sem_post(&main_april);
	kill(getpid(),SIGINT);

   	pthread_join(read_thread, NULL);/*wait for read_thread exit*/
	pthread_join(guidance_thread, NULL);
	
    return 0;
}
