#include <stdio.h>
#include <setjmp.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <ctype.h>
#include <iostream>
#include <opencv.hpp>
#include <sys/time.h>
#include <unistd.h>
#include <semaphore.h>

extern "C"
{
#include "apriltag_demo.h"
};

extern "C" int manifold_cam_init(int mode);
extern "C" int manifold_cam_read(unsigned char *buffer, unsigned int *nframe, unsigned int block);
extern "C" int manifold_cam_exit();

using namespace std;
using namespace cv;
//#define isvideo
//#define showvideo
#define coutdetail
//#define couttime
double frequence = 5;
int size = 1;
int family= 11;

#define FRAME_SIZE              (1280*720*3/2) 
april_res_all april_result_all;    
int detect_mode=1;           //mode=1:ËÑË÷AprilTags   mode=2£ºµÈŽý£¬Žó  mode=3£ºœµÂä£¬Žó  mode=4£ºœµÂä£¬Ð¡
extern sem_t main_april;
int aprilready;

char key1='a';
Mat showkey=Mat::zeros(33,33,CV_8UC1);

double tic();
double April_detect(unsigned char* pYUV, int size = 1,int family=11);

void *get_images_loop(void * data)
{
	int mode = 6;
	unsigned char buffer_X3[FRAME_SIZE+8] = {0};
	unsigned int nframe;
	int ret = manifold_cam_init(6);
	if (-1 == ret)
	{
		
		printf("manifold init error \n");
		return NULL;
	}
	sleep(8);
	while (!manifold_cam_exit()) 
	{	
		// << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% " << detect_mode << endl;
	
		switch (detect_mode) {  
		case 1: {size = 1; family = 9; frequence = 5;}break;
		case 2: {size = 1; family = 11; frequence = 25;}break;			//飞机悬停，等待检测到apriltags
		case 3: {size = 1; family = 11; frequence = 25;}break;			//飞机跟踪大的apriltags
		case 4: {size = 2; family = 11; frequence = 50; }break;			//飞机跟踪大的apriltags
		case 5: {size = 1; family = 5; frequence = 25;}break;			//飞机跟踪小的apriltags
		case 6: {size = 2; family = 5; frequence = 50; }break;			//飞机跟踪小的apriltags 
		default: {size = 1; family = 11; frequence = 5; cout << "mode error" << endl; }
		}
		if (detect_mode == 3 || detect_mode == 4 || detect_mode == 5 || detect_mode ==6 && key1 !='q') {
			sem_wait(&main_april);
		}
		ret = manifold_cam_read(buffer_X3, &nframe,0);
		if (ret >= 0)
		{
			double use_time = April_detect(buffer_X3, size, family);
			aprilready = 1;
			double T_time = (double)1.0 / frequence;	
			if (detect_mode == 1|| detect_mode == 2) {
				if (use_time > T_time) {
					cout << "using time error"
						<<"mode :"<<detect_mode<<"||"
						<<"frequence :"<<frequence<<"||"
						<<"time :"<<use_time<<"||"
						<<endl;	
				}
				else usleep((T_time - use_time) * 1000000);
			}
		}
		else { 
			usleep(100000);
			//cout << "cann't get frame, ret = " <<ret<< endl;
		}
	}
	printf("get_images_loop thread exit! \n");
	return NULL;
}
double April_detect(unsigned char* pYUV,int size, int family){  //£šsize=640*360£©  320*180ÁœÖÖ
	static int ini = 0;
	                                        //family=5,9,11¶ÔÓŠ16h5,25h9,£š36h11£©  
	static int width = 1280, height = 720;			//tag16h5 6_6_7  tag25h9 6_6_5  tag36h11 39_39_1
	int width_use, height_use;
	static double old_time = 0, time = 0,use_time = 0;
	static double time1, time2, time3;
	old_time = tic();
	Mat gray;
	Mat yuv(height + height / 2, width, CV_8UC1, pYUV);
	cvtColor(yuv, gray, CV_YUV2GRAY_NV12);
	if (detect_mode >= 2 && ini == 1) {
		ini=0;
		resize(gray, gray, Size(width / 4, height / 4));
		aprilTags_16h5(gray.cols, gray.rows, gray.data, size, 6.0);
		//aprilTags_25h9(gray.cols, gray.rows, gray.data, size, 6.0);
		aprilTags_36h11(gray.cols, gray.rows, gray.data, size, 39.3);
		cout<<"apriltags initialize success";
	}
	time1 = tic();
	if (size == 1) {
		resize(gray, gray, Size(width / 2, height / 2));
		if (family == 5) {
			april_result_all = aprilTags_16h5(gray.cols, gray.rows, gray.data, size , 6.0);
		}else if (family == 9) {
			april_result_all = aprilTags_25h9(gray.cols, gray.rows, gray.data, size , 6.0);
		}else if (family == 11) {
			april_result_all = aprilTags_36h11(gray.cols, gray.rows, gray.data, size , 39.3);
		}
	}else if(size == 2){
		resize(gray, gray, Size(width / 4, height / 4));
		if (family == 5) {
			april_result_all = aprilTags_16h5(gray.cols, gray.rows, gray.data, size , 6.0);
		}
		else if (family == 9) {
			april_result_all = aprilTags_25h9(gray.cols, gray.rows, gray.data, size , 6.0);
		}
		else if (family == 11) {
			april_result_all = aprilTags_36h11(gray.cols, gray.rows, gray.data, size , 39.3);
		}
	}
	time2 = tic();
#ifdef coutdetail
	for(int i = 0; i < april_result_all.num; i++ ){
		cout << april_result_all.april_result[i].ID << "||"
		<< april_result_all.april_result[i].x << "||"
		<< april_result_all.april_result[i].y << "||"
		<< april_result_all.april_result[i].z << "||"
		<< april_result_all.april_result[i].yaw << "||"
		<< april_result_all.april_result[i].pitch << "||"
		<< april_result_all.april_result[i].roll << "||"
		<< april_result_all.april_result[i].c[1] << "||"
		<< april_result_all.april_result[i].c[0] << endl;
	}
#endif 
	time3 = tic();

#ifdef isvideo
	static VideoWriter DDwriter("../camera.avi", CV_FOURCC('D', 'I', 'V', 'X'), 15, Size(width / 2, height / 2), false);
	for(int i = 0; i < april_result_all.num; i++ ){
		circle(gray, Point(april_result_all.april_result[i].c[0], april_result_all.april_result[i].c[1]), 20, Scalar(0, 0, 255), 3);
	}
	DDwriter.write(gray);
#endif 

#ifdef showvideo
	namedWindow("yuvtogray", WINDOW_AUTOSIZE);
	imshow("yuvtogray", gray);
	key1 =waitKey(1);
	//cout<<"key"<<key<<endl;
#else

	imshow("yuvtogray", showkey);
	key1 = waitKey(1);

#endif 
	
	time = tic();
	use_time = time - old_time;

#ifdef couttime
	printf("using time:%f \n", use_time);
	printf("using time1:%f \n", time1 - old_time);
	printf("using time2:%f \n", time2 - old_time);
	printf("using time3:%f \n", time3 - old_time);

#endif 

	return use_time;
}
