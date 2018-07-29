#include "apriltag_demo.h"
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include "apriltag.h"
#include "image_u8.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "tag16h5.h"
#include "zarray.h"
#include "getopt.h"
#include <sys/time.h>

extern "C" april_res print_detection(double p[4][2], int size, double aprilsize);
apriltag_family_t *tf_16h5 = NULL;
apriltag_family_t *tf_25h9 = NULL;
apriltag_family_t *tf_36h11 = NULL;
apriltag_detector_t *td_16h5;
apriltag_detector_t *td_25h9;
apriltag_detector_t *td_36h11;

double tic();

april_res_all aprilTags_16h5(int width,int height,uint8_t* buf,int size ,double aprilsize)
{
	april_res_all april_result_all;
	static int ini=0;
	int hamm_hist[10]; 
	if(ini==0){
		ini++;	
		tf_16h5 = tag16h5_create();
		td_16h5 = apriltag_detector_create();
		td_16h5->nthreads = 4;
		//td_16h5->refine_pose = 1;
    		//td_16h5->refine_decode = 1;
		// std->quad_sigm = 
		apriltag_detector_add_family(td_16h5, tf_16h5);		  
	}
	//printf("*************16h5*************\n");
	memset(hamm_hist, 0, sizeof(hamm_hist));
	image_u8_t *im = image_u8_create_from_pnm(width,height, buf);	
	 
    zarray_t *detections = apriltag_detector_detect(td_16h5, im);
	april_result_all.num = 0;
	april_result_all.num = zarray_size(detections);
	for (int i = 0; i < 10; i++) {
		april_result_all.april_result[i].ID = -1;	
	}
    for (int i = 0; i < zarray_size(detections); i++) 
	{
		apriltag_detection_t *det;
		zarray_get(detections, i, &det);
		if(det->id==5){
			april_result_all.april_result[i] = print_detection(det->p, size ,aprilsize);
			april_result_all.april_result[i].ID = det->id;
			april_result_all.april_result[i].c[0] = det->c[0];
			april_result_all.april_result[i].c[1] = det->c[1];
			april_result_all.april_result[i].corner[0][0] = det->p[0][0];
			april_result_all.april_result[i].corner[0][1] = det->p[0][1];
			april_result_all.april_result[i].corner[1][0] = det->p[1][0];
			april_result_all.april_result[i].corner[1][1] = det->p[1][1];
			april_result_all.april_result[i].corner[2][0] = det->p[2][0];
			april_result_all.april_result[i].corner[2][1] = det->p[2][1];
			april_result_all.april_result[i].corner[3][0] = det->p[3][0];
			april_result_all.april_result[i].corner[3][1] = det->p[3][1];
		}
		else {
			april_result_all.num--;
		}

		hamm_hist[det->hamming]++;
    }
	//printf(" using time just apriltags %12.3f", timeprofile_total_utime(td_16h5->tp) / 1.0E3);
	//printf("\n");
    apriltag_detections_destroy(detections);
    image_u8_destroy(im);
	return april_result_all;
}
april_res_all aprilTags_25h9(int width, int height, uint8_t* buf, int size ,double aprilsize)
{
	april_res_all april_result_all;
	static int ini = 0;
	int hamm_hist[10];
	if (ini == 0) {
		ini++;
		tf_25h9 = tag25h9_create();
		td_25h9 = apriltag_detector_create();
		//td_25h9->nthreads = 4;
		apriltag_detector_add_family(td_25h9, tf_25h9);
	}
	memset(hamm_hist, 0, sizeof(hamm_hist));
	image_u8_t *im = image_u8_create_from_pnm(width,height, buf);
	zarray_t *detections = apriltag_detector_detect(td_25h9, im);
	april_result_all.num = 0;
	april_result_all.num = zarray_size(detections);
	for (int i = 0; i < 10; i++) {
		april_result_all.april_result[i].ID = -1;
	}
	for (int i = 0; i < zarray_size(detections); i++)
	{
		apriltag_detection_t *det;
		zarray_get(detections, i, &det);
		april_result_all.april_result[i] = print_detection(det->p, size, aprilsize);
		april_result_all.april_result[i].ID = det->id;
		april_result_all.april_result[i].c[0] = det->c[0];
		april_result_all.april_result[i].c[1] = det->c[1];
		april_result_all.april_result[i].corner[0][0] = det->p[0][0];
		april_result_all.april_result[i].corner[0][1] = det->p[0][1];
		april_result_all.april_result[i].corner[1][0] = det->p[1][0];
		april_result_all.april_result[i].corner[1][1] = det->p[1][1];
		april_result_all.april_result[i].corner[2][0] = det->p[2][0];
		april_result_all.april_result[i].corner[2][1] = det->p[2][1];
		april_result_all.april_result[i].corner[3][0] = det->p[3][0];
		april_result_all.april_result[i].corner[3][1] = det->p[3][1];
		hamm_hist[det->hamming]++;
	}
	apriltag_detections_destroy(detections);
	image_u8_destroy(im);
	return april_result_all;
}
april_res_all aprilTags_36h11(int width, int height, uint8_t* buf, int size ,double aprilsize)
{
	april_res_all april_result_all;
	static int ini = 0;
	int hamm_hist[10];
	if (ini == 0) {
		ini++;
		tf_36h11 = tag36h11_create();
		td_36h11 = apriltag_detector_create();
		td_36h11->nthreads = 4;
		//td_36h11->refine_pose = 0;
    		//td_36h11->refine_decode = 0;
		apriltag_detector_add_family(td_36h11, tf_36h11);
	}
	memset(hamm_hist, 0, sizeof(hamm_hist));
	image_u8_t *im = image_u8_create_from_pnm(width,height, buf);
	zarray_t *detections = apriltag_detector_detect(td_36h11, im);
	april_result_all.num = 0;
	april_result_all.num = zarray_size(detections);
	for (int i = 0; i < 10; i++) {
		april_result_all.april_result[i].ID = -1;
	}
	for (int i = 0; i < zarray_size(detections); i++)
	{
		apriltag_detection_t *det;
		zarray_get(detections, i, &det);
		april_result_all.april_result[i] = print_detection(det->p, size, aprilsize);
		april_result_all.april_result[i].ID = det->id;
		april_result_all.april_result[i].c[0] = det->c[0];
		april_result_all.april_result[i].c[1] = det->c[1];
		april_result_all.april_result[i].corner[0][0] = det->p[0][0];
		april_result_all.april_result[i].corner[0][1] = det->p[0][1];
		april_result_all.april_result[i].corner[1][0] = det->p[1][0];
		april_result_all.april_result[i].corner[1][1] = det->p[1][1];
		april_result_all.april_result[i].corner[2][0] = det->p[2][0];
		april_result_all.april_result[i].corner[2][1] = det->p[2][1];
		april_result_all.april_result[i].corner[3][0] = det->p[3][0];
		april_result_all.april_result[i].corner[3][1] = det->p[3][1];
		hamm_hist[det->hamming]++;
	}
	//printf(" using time just apriltags %12.3f", timeprofile_total_utime(td_36h11->tp) / 1.0E3);
	//printf("\n");
	apriltag_detections_destroy(detections);
	image_u8_destroy(im);
	return april_result_all;
}
