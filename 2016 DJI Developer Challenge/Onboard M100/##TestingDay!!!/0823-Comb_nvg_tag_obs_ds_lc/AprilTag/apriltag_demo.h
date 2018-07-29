#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include "getcor.h"

extern "C" april_res_all aprilTags_16h5(int width, int height, uint8_t* buf, int size ,double aprilsize = 6);
extern "C" april_res_all aprilTags_25h9(int width, int height, uint8_t* buf, int size , double aprilsize = 6);
extern "C" april_res_all aprilTags_36h11(int width,int height,uint8_t* buf,int size , double aprilsize = 39);
