#include <signal.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <unistd.h>
#include "DJI_guidance.h"
#include "DJI_utility.h"
#include "opencv2/core/core.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace std;
using namespace cv;

#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)
#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; return NULL;}}

#define GoingForward 0
#define GoingUp 1
#define StayStill 2
#define BreakExit 3
#define TurnRight 4
#define GoAfterRight 5
#define TurnLeft 6
#define GoAfterLeft 7
#define TurnBack 16

#define F 1
#define R 2
#define U 3
#define L 4
#define D 0

//
#define DISKEEPGO 1.5
#define TIMEKEEPTURN 1

#define MAXHEIGHT 2
#define MINHEIGHT 1
#define CHANSPEDIS 4.5 //distance to change speed
#define SAFEDISF 2.5
#define SAFEDIS 2

//
#define LOWSPEED 0.7
#define HIGHSPEED 2

#define AREA 140
#define ERODETIMES 2
#define DILATETIMES 6
#define MaxSampleTimes 10

#define GDC_FORWARD 0
#define GDC_LEFT 1
#define GDC_RIGHT 2

const char *statetext[22] = { "Going Forward", "Going Up", "Stay Still", "Break and exit", "Turning Right", "GoAfterRight", "Turning Left", "GoAfterLeft"};
const char *movestate = "Init";

char key = 0;
float yaw;
float yawangle;
float SafeDis[CAMERA_PAIR_NUM];
float TempDis[CAMERA_PAIR_NUM];
float MinDis[CAMERA_PAIR_NUM];
bool depclose[CAMERA_PAIR_NUM];
bool firstcall = true;
bool EnableControl = false;
bool testhomeward = false;
bool isUp = false;
int state = 0;
int SampleTimes = 0;
int globalframeindex = 0;
double timeTimeout;
double DisplayTimeout;
double displaytimeoutRef = 0;
String gs;
String ex;

Mat grayL[CAMERA_PAIR_NUM];
Mat grayR[CAMERA_PAIR_NUM];
Mat depth[CAMERA_PAIR_NUM];//(HEIGHT,WIDTH,CV_16SC1);
Mat disparity[CAMERA_PAIR_NUM];//(HEIGHT,WIDTH,CV_16SC1);

timeval time_now;
timeval time_old;
double time_dura;

unsigned int frame_index;        // corresponse frame index 
unsigned int time_stamp;         // time stamp of corresponse image captured in ms 
float acc_x;                   // acceleration of x in unit of m/s^2
float acc_y;                   // acceleration of y in unit of m/s^2
float acc_z;                   // acceleration of z in unit of m/s^2
float q[4];                    // quaternion: [w,x,y,z]
float vx;                 // velocity of x in mm/s
float vy;                 // velocity of y in mm/s 
float vz;                 // velocity of z in mm/s
float movespeed = 0;
float globaldistance[CAMERA_PAIR_NUM];		// distance of obstacle in cm
float globalultrasonic[CAMERA_PAIR_NUM];	// distance in mm. -1 means invalid measurement. 
unsigned short reliability[CAMERA_PAIR_NUM];	// reliability of the distance data 
float position_in_global_x;
float position_in_global_y;
float position_in_global_z;

e_vbus_index channel = e_vbus1;
DJI_lock    g_lock;
DJI_event   g_event;


#include "tc.h"
extern TThreadController ttc;
#include "../DJIlib/conboardSDK/conboardsdktask.h"
extern BroadcastData bd;
extern ScriptThread st;
static ofstream fout("obstacle.log");

//#define DEBUG 1
#define DEBUG_DOCU 1

void GoUp()
{
	movestate = "moving up";
	if ( EnableControl) return;
	FlightData fd;
	fd.flag = 0x43;
	fd.x = 0;
	fd.y = 0;
	fd.z = 0.5;	
	fd.yaw = yaw;
	st.script->getFlight()->setFlight(&fd);
}
void GoDown()
{
	movestate = "moving down";
	if ( EnableControl) return;
	FlightData fd;
	fd.flag = 0x43;
	fd.x = 0;
	fd.y = 0;
	fd.z = -0.5;
	fd.yaw = yaw;
	st.script->getFlight()->setFlight(&fd);
}
void GoForward()
{
	movestate = "moving forward";
	if ( EnableControl) return;
	FlightData fd;
	fd.flag = 0x53;
	fd.x = ttc.fNvgSpeed;
	fd.y = 0;
	fd.z = ttc.fFlyHeight;
	fd.yaw = yaw;
	st.script->getFlight()->setFlight(&fd);
}
void GoBack()
{
	movestate = "moving back";
	if ( EnableControl) return;
	FlightData fd;
	fd.flag = 0x53;
	fd.x = -ttc.fNvgSpeed;
	fd.y = 0;
	fd.z = ttc.fFlyHeight;
	fd.yaw = yaw;
	st.script->getFlight()->setFlight(&fd);
}
void GoLeft()
{
	movestate = "moving left";
	if ( EnableControl) return;
	FlightData fd;
	fd.flag = 0x53;
	fd.x = 0;
	fd.y = -ttc.fNvgSpeed;
	fd.z = ttc.fFlyHeight;
	fd.yaw = yaw;
	st.script->getFlight()->setFlight(&fd);
}
void GoRight()
{
	movestate = "moving right";
	if ( EnableControl) return;
	FlightData fd;
	fd.flag = 0x53;
	fd.x = 0;
	fd.y = ttc.fNvgSpeed;
	fd.z = ttc.fFlyHeight;
	fd.yaw = yaw;
	st.script->getFlight()->setFlight(&fd);
}
void Turn(float yawangle)
{
	movestate = "turning";
	if ( EnableControl) return;
	FlightData fd;

	fd.flag = 0x53;
	fd.x = 0;
	fd.y = 0;
	fd.z = ttc.fFlyHeight;
	if (yaw > 180) yaw -= 360;
	else if (yaw < -180) yaw += 360;
	fd.yaw = yaw;
	st.script->getFlight()->setFlight(&fd);
}
timeval tic_guidance()
{
	struct timeval Time;
	gettimeofday(&Time,NULL);
	return Time;
}
void update_height()
{	
	bd = st.script->getApi()->getBroadcastData();
	ttc.fFlyHeight = bd.pos.height;
	
	fout<<"bdheight:"<<ttc.fFlyHeight<<endl;
	cout<<"bdheight:"<<ttc.fFlyHeight<<endl;
	fout<<"MinDis[D]:"<<MinDis[D]<<endl;
	cout<<"MinDis[D]:"<<MinDis[D]<<endl;
}
float height()
{
	return ttc.fFlyHeight > MinDis[D] ? ttc.fFlyHeight : MinDis[D];
}

void x3_direction()
{
	if(MinDis[L]<SafeDis[L]) ttc.bUltrasonicLeft = true;
	else ttc.bUltrasonicLeft = false;
	if(MinDis[R]<SafeDis[R]) ttc.bUltrasonicRight = true;
	else ttc.bUltrasonicRight = false;
}
float yawGap(float yaw1,float yaw2)
{
	if(yaw1 > 180 ) yaw1 -= 360;
	if(yaw1 < -180 ) yaw1 += 360;
	if(yaw2 > 180 ) yaw2 -= 360;
	if(yaw2 < -180 ) yaw2 += 360;

	float y = fabs(yaw1 - yaw2);
	if (y > 180) y = 360 - y;
	
	return y;
}
bool IsYawEqualYawangle()
{
	if(yawGap(yaw,yawangle) <1) return true;
	else return false;
}
bool IsYawSmallYawangle()
{
	if(yawangle > 180 ) yawangle -= 360;
	if(yawangle < -180 ) yawangle += 360;
	if(yaw > 180 ) yaw -= 360;
	if(yaw < -180 ) yaw += 360;
	
	float y = yaw - yawangle;
	if (y < -180) y += 360;
	else if (y > 180) y -= 360;
	if (y < 0) return true;
	else return false;
}
float yawAdd(float yaw1, float number)
{
	yaw1 = yaw1 + number;
	if(yaw1 > 180 ) yaw1 -= 360;
	if(yaw1 < -180 ) yaw1 += 360;

	return yaw1;
}
void TurnToYaw()
{
	if (IsYawSmallYawangle()) Turn(++yaw);
	else Turn(--yaw);
}
bool IsFinish()
{
	if( yawGap(yawangle,ttc.fYawAngle) <= 45 ) return true;
	else return false;
}
int close(int dir)
{
	if (MinDis[dir] < SafeDis[dir]) 
	{
		return 1;
	}
	else
	{
		if (depclose[dir])
		{
			if (dir == U) return 0;
			else return 2;
		}
		else 
		{
			return 0;
		}
	}
}
void SelectChannel(int dir)
{
	channel = (e_vbus_index)dir;
	destroyAllWindows();
	stop_transfer();
	reset_config();
	select_obstacle_distance();
	select_ultrasonic();
	select_greyscale_image( (e_vbus_index)F , true);
	select_depth_image( (e_vbus_index)F );
	select_greyscale_image(channel, true);
	select_depth_image(channel);
	start_transfer();
}
bool ChannelChanged(int dir)
{
	if (dir != channel)
	{
		SelectChannel(dir);	
		return true;
	}
	else
	{
		return false;
	}
}
void navigation()
{
	update_height();
	movestate = "no movement";
	x3_direction();
	
	fout<<"yaw:"<<yaw<<endl;
	fout<<"yawangle"<<yawangle<<endl;
	fout<<"ttc,yawangle"<<ttc.fYawAngle<<endl;
	fout<<"firstcall "<<firstcall<<endl;
	
	switch (state)
	{
	case GoingForward:
		{
			yawangle = ttc.fYawAngle;
			yaw = yawangle;
		
			cout << "GoingForward........." << endl;
			fout << "GoingForward........." << endl;

			if( ttc.iGuidance == GDC_FORWARD )
			{
				if (MinDis[D] < 1 /*MINHEIGHT*/ && MinDis[U] > 2)
				{
					 
					ttc.iRequestControl = 1;
					if(ttc.iCrntCtrl == CTRL_OBSTACLE && MinDis[D] < 1){ GoUp(); isUp = true;fout << "Go up." << endl;}
				}
				else if ( MinDis[D] < 1.5 && MinDis[D] > 1 && isUp == true/*MINHEIGHT*/ && MinDis[U] > 2)
				{
					fout << "Go up." << endl;
					ttc.iRequestControl = 1;
					if(ttc.iCrntCtrl == CTRL_OBSTACLE && MinDis[D] < 1) GoUp();
				}
				else if (MinDis[D] > MAXHEIGHT)
				{
					
					ttc.iRequestControl = 1;
					if(ttc.iCrntCtrl == CTRL_OBSTACLE){fout << "Go down." << endl; GoDown();}
				}
				else
				{
					isUp = false;
					if(close(F))
					{
						ttc.iRequestControl = 1;
						

						if (ChannelChanged(R)) return;
						if(close(R)) 
						{
							if(ttc.iCrntCtrl == CTRL_OBSTACLE)
								state = TurnLeft;
						}
						else 
						{
							if(ttc.iCrntCtrl == CTRL_OBSTACLE)
								state = TurnRight;				
						}

					}
					else
					{
						ttc.iRequestControl = 0;
					}
				}
			}
			if( ttc.iGuidance == GDC_LEFT )//right
			{
				fout<<"unsafe"<<ttc.bBTLUnsafe<<endl;
				if (ChannelChanged(L)) return;
				if(close(L)) 
				{
					ttc.bBTLUnsafe = true;//left && back
					//ttc.iRequestControl = 1;
				}
				else ttc.bBTLUnsafe = false;//back

				if(close(F)) //forward
				{
					ttc.iRequestControl = 1;
					if( ttc.iCrntCtrl == CTRL_OBSTACLE ) state = TurnRight;
				}
				else
				{
					ttc.iRequestControl = 0;
				}
			}
			if(ttc.iGuidance == GDC_RIGHT)//left
			{
			fout<<"unsafe"<<ttc.bBTLUnsafe<<endl;
				if (ChannelChanged(R)) return;
				if(close(R))
				{
					ttc.bBTLUnsafe = true;
					//ttc.iRequestControl = 1;
				}
				else ttc.bBTLUnsafe = false;

				if(close(F)) 
				{
					ttc.iRequestControl = 1;
					if( ttc.iCrntCtrl == CTRL_OBSTACLE ) state = TurnLeft;
				}
				else
				{
					ttc.iRequestControl = 0;
				}
			}
			break;
		}
	case TurnRight:
		{
			cout << "TurnRight........." << endl;
			fout << "TurnRight........." << endl;
			
			if (close(F)) 
			{
				Turn(++yaw);
			}
			else
			{
				//if( yawGap(yaw,yawangle) < 38) yawangle = yawAdd( yawangle,48 );
				//else yawangle = yawAdd(yaw,10);
				//state = GoAfterRight;
				yawangle = yawAdd(yaw,10);
				state = GoAfterRight;
			}
			break;
		}
	case GoAfterRight:
		{
			cout << " GoAfterRight........" << endl;
			fout << " GoAfterRight........" << endl;

			if ( !IsYawEqualYawangle() ) TurnToYaw();
			else if(close(F))
			{
				state = TurnRight;
			}
			else if (ChannelChanged(L)) return;
			else if(close(L)) 
			{
				GoForward();
				firstcall = true;
			}
			else
			{
				if(firstcall)
				{
					firstcall = false;
					time_now = tic_guidance();
					time_old = tic_guidance();
					time_dura = 0;
				}	
				time_now = tic_guidance();
				time_dura = (time_now.tv_sec - time_old.tv_sec)*1000 + (time_now.tv_usec - time_old.tv_usec) / 1000;
				fout<<"time_now "<<time_now.tv_sec*1000<<endl;
				fout<<"time_old "<<time_old.tv_sec*1000<<endl;
				fout<<"time_dura:"<<time_dura<<"||"<<"timekeepgo"<<DISKEEPGO/ttc.fNvgSpeed*1000<<endl;
				if(time_dura < DISKEEPGO/ttc.fNvgSpeed*1000) GoForward();
				else
				{
					if(IsFinish())
					{
						if (!IsYawEqualYawangle()) TurnToYaw();//
						else 
						{
							state = GoingForward;
							ttc.iPian = PianRight;
						}
					}
					else
					{
						yawangle = yaw - 90;
						firstcall = true;
					}
				}
			}
			break;
		}
	case TurnLeft:
		{
			cout << " TurnLeft........" << endl;
			fout << " TurnLeft........" << endl;
			
			if (close(F))
			{
				Turn(--yaw);
			}
			else
			{
				//if( yawGap(yaw,yawangle) < 38) yawAdd( yawangle,-48 );
				//else yawangle = yawAdd(yaw,-10);
				//state = GoAfterLeft;
				yawangle = yawAdd(yaw,-10);
				state = GoAfterLeft;
			}
			break;
		}
	case GoAfterLeft:
		{
			cout << " GoAfterLeft........" << endl;
			fout << " GoAfterLeft........" << endl;

			if ( !IsYawEqualYawangle() ) TurnToYaw();
			else if(close(F))
			{
				state = TurnLeft;
			}
			else if (ChannelChanged(R)) return;
			else if(close(R)) 
			{
				GoForward();
				firstcall = true;
			}
			else
			{
				if(firstcall)
				{
					firstcall = false;
					time_now = tic_guidance();
					time_old = tic_guidance();
					time_dura = 0;
				}	
				time_now = tic_guidance();
				time_dura = (time_now.tv_sec - time_old.tv_sec)*1000 + (time_now.tv_usec - time_old.tv_usec) / 1000;	
				fout<<"time_now"<<time_now.tv_sec*1000<<endl;
				fout<<"time_old"<<time_old.tv_sec*1000<<endl;
				fout<<"time_dura:"<<time_dura<<"||"<<"timekeepgo"<<DISKEEPGO/ttc.fNvgSpeed*1000<<endl;
				if(time_dura < DISKEEPGO/ttc.fNvgSpeed*1000) GoForward();
				else
				{
					if(IsFinish())
					{
						if (!IsYawEqualYawangle()) TurnToYaw();//
						else 
						{
							state = GoingForward;
							ttc.iPian = PianLeft;
						}
					}
					else
					{
						yawangle = yaw + 90;
						firstcall = true;
					}
				}
			}
			break;
		}
	case StayStill: break;
	default:
		{
			ex += "\nUnexpect default in navigation()"; 
			state = StayStill;
			break;
		}
	}
}

void homeward()
{
	update_height();
	movestate = "homeward";
	if (ttc.fFlyHeight > 15)
	{
		state = BreakExit;
		ttc.iRequestControl = -1;
	}

	cout<<"homeward"<<endl;
	fout<<"homeward"<<endl;
	fout<<"yaw:"<<yaw<<endl;
	fout<<"yawangle:"<<yawangle<<endl;
	switch (state)
    	{
		case GoingUp:
			cout<<"GoingUp........."<<endl;
			fout<<"GoingUp........."<<endl;

			if (ChannelChanged(U)) return;
			if (!close(U)) GoUp();
			else state = GoingForward;
			break;
		case GoingForward:
			cout << "GoingForward........." << endl;
			fout << "GoingForward........." << endl;

			if (ChannelChanged(U)) return;
			if (!close(U)) 
			{
				GoUp();
				state = GoingUp;
			}

			if (MinDis[D] < 1 /*MINHEIGHT*/ && MinDis[U] > 2)
			{
				fout << "Go up." << endl; 
				if( MinDis[D] < 1 ){ GoUp(); isUp = true;}
			}
			else if ( MinDis[D] < 1.5 && MinDis[D] > 1 && isUp == true && MinDis[U] > 2)
			{
				fout << "Go up." << endl;
				if( MinDis[D] < 1) GoUp();
			}
			else if (MinDis[D] > MAXHEIGHT)
			{
				fout << "Go down." << endl;
				GoDown();
			}
			else
			{
				isUp = false;
				if(close(F))
				{
					if (ChannelChanged(R)) return;
					if(close(R)) 
					{
						state = TurnLeft;
					}
					else 
					{
						state = TurnRight;				
					}

				}
				else GoForward();
			}
			break;
		case TurnRight:
			cout << "TurnRight........." << endl;
			fout << "TurnRight........." << endl;

			if (close(F)) 
			{
				Turn(++yaw);
			}
			else 
			{
				yawangle = yawAdd(yaw,10);
				state = GoAfterRight;
			}
			break;
		case GoAfterRight:
			cout << " GoAfterRight........" << endl;
			fout << " GoAfterRight........" << endl;

			if (ChannelChanged(U)) return;
			if (!close(U))
			{
				GoUp();
				state = GoingUp;
			}

			if (ChannelChanged(L)) return;

			if ( !IsYawEqualYawangle() ) TurnToYaw();
			else if(close(F))
			{
				state = TurnRight;
			}
			else if(close(L)) 
			{
				GoForward();
				firstcall = true;
			}
			else
			{
				if(firstcall)
				{
					firstcall = false;
					time_now = tic_guidance();
					time_old = tic_guidance();
					time_dura = 0;
				}	
				time_now = tic_guidance();
				time_dura = (time_now.tv_sec - time_old.tv_sec)*1000 + (time_now.tv_usec - time_old.tv_usec) / 1000;			
				fout<<"time_dura"<<time_dura<<endl;
				if(time_dura < DISKEEPGO/ttc.fNvgSpeed*1000) GoForward();
				else
				{
					if(IsFinish())
					{
						state = GoingForward;
					}
					else
					{
						yawangle = yaw - 90;
					}
				}
			}
			break;
		case TurnLeft:
			cout << " TurnLeft........" << endl;
			fout << " TurnLeft........" << endl;

			if (close(F))
			{
				Turn(--yaw);
			}
			else 
			{
				yawangle = yawAdd(yaw,-10);
				state = GoAfterLeft;
			}
			break;
		case GoAfterLeft:
			cout << " GoAfterLeft........" << endl;
			fout << " GoAfterLeft........" << endl;

			if (ChannelChanged(U)) return;
			if (!close(U))
			{
				GoUp();
				state = GoingUp;
			}
			
			if (ChannelChanged(R)) return;
	
			if ( !IsYawEqualYawangle() ) TurnToYaw();	
			else if(close(F))
			{
				state = TurnLeft;
			}
			else if(close(R)) 
			{
				GoForward();
				firstcall = true;
			}
			else
			{
				if(firstcall)
				{
					firstcall = false;
					time_now = tic_guidance();
					time_old = tic_guidance();
					time_dura = 0;
				}	
				time_now = tic_guidance();
				time_dura = (time_now.tv_sec - time_old.tv_sec)*1000 + (time_now.tv_usec - time_old.tv_usec) / 1000;		
				fout<<time_dura<<endl;
				fout<<(time_now.tv_sec - time_old.tv_sec)*1000<<endl;
				fout<<(time_now.tv_usec - time_old.tv_usec)/1000<<endl;
				if(time_dura < DISKEEPGO/ttc.fNvgSpeed*1000) GoForward();
				else
				{
					if(IsFinish())
					{
						state = GoingForward;
					}
					else
					{
						yawangle = yaw + 90;
					}
				}
			}
			break;
		case StayStill: break;
		default:state = GoingUp; break;
	}
}

#ifdef DEBUG
void DrawText(Mat image, String s, int x, int y, int size = 1)
{
	char buf[99];
	int index = 0;
	int yi = y;
	for (uint i = 0; i < s.length(); i++)
	{
		if (s[i] == '\t') for (int j = 0; j < 8; j++) buf[index++] = ' ';
		else if (s[i] == '\n')
		{
			buf[index] = '\0';
			putText(image, buf, cvPoint(x, yi), size, 1, cv::Scalar(255,255,255), 1);
			index = 0;
			yi += 15 * size;
		}
		else buf[index++] = s[i];
	}
	putText(image, buf, cvPoint(x, yi), size, 1, cv::Scalar(255,255,255), 1);
}
#endif
Mat deal(Mat frame, Mat ROIgrey, int dir)
{
	Mat dealframe = frame.clone();
	int row = dealframe.rows;
	int col = dealframe.cols;
	int stop = col * row;
	if (dir == F) stop = col * 190;
	if (dealframe.isContinuous())
	{
		col *= row;
		row = 1;
	}
	for (int j = 0; j < row; j++)
	{
		uchar *data = dealframe.ptr<uchar>(j);
		for (int i = 0; i < col; i++)
		{
			if (i > stop || data[i] < 1 || data[i] > 254)
			{
				data[i] = 0;
			}
		}
	}
	
	cv::medianBlur(dealframe,dealframe,5);
	cv::erode(dealframe,dealframe,Mat(),Point(-1,-1), ERODETIMES);
	cv::dilate(dealframe,dealframe,Mat(),Point(-1,-1), DILATETIMES);
	
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(dealframe, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	depclose[dir] = false;
	for (unsigned int k = 0; k<contours.size(); k++)
	{
		if (contours[k].size() > AREA)
		{
			#ifdef DEBUG
			if (!ROIgrey.empty()) cv::drawContours(ROIgrey, contours, k, cv::Scalar(255,255,255), 4);
			#endif			
			depclose[dir] = true;
		}
	}
	uchar *framedata = dealframe.ptr<uchar>(0);
	int end = col * row;
	for (int i = 0; i < end; i++)
	{
		if (framedata[i] > 0) framedata[i] = 255;
	}
	#ifdef DEBUG
	char buf[11];
	sprintf(buf, "%d", globalframeindex);
	globalframeindex++;	
	putText(dealframe, buf, cvPoint(11, 22), 1, 1, cv::Scalar(255,255,255), 1);
	#endif
	return dealframe;
}

Mat deal(Mat frame,int dir)
{
	Mat dealframe = frame.clone();
	int row = dealframe.rows;
	int col = dealframe.cols;
	int stop = col * row;
	if (dir == F) stop = col * 190;
	if (dealframe.isContinuous())
	{
		col *= row;
		row = 1;
	}
	for (int j = 0; j < row; j++)
	{
		uchar *data = dealframe.ptr<uchar>(j);
		for (int i = 0; i < col; i++)
		{
			if (i > stop || data[i] < 1 || data[i] > 254)
			{
				data[i] = 0;
			}
		}
	}
	
	cv::medianBlur(dealframe,dealframe,5);
	cv::erode(dealframe,dealframe,Mat(),Point(-1,-1), ERODETIMES);
	cv::dilate(dealframe,dealframe,Mat(),Point(-1,-1), DILATETIMES);
	
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(dealframe, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	depclose[dir] = false;
	for (unsigned int k = 0; k<contours.size(); k++)
	{
		if (contours[k].size() > AREA)
		{
			depclose[dir] = true;
		}
	}
	uchar *framedata = dealframe.ptr<uchar>(0);
	int end = col * row;
	for (int i = 0; i < end; i++)
	{
		if (framedata[i] > 0) framedata[i] = 255;
	}
	#ifdef DEBUG
	char buf[11];
	sprintf(buf, "%d", globalframeindex);
	globalframeindex++;	
	putText(dealframe, buf, cvPoint(11, 22), 1, 1, cv::Scalar(255,255,255), 1);
	#endif
	return dealframe;
}

void DepthProcess()
{
#ifdef DEBUG
	Mat result(3 * HEIGHT, 3 * WIDTH, CV_8UC1);
	Mat tdepthF(HEIGHT, WIDTH, CV_8UC1);
	Mat tdepthC(HEIGHT, WIDTH, CV_8UC1);
	if (!depth[F].empty()) depth[F].convertTo(tdepthF, CV_8UC1);
	if (!depth[channel].empty()) depth[channel].convertTo(tdepthC, CV_8UC1);
#endif
	Mat dealF(HEIGHT, WIDTH, CV_8UC1);
	Mat dealC(HEIGHT, WIDTH, CV_8UC1);
	if (!depth[F].empty()) depth[F].convertTo(dealF, CV_8UC1);
	if (!depth[channel].empty()) depth[channel].convertTo(dealC, CV_8UC1);
	if (!dealF.empty()) dealF = deal(dealF, grayL[F], F);
	if (!dealC.empty()) dealC = deal(dealC, grayL[channel], channel);
#ifdef DEBUG
	for(int j = 0; j < 2 * HEIGHT; j++)
	{
		uchar *res = result.ptr<uchar>(j);
		if (j < HEIGHT)
		{
			uchar *greyF = grayL[F].ptr<uchar>(j);
			uchar *tF = tdepthF.ptr<uchar>(j);
			uchar *dF = dealF.ptr<uchar>(j);
			for(int i = 0; i < 3 * WIDTH; i++)
			{
				if (i < WIDTH) res[i] = greyF[i];
				else if (i < 2*WIDTH) res[i] = tF[i-WIDTH];
				else res[i] = dF[i-2*WIDTH];
			}
		}
		else
		{
			uchar *greyC = grayL[channel].ptr<uchar>(j-HEIGHT);
			uchar *tC = tdepthC.ptr<uchar>(j-HEIGHT);
			uchar *dC = dealC.ptr<uchar>(j-HEIGHT);
			for(int i = 0; i < 3 * WIDTH; i++)
			{
				if (i < WIDTH)
				{
					if (!grayL[channel].empty()) res[i] = greyC[i];
					else res[i] = 0;
				}
				else if (i < 2*WIDTH) res[i] = tC[i-WIDTH];
				else res[i] = dC[i-2*WIDTH];
			}
		}
	}
	DrawText(result, gs, 11, 2*HEIGHT+10);
	if (DEBUG && !result.empty())
	{
		imshow("result",result);
	}
	static cv::VideoWriter writer("./result.avi", CV_FOURCC('D', 'I', 'V', 'X'), 10, Size(3*WIDTH,3*HEIGHT), false);
	writer.write(result);
#endif
}

#ifdef DEBUG_DOCU
String display()
{
	String display;
	char buf[99];
	display += "\n\n";
	display = "Orientation :\tdown\tfront\tright\tup\tleft";
	display += "\ndistance    :";
	for(int i=0;i<CAMERA_PAIR_NUM;i++)
	{
		if(globaldistance[i]<20) sprintf(buf,"\t%.2fm",globaldistance[i]);
		else sprintf(buf,"\t-----");
		display += buf;
	}
	display += "\nultrasonic  :";
	for(int i=0;i<CAMERA_PAIR_NUM;i++)
	{
		if(reliability[i]) sprintf(buf,"\t%.2fm",globalultrasonic[i]);
		else sprintf(buf,"\t-----");
		display += buf;
	}
	display += "\nMin         :";
	for(int i=0;i<CAMERA_PAIR_NUM;i++)
	{
		sprintf(buf,"\t%.2fm",MinDis[i]);
		display += buf;
	}
	display += "\ndepth close :";
	for(int i=0;i<CAMERA_PAIR_NUM;i++)
	{
		if (i == channel)
			if (depclose[i]) sprintf(buf,"\t    1");
			else sprintf(buf,"\t    0");
		else
			if (depclose[i]) sprintf(buf,"\t----1");
			else sprintf(buf,"\t----0");
		display += buf;
	}
	sprintf(buf,"%s%.2f","\n height: ",ttc.fFlyHeight);
	display += buf;
	sprintf(buf,"%s%lf%s%lf","\ndisplay timeout: ",time_dura," / ",DISKEEPGO/ttc.fNvgSpeed*1000);
	display += buf;
	display += "\nex: " + ex;
	sprintf(buf,"%s%s%s%lf","\nmovestate: ",movestate,"\nmovespeed: ",ttc.fNvgSpeed);
	display += buf;
	if (EnableControl) display += "\t(control)";
	else  display += "\t(auto)";
	sprintf(buf,"%s%s","\nstate: ",statetext[state]);
	display += buf;
	sprintf(buf,"%s%.2f%s%.2f%s%.2f","\nYAWANGLE: ",ttc.fYawAngle,"\tyawangle: ",yawangle,"\tyaw: ",yaw);
	display += buf;
	
	const char* channelWord = "0";
	if(channel==1) channelWord  = "F";
	else if(channel==2)  channelWord  = "R";
	else if(channel==3) channelWord  = "U";
	else if(channel==4) channelWord  = "L";
	else if(channel==0) channelWord = "D";
	sprintf(buf,"%s%s","\nchannel: ",channelWord);
	display += buf;
	display += "\n\n";
	//cout<<display;
	return display;
}
#endif
void GetMinDistance(int i)
{
	if (globaldistance[i] < 10 && globalultrasonic[i] > 0)
	{
		TempDis[i] = globaldistance[i] > globalultrasonic[i] ? globalultrasonic[i] : globaldistance[i];
	}
	else if (globaldistance[i] < 10)
	{
		TempDis[i] = globaldistance[i];
	}
	else if (globalultrasonic[i] > 0)
	{
		TempDis[i] = globalultrasonic[i];
	}
	else
	{
		if (SampleTimes > MaxSampleTimes) TempDis[i] = 20;
		if (i == D && SampleTimes > MaxSampleTimes) TempDis[i] = ttc.fFlyHeight;
	}
	if (MinDis[i] > TempDis[i]) MinDis[i] = TempDis[i];
	if (TempDis[i] == 20 && i != D) MinDis[i] = TempDis[i];
	if (SampleTimes > MaxSampleTimes && i != D) MinDis[i] = TempDis[i];
	if (SampleTimes > MaxSampleTimes && i == D && TempDis[i] != 20) MinDis[i] = TempDis[i];
	if (MinDis[i] == 0)
	{
		if(i != D) MinDis[i] = 20;
		else MinDis[i] = 0.1;
	}

#ifdef DIFSPEED
	if(MinDis[F]>CHANSPEDIS && !close(F)) ttc.fNvgSpeed = HIGHSPEED;
	else ttc.fNvgSpeed = LOWSPEED; 
#else
	ttc.fNvgSpeed = LOWSPEED; 
#endif
}
	
int my_callback(int data_type, int data_len, char *content)
{
	g_lock.enter();
	if (e_image == data_type && NULL != content)
	{
		image_data* data = (image_data* )content;
		frame_index = data->frame_index;
		time_stamp = data->time_stamp;
		for ( int d = 0; d < CAMERA_PAIR_NUM; d++ )
		{
#ifdef DEBUG
			if ( data->m_greyscale_image_left[d] )
			{
				grayL[d] = Mat::zeros(HEIGHT,WIDTH,CV_8UC1);
				memcpy( grayL[d].data, data->m_greyscale_image_left[d], IMAGE_SIZE );
			}else grayL[d].release();
			if ( data->m_greyscale_image_right[d] )
			{
				grayR[d] = Mat::zeros(HEIGHT,WIDTH,CV_8UC1);
				memcpy( grayR[d].data, data->m_greyscale_image_right[d], IMAGE_SIZE );
			}else grayR[d].release();
#endif
			
			if ( data->m_depth_image[d] )
			{
				depth[d] = Mat::zeros(HEIGHT,WIDTH,CV_16SC1);
				memcpy( depth[d].data, data->m_depth_image[d], IMAGE_SIZE * 2 );
				
			}
			else depth[d].release();


			if ( data->m_disparity_image[d] )
			{
				disparity[d] = Mat::zeros(HEIGHT,WIDTH,CV_16SC1);
				memcpy( disparity[d].data, data->m_disparity_image[d], IMAGE_SIZE * 2 );
			}else disparity[d].release();
		}
	}
	if ( e_imu == data_type && NULL != content )
	{
		imu *imu_data = (imu*)content;
		frame_index = imu_data->frame_index;
		time_stamp = imu_data->time_stamp;
		acc_x = imu_data->acc_x;
		acc_y = imu_data->acc_y;
		acc_z = imu_data->acc_z;
		for(int i=0;i<4;i++)q[i]=imu_data->q[i];
	}
	if ( e_velocity == data_type && NULL != content )
	{
		velocity *vo = (velocity*)content;
		frame_index = vo->frame_index;
		time_stamp = vo->time_stamp;
		vx = 0.001f * vo->vx;
		vy = 0.001f * vo->vy;
		vz = 0.001f * vo->vz;
	}
	if ( e_obstacle_distance == data_type && NULL != content )
	{
		obstacle_distance *oa = (obstacle_distance*)content;
		frame_index = oa->frame_index;
		time_stamp = oa->time_stamp;
		for ( int i = 0; i < CAMERA_PAIR_NUM; i++ )
		globaldistance[i] = 0.01f * oa->distance[i];
	}
	if ( e_ultrasonic == data_type && NULL != content )
	{
		ultrasonic_data *ultrasonic = (ultrasonic_data*)content;
		frame_index = ultrasonic->frame_index;
		time_stamp = ultrasonic->time_stamp;
		for ( int d = 0; d < CAMERA_PAIR_NUM; d++ )
		{
			globalultrasonic[d] = ultrasonic->ultrasonic[d] * 0.001f;
			reliability[d] = (int)ultrasonic->reliability[d];
		}
	}
	if (e_motion == data_type && NULL != content)
	{
		motion* m = (motion*)content;
		position_in_global_x = m->position_in_global_x;
		position_in_global_y = m->position_in_global_y;
		position_in_global_z = m->position_in_global_z;
	}	
	g_lock.leave();
	g_event.set_event();
	return 0;
}


void* main_guidance(void* data)
{
	// Connect to Guidance and subscribe data
	reset_config();
	int err_code = init_transfer();
	RETURN_IF_ERR(err_code);

	int online_status[CAMERA_PAIR_NUM];
	err_code = get_online_status(online_status);
	RETURN_IF_ERR(err_code);

	cout<<"Sensor online status: ";
	for (int i=0; i<CAMERA_PAIR_NUM; i++)cout<<online_status[i]<<" ";
	cout<<endl;
	// get cali param
	stereo_cali cali[CAMERA_PAIR_NUM];
	err_code = get_stereo_cali(cali);
	RETURN_IF_ERR(err_code);
	
	cout<<"cu\tcv\tfocal\tbaseline\n";
	for (int i=0; i<CAMERA_PAIR_NUM; i++)cout<<cali[i].cu<<"\t"<<cali[i].cv<<"\t"<<cali[i].focal<<"\t"<<cali[i].baseline<<endl;

	//select_motion();	
	channel = (e_vbus_index)D;
	select_obstacle_distance();
	RETURN_IF_ERR( err_code );
	select_ultrasonic();
	RETURN_IF_ERR( err_code );
	err_code = select_depth_image( (e_vbus_index)F );
	RETURN_IF_ERR( err_code );
	err_code = select_depth_image(channel);
	RETURN_IF_ERR( err_code );
	err_code = select_greyscale_image( (e_vbus_index)F , true);
	RETURN_IF_ERR( err_code );
	err_code = select_greyscale_image(channel, true);
	RETURN_IF_ERR( err_code );	
	err_code = set_sdk_event_handler(my_callback);
	RETURN_IF_ERR( err_code );
	err_code = start_transfer();
	RETURN_IF_ERR( err_code );

	// for setting exposure
	exposure_param para;
	para.m_is_auto_exposure = 1;
	para.m_step = 10;
	para.m_expected_brightness = 120;
	para.m_camera_pair_index = (e_vbus_index)F;


	for(int i=0;i<CAMERA_PAIR_NUM;i++)
	{
		TempDis[i] = MinDis[i] = 20;
		SafeDis[i] = SAFEDIS;
		depclose[i] = false;
	}
	SafeDis[F] = SAFEDISF;
	SafeDis[D] = MINHEIGHT;

#ifndef DEBUG	
	Mat operation = Mat::zeros(11,11,CV_8UC1);
#endif

#ifdef DEBUG
	timeval startTime,endTime;
#endif

#ifdef DEBUG_DOCU
		ofstream log("log.txt");
		
#endif
	while(1)
	{
#ifdef DEBUG
		startTime = tic_guidance();
#endif
		g_event.wait_event();
	
#ifndef DEBUG
		imshow("operation_guidance", operation);
#endif
		DepthProcess();
		
		SampleTimes++;
	    for(int i=0;i<CAMERA_PAIR_NUM;i++) GetMinDistance(i);
		if (SampleTimes > MaxSampleTimes) SampleTimes = 0;
		
		if (testhomeward || ttc.iCrntCtrl == CTRL_HOMEWARD ) homeward();
		else navigation();



#ifdef DEBUG_DOCU
		gs = display();
		log << gs;
#endif
		if (state == BreakExit) break;
		char key = 'p';
		key = (char) waitKey(1);
			
		if(key=='q')break;
#ifdef DEBUG
		if(key=='j' || key=='k' || key=='m' || key=='n')// set exposure parameters
		{
			switch(key)
			{
				case 'j':
					if(para.m_is_auto_exposure) para.m_expected_brightness += 20;
					else para.m_exposure_time += 3;
					break;
				case 'k':
					if(para.m_is_auto_exposure) para.m_expected_brightness -= 20;
					else para.m_exposure_time -= 3;
					break;
				case 'm':
					para.m_is_auto_exposure = !para.m_is_auto_exposure;
					cout<<"exposure is "<<para.m_is_auto_exposure<<endl;
					break;
				case 'n'://return to default
					para.m_expected_brightness = para.m_exposure_time = 0;
					break;
				default:break;
			}
			
			cout<<"Setting exposure parameters...."<<endl;
			for(int i=0;i<CAMERA_PAIR_NUM;i++)
			{
				para.m_camera_pair_index = i;
				set_exposure_param(&para);
			}
			
			key = 0;
		}
	
		if (key=='c') 
		{
			int Channel = 1 + channel;
			Channel = Channel > 4 ? 0 : Channel;
			SelectChannel(Channel);
			key = 0;
		}
		if (key == 'p') ;
		else if (key == 'w') GoForward();
		else if (key == 's') GoBack();
		else if (key == 'a') GoLeft();
		else if (key == 'd') GoRight();
		else if (key == 't') GoUp();
		else if (key == 'g') GoDown();
		else if (key == 'r') {Turn(--yaw);}
		else if (key == 'y') {Turn(++yaw);}
		else if (key == 'e') EnableControl = !EnableControl;
		else if (key == 'v') testhomeward = true;
		else if (key == 'z') yawangle = ttc.fYawAngle - 90;
		else if (key == 'x') yawangle = ttc.fYawAngle;
#endif
#ifdef DEBUG
		endTime = tic_guidance();
		cout<<(endTime.tv_sec - startTime.tv_sec)*1000 + (endTime.tv_usec - startTime.tv_usec)/1000<<endl;
		fout<<(endTime.tv_sec - startTime.tv_sec)*1000 + (endTime.tv_usec - startTime.tv_usec)/1000<<endl;
#endif
	}

	err_code = stop_transfer();
	RETURN_IF_ERR( err_code );
	err_code = release_transfer();
	RETURN_IF_ERR( err_code );

#ifdef DEBUG_DOCU
	log.close();
#endif

	cout<<"guidance exit successfully"<<endl;
	return NULL;
}
