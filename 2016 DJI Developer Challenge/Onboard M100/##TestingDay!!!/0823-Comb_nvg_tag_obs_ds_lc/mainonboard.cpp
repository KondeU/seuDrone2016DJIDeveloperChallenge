#include <iostream>
#include <fstream>
#include <iomanip>
#include "DJIHardDriverManifold.h"
#include "conboardsdktask.h"
#include "APIThread.h"
#include "math.h"
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <semaphore.h>
#include <fstream>
extern "C"
{
#include "apriltag_demo.h"
};
using namespace std;


//#define xonly 
//#define yonly
//#define directiononly
//#define xyall
//#define xydirectionall
#define xydirectionallheight
#define gearctrlstate

//#define coutdetail
//#define cameraPIDdetail
#define flightPIDdetail
//#define distancedetail
#define errortime 
#define flightmove
#define cameramove 

#define yawdirection 0
#define Droneheight 6.5
#define speedcnst_x 1.0
#define speedcnst_y 1.0
#define roll_cnst 5.0
#define pitch_cnst 5.0

extern double frequence;
extern int size;
extern int family;


//Data from mobile

extern int iDataFromMobile;





april_res april_result;
int num_april = 0;
extern april_res_all april_result_all;
extern sem_t main_april;
extern int detect_mode;
extern int aprilready;
BroadcastData bd;
double drone_yaw, drone_pitch, drone_roll;
double drone_speed_x, drone_speed_y;
double x_flight,y_flight,z_flight,yaw_flight;
int ctrlstate = 1;

extern char key1;

#define C_EARTH (double) 6378137.0
#define pi 3.1415926535
#define max(x,y)  ( x>y?x:y );

class GPS
{
public:
	double latitude;
	double longitude;
};

class apriltodis {
public:
	int ID;
	double x;
	double y;
	double z;
	double yaw;
};

class PIDperformance {
public:
	double Mp;
	int tp;
	int ts;
	double ess;
	double inierror;
};

double tic();

void updatebd(ConboardSDKScript* script){
	bd = script->getApi()->getBroadcastData();
	drone_roll = atan2(2.0 * (bd.q.q3 * bd.q.q2 + bd.q.q0 * bd.q.q1), 1.0 - 2.0 * (bd.q.q1 * bd.q.q1 + bd.q.q2 * bd.q.q2));
	drone_pitch = asin(2.0 * (bd.q.q2 * bd.q.q0 - bd.q.q3 * bd.q.q1));
	drone_yaw = atan2(2.0 * (bd.q.q3 * bd.q.q0 + bd.q.q1 * bd.q.q2) , - 1.0 + 2.0 * (bd.q.q0 * bd.q.q0 + bd.q.q1 * bd.q.q1));
	drone_roll = drone_roll / pi * 180.0;
	drone_pitch = drone_pitch / pi * 180.0;
	drone_yaw = drone_yaw /pi * 180.0;
	drone_speed_x = bd.v.x;
	drone_speed_y = bd.v.y;
	//cout << "gear: " << bd.rc.gear << endl;	
	if (bd.rc.gear == -10000) {
		if (ctrlstate == 0) { script->getApi()->setControl(true); sleep(2); }
		ctrlstate = 1;
	}
	else
	{
		if (ctrlstate == 1) { script->getApi()->setControl(false); sleep(2); }
		ctrlstate = 0;
	}
}

int update_april(ConboardSDKScript* script) {
	int count_update = 0;
	updatebd(script);
	if (detect_mode == 3 || detect_mode == 4 ||detect_mode == 5 ||detect_mode == 6) {
		aprilready = 0;
		sem_post(&main_april);
		while (aprilready == 0){
			count_update++;
			if(count_update > 500) break;
			usleep(1000);
		}
	}
	april_result = april_result_all.april_result[0];          //need to modify
	num_april = april_result_all.num;
#ifdef coutdetail	
	if(num_april !=0){	
		cout<<"family: "<<family<<"||"<<"size: "<<size<<"||"<<"frequence: "<<frequence<<endl;
		cout <<"ID:"<< april_result.ID << "||"
			<<"num: "<< num_april << "||"
			<<"detail: "<< april_result.x << "||"
			<< april_result.y << "||"
			<< april_result.z << "||"
			<< april_result.yaw << "||"
			<< april_result.roll << "||"
			<< april_result.pitch << "||"
			<<endl;
	}
#endif 	
	return detect_mode;
}

PIDperformance calPIDperformance(double error,int errornum,bool reset) {

	static PIDperformance PIDperformancedate;
	static int isstart = 1;
	static double inierror;
	static int ininum;
	static int clats = 1;
	static int begincalMp = 0;

	if (reset) isstart = 1;
	if (isstart == 1) {
		isstart = 0;	
		inierror = error; 
		ininum = errornum;
		clats = 1;
		begincalMp = 0;
		PIDperformancedate.Mp = 0;
		PIDperformancedate.tp = 0;
		PIDperformancedate.ts = 0;
		PIDperformancedate.ess = 0;
		PIDperformancedate.inierror = inierror;
	}
	errornum = errornum - ininum;

	PIDperformancedate.ess = fabs(error / inierror);
	if (fabs(error / inierror) < 0.10&& clats == 1) { clats = 0; PIDperformancedate.ts = errornum; }
	if (fabs(error / inierror) > 0.10&& clats == 0) { clats = 1; PIDperformancedate.ts = 0; }

	if (begincalMp == 0) {
		if (inierror < 0 && error >0) {
			begincalMp = 1; 
			PIDperformancedate.Mp = error;
			PIDperformancedate.tp = errornum;
		}
		else if (inierror > 0 && error < 0) { 
			begincalMp = 1;
			PIDperformancedate.Mp = error;
			PIDperformancedate.tp = errornum;
		}
	}
	else {
		if (fabs(error) > fabs(PIDperformancedate.Mp)) {
			PIDperformancedate.Mp = error; 
			PIDperformancedate.tp = errornum;
		}
	}
	//cout << "error: " << error << "errornum: " << errornum << "inierror:" << inierror << endl;
	return PIDperformancedate;
}

void update_offset(double & x, double & y, GPS dest_gps, ConboardSDKScript* script)
{
	BroadcastData bd = script->getApi()->getBroadcastData();

	double dlati = dest_gps.latitude - bd.pos.latitude;
	double dlongti = dest_gps.longitude - bd.pos.longitude;

	x = dlati * C_EARTH;
	y = dlongti * C_EARTH * cos(dest_gps.latitude / 2.0 + bd.pos.latitude / 2.0);
}

GPS updategps(GPS origin_gps, double x, double y) {
	GPS dest_gps;
	dest_gps.latitude = x / C_EARTH + origin_gps.latitude;
	dest_gps.longitude = y / (C_EARTH* cos(dest_gps.latitude / 2.0 + origin_gps.latitude / 2.0)) + origin_gps.longitude;
	return dest_gps;
}

int GPScontrol_one(GPS dest_gps, double height, ConboardSDKScript* script) {
	double x, y;
	double speed = 3.0;
	if (height < 2)height = 2;
	else if (height >6)height = 6;
	update_offset(x, y, dest_gps, script);
	FlightData fd;
		if (fabs(x) > 1 || fabs(y) > 1) {	
			update_offset(x, y, dest_gps, script);
			//cout << "x= " << x << "	" << "y= " << y << ";" << "	";
			if (x > 1) fd.x = speed;
			else if (x < -1) fd.x = 0 - speed;
			else fd.x = 0;
			if (y > 1)fd.y = speed;
			else if (y < -1)fd.y = 0 - speed;
			else fd.y = 0;
			fd.flag = 0x51;  //xy?šŽ?šš z???šš yaw¡€??š° ŠÌ?š€šª¡Á?¡Àšº ???šš
			fd.z = 15.0;
			fd.yaw = 0;      //o??š°??ŠÌ??šºš¬a
			script->getFlight()->setFlight(&fd);
		}
		else {
			fd.flag = 0x91;
			fd.x = x;
			if (fabs(fd.x) > 1)fd.x = 0;
			fd.y = y;
			if (fabs(fd.y) > 1)fd.y = 0;
			fd.z = 6;
			fd.yaw = 0;
			script->getFlight()->setFlight(&fd);
		}
	return 1;
}

void camera_angle(double yaw,double roll,double pitch,ConboardSDKScript* script){         
	void *data = (UserData)"0 0 -900 10";
	stringstream s;
	s << (char *)data;
	GimbalAngleData a;
	s >> a.yaw >> a.roll >> a.pitch >> a.duration;
	a.mode = 1;
	a.yaw = yaw * 10;
	a.roll = roll * 10;
	a.pitch = pitch * 10;
	a.duration = 10;
	script->getCamera()->setGimbalAngle(&a);
}
void camera_speed(double yaw,double roll,double pitch,ConboardSDKScript* script){  

	//double yaw_cnst = 1500;
	//double pitch_cnst = 1100;
       
	GimbalSpeedData a;
	a.yaw = yaw*10;
	a.roll = roll*10;
	a.pitch = pitch*10;

	//if(yaw>yaw_cnst) yaw = yaw_cnst;
	//else if(yaw<-yaw_cnst) yaw = -yaw_cnst;
	//if(pitch>pitch_cnst) pitch = pitch_cnst;
	//else if(pitch<-pitch_cnst) pitch = -pitch_cnst;

	script->getCamera()->setGimbalSpeed(&a);
}
void camera_to_april_speed (ConboardSDKScript* script) {
	static int state = 0,stateini = 0;
	static double Ppitch = 2.5, Ipitch = 0.012, Dpitch = 0.245; //2.15 0.004 0.217
	static double pitch_ctrl = 0, pitch_ctrlvar = 0;
	static double error_pitch[4]={0,0,0,0};
	
	#ifdef cameraPIDdetail
	static int count = 0;
	static PIDperformance PIDperformancedate;
	count ++;
	
	switch(key1){
		case 'x': {state = 1; stateini =1; }break;
		case 'c':state = 0;break;
	/*	case 'w':Pyaw += 0.01;break;   
		case 's':Pyaw -= 0.01;break; 
		case 'e':Iyaw += 0.01;break;   
		case 'd':Iyaw -= 0.01;break;
		case 'r':Dyaw += 0.01;break;   
		case 'f':Dyaw -= 0.01;break;*/
		case 't':Ppitch += 0.01;break;   
		case 'g':Ppitch -= 0.01;break;
		case 'y':Ipitch += 0.001;break;   
		case 'h':Ipitch -= 0.001;break;
		case 'u':Dpitch += 0.001;break;   
		case 'j':Dpitch -= 0.001;break;
	}
	#endif 
	
	if (num_april == 0) {                       // diu shi mu biao hou zenme zhaodao mubiao
		camera_angle(0,0,-90,script);
		#ifdef cameraPIDdetail	
			//计算超调量和调节时间
			cout<<"******************************************"<<endl;
			cout<<"state :"<<state<<"||"<<"num : "<<count<<endl;
			//cout<<"PIDyaw: "<<Pyaw<<"||"<<Iyaw<<"||"<<Dyaw<<end;
			cout<<"PIDpitch: "<<Ppitch<<"||"<<Ipitch<<"||"<<Dpitch<<endl;
			cout << "PIDperformance" << PIDperformancedate.Mp << "||" << PIDperformancedate.tp << PIDperformancedate.ts << "||" << PIDperformancedate.ess << endl;
			cout<<" $$$$$$$$$$$$$$$$$ can't find apriltags $$$$$$$$$$$$$$$$$$$$$$"<<endl;
		#endif 	
		error_pitch[0]=0;
		error_pitch[1]=0;
		error_pitch[2]=0;
		error_pitch[3]=0;
		pitch_ctrl = pitch_ctrlvar = 0;
	}
	else {
		double errini_pitch;
		int N = 0; //0 or 1

		errini_pitch = 0 - atan2(april_result.z, april_result.x) / pi * 180.0;
		if (error_pitch[1] !=0 && error_pitch[2] !=0 && fabs(error_pitch[0] - error_pitch[1])<2)
			error_pitch[0] = 0.7865*  errini_pitch +0.2129*error_pitch[1]+0.0006*error_pitch[2];
		else error_pitch[0] = errini_pitch;

		if (fabs(error_pitch[0]) < 6.0) N = 1;
		else N = 0;
		N = 1;

#ifdef cameraPIDdetail
		if (state == 1) {
			if (stateini == 1) {
				PIDperformancedate = calPIDperformance(error_pitch[0], count, true);
				stateini = 0;
				pitch_ctrl = pitch_ctrlvar = 0;	
				error_pitch[1] = 0;
				error_pitch[2] = 0;
				error_pitch[3] = 0;
			}else PIDperformancedate = calPIDperformance(error_pitch[0], count, false);
		}else {
			camera_angle(0,0,18,script);
			pitch_ctrl = pitch_ctrlvar = 0;
			
			error_pitch[1] = 0;
			error_pitch[2] = 0;
			error_pitch[3] = 0;
		}
#endif 		

#ifdef cameraPIDdetail
		if (state == 1) {
#endif 
			switch (detect_mode) {
			case 3: {
				pitch_ctrlvar = 2.420 * (error_pitch[0] - error_pitch[1]) + N * 0.005 * error_pitch[0] + 0.527 * (error_pitch[0] - 2 * error_pitch[1] + error_pitch[2]);
				//pitch_ctrlvar = Ppitch * (error_pitch[0] - error_pitch[1]) + N * Ipitch * error_pitch[0] + Dpitch * (error_pitch[0] - 2 * error_pitch[1] + error_pitch[2]);
			}break;
			case 4: {
				pitch_ctrlvar = 2.870 * (error_pitch[0] - error_pitch[1]) + N * 0.002 * error_pitch[0] + 1.224 * (error_pitch[0] - 2 * error_pitch[1] + error_pitch[2]);
				//pitch_ctrlvar = Ppitch * (error_pitch[0] - error_pitch[1]) + N * Ipitch * error_pitch[0] + Dpitch * (error_pitch[0] - 2 * error_pitch[1] + error_pitch[2]);
			}break;
			case 5: {
				pitch_ctrlvar = 2.390 * (error_pitch[0] - error_pitch[1]) + N * 0.004 * error_pitch[0] + 0.797 * (error_pitch[0] - 2 * error_pitch[1] + error_pitch[2]);
				//pitch_ctrlvar = Ppitch * (error_pitch[0] - error_pitch[1]) + N * Ipitch * error_pitch[0] + Dpitch * (error_pitch[0] - 2 * error_pitch[1] + error_pitch[2]);
			}break;
			case 6: {
				pitch_ctrlvar = 3.110 * (error_pitch[0] - error_pitch[1]) + N * 0.002 * error_pitch[0] + 1.140 * (error_pitch[0] - 2 * error_pitch[1] + error_pitch[2]);
				//pitch_ctrlvar = Ppitch * (error_pitch[0] - error_pitch[1]) + N * Ipitch * error_pitch[0] + Dpitch * (error_pitch[0] - 2 * error_pitch[1] + error_pitch[2]);
			}break;
			}

		pitch_ctrl = pitch_ctrl + pitch_ctrlvar;
		if (fabs(error_pitch[0]) < 4.0) pitch_ctrl = 0;
		camera_speed(0, 0, pitch_ctrl, script);
		error_pitch[3] = error_pitch[2];
		error_pitch[2] = error_pitch[1];
		error_pitch[1] = error_pitch[0];
#ifdef cameraPIDdetail
		}
#endif 		
#ifdef cameraPIDdetail
		cout<<"**********************************"<<endl;
		cout<<"state :"<<state<<"||"<<"num : "<<count<<endl;	
		cout<<"PIDpitch: "<<Ppitch<<"||"<<Ipitch<<"||"<<Dpitch<<endl;
		cout<<"pitchs: "<<errini_pitch<<"||"<< pitch_ctrl << "||"<<pitch_ctrlvar<<endl;	
		cout << "PIDperformance: " << PIDperformancedate.Mp << "||" << PIDperformancedate.tp << "||" << PIDperformancedate.ts << "||" << PIDperformancedate.ess <<"||"<<"inierror"<< PIDperformancedate.inierror<<endl;
#endif 	
	}
}
apriltodis CalApriltodistance(double x_cal,double y_cal,double z_cal,double pitch_cal,double yaw_april,int ID) {
	apriltodis apriltodistance;
	if (ID >= 0 && ID<=6) {
		apriltodistance.x = x_cal*cos(pitch_cal) - z_cal*sin(pitch_cal);
		apriltodistance.y = y_cal;
		apriltodistance.z = x_cal*sin(pitch_cal) + z_cal*cos(pitch_cal);
		apriltodistance.yaw = yaw_april + drone_yaw;
		apriltodistance.ID = ID;
	}
	else {
		apriltodistance.x = 0;
		apriltodistance.y = 0;
		apriltodistance.z = 0;
		apriltodistance.yaw = 0;
		apriltodistance.ID = ID;
	}
	return apriltodistance;
}

void calculate_distance(ConboardSDKScript* script) {
	double x_cal;
	double y_cal;
	double z_cal;        //坐标系  
	double yaw_cal;
	double pitch_cal;

	//if (april_result.ID != 5) {

		static double yaw_flight_temp[4] = { 0,0,0,0 };
		static double yaw_flight_temptemp = 0;
		if (yaw_flight_temp[0] == 0 || yaw_flight_temp[1] == 0 || yaw_flight_temp[2] == 0 || yaw_flight_temp[3] == 0) {
			yaw_flight_temptemp = april_result.yaw + drone_yaw;
		}
		else yaw_flight_temptemp = 0.4689*(april_result.yaw + drone_yaw) + 0.2604*yaw_flight_temp[0] + 0.1562 * yaw_flight_temp[1] + 0.0781 * yaw_flight_temp[2] + 0.0364 * yaw_flight_temp[3];
		yaw_flight_temp[3] = yaw_flight_temp[2];
		yaw_flight_temp[2] = yaw_flight_temp[1];
		yaw_flight_temp[1] = yaw_flight_temp[0];
		yaw_flight_temp[0] = april_result.yaw + drone_yaw;

		if (yaw_flight_temptemp < -180.0) yaw_flight_temptemp += 360;
		else if (yaw_flight_temptemp >180.0) yaw_flight_temptemp -= 360;

		//-180 < yaw_flight < 180   !!!!!!!!!!!!!!!!!!!!

		if (fabs(yaw_flight_temptemp - yaw_flight) > 4.0) {
			yaw_flight = yaw_flight_temptemp;
		}

		//cout<<"yaw_flight_temp: "<<yaw_flight_temptemp<<"||"<<"yaw_flight: "<<yaw_flight<<endl;
	
		x_cal = april_result.x;
		y_cal = april_result.y;
		z_cal = april_result.z;        //坐标系  
		yaw_cal = 0;
		pitch_cal = (-bd.gimbal.pitch) / 180.0*pi;	   //弧度制

		//x_cal = x_temp * cos(yaw_cal) + y_temp*sin(yaws_cal);
		//y_cal = -(x_temp * sin(yaw_cal) - y_temp *cos(yaw_cal));
		//z_cal = z_temp;

		x_flight = x_cal*cos(pitch_cal) - z_cal*sin(pitch_cal);
		y_flight = y_cal;
		z_flight = x_cal*sin(pitch_cal) + z_cal*cos(pitch_cal);

		//x_flight = cos(pitch_cal)*cos(yaw_cal)*x_cal + (sin(pitch_cal)*sin(yaw_cal)-sin(yaw_cal))*y_cal - cos(yaw_cal)*sin(pitch_cal)*z_cal;
		//y_flight = cos(yaw_cal)*y_cal + sin(yaw_cal)*sin(pitch_cal)*y_cal;
		//z_flight = -sin(pitch_cal)*x_cal + cos(pitch_cal)*z_cal;

		//x_flight = cos(pitch_cal)*cos(yaw_cal)*x_cal + cos(pitch_cal)*sin(yaw_cal)*y_cal - sin(pitch_cal)*z_cal;
		//y_flight = -sin(yaw_cal)*x_cal + cos(yaw_cal)*y_cal;
		//z_flight = cos(pitch_cal)*sin(yaw_cal)*x_cal + sin(pitch_cal)*sin(yaw_cal)*y_cal + cos(pitch_cal)*z_cal;
/*	}
	else {
		double pitch_cal = (-bd.gimbal.pitch) / 180.0*pi;
		apriltodis apriltodistance[7];
		int count = 7;
		for (int i=0; i < 7; i++) {
			apriltodistance[i] = CalApriltodistance(april_result_all.april_result[i].x, april_result_all.april_result[i].y, april_result_all.april_result[i].z, pitch_cal, april_result_all.april_result[i].yaw, april_result_all.april_result[i].ID);
			switch (apriltodistance[i].ID) {
			case 0:apriltodistance[i].y -= 0.3; break;
			case 1:break;
			case 2:apriltodistance[i].x += 0.115; break;
			case 3:apriltodistance[i].y += 0.3; break;
			case 4:apriltodistance[i].x -= 0.115; break;
			case 5:apriltodistance[i].x += 0.23; break;
			case 6:apriltodistance[i].x -= 0.23; break;
			default: count--;
			}
		}
		x_flight = 0;
		y_flight = 0;
		z_flight = 0;
		yaw_flight = 0;
		for (int i = 0; i < 7; i++) {
			x_flight += apriltodistance[i].x;
			y_flight += apriltodistance[i].y;
			z_flight += apriltodistance[i].z;
			yaw_flight += apriltodistance[i].yaw;
		}
		x_flight = x_flight / count;
		y_flight = y_flight / count;
		z_flight = z_flight / count;
		yaw_flight = yaw_flight / count;
	}	*/
#ifdef distancedetail
	cout<<"x_cal :"<<x_cal<<"||"<<"y_cal :"<<y_cal<<"||"<<"z_cal :"<<z_cal<<endl;
	cout<<"yaw_cal :"<<yaw_cal/pi*180.0 <<"||"<<"pitch_cal :"<<pitch_cal/pi*180.0<<endl;
	cout<<"x_flight :"<<x_flight<<"||"<<"y_flight :"<<y_flight<<"||"<<"z_flight"<<z_flight<<endl;
#endif 
}

typedef void(*Ctrlfight)(int, double, double, double, double, ConboardSDKScript*);

void ctrlspeed(int landflag, double x, double y, double z, double yaw, ConboardSDKScript* script) {
	FlightData fd;

	if(landflag == 0)fd.flag = 0x53;
	else fd.flag = 0x43;
	
	fd.x = x;
	fd.y = y;
	fd.z = z;
	fd.yaw = yaw;
	if (fd.x > speedcnst_x)fd.x = speedcnst_x;
	else if (fd.x < -speedcnst_x)fd.x = -speedcnst_x;
	if (fd.y > speedcnst_y)fd.y = speedcnst_y;
	else if (fd.y < -speedcnst_y)fd.y = -speedcnst_y;
	if (ctrlstate == 1)script->getFlight()->setFlight(&fd);
}
void ctrlangle(int landflag, double pitch, double roll, double z, double yaw, ConboardSDKScript* script) {
	FlightData fd;

	if(landflag == 0)fd.flag = 0x12;
	else fd.flag = 0x02;
	
	fd.x = roll;        
	fd.y = - pitch;
	fd.z = z;
	fd.yaw = yaw;

	cout<<"@@@@@@@@@@@@@@@@222"<<endl;

	if (fd.x > roll_cnst)fd.x = roll_cnst;
	else if (fd.x < -roll_cnst)fd.x = -roll_cnst;
	if (fd.y > pitch_cnst)fd.y = pitch_cnst;
	else if (fd.y < -pitch_cnst)fd.y = -pitch_cnst;

	if (ctrlstate == 1)script->getFlight()->setFlight(&fd);
}
void Speedctrl(ConboardSDKScript* script) {
	static Ctrlfight ctrlfight = ctrlspeed;
	static int state = 0, stateini = 0;
	static int landstate = 0;
	static double Px = 1.40, Ix = 0.035, Dx = 0.035;
	static double Py = 1.50, Iy = 0.017, Dy = 0.035;
	static double x_ctrl,y_ctrl,z_ctrl,yaw_ctrl;
	static double error_x[4]={0,0,0,0},error_y[4]={0,0,0,0};
	static int changemode = 4;
	static double height_flight = Droneheight;  
	static double Uxp, Uxi, Uxd;
	static double Uyp, Uyi, Uyd;
	static double x_ctrl_p[2], x_ctrl_i[2], x_ctrl_d[2];
	static double y_ctrl_p[2], y_ctrl_i[2], y_ctrl_d[2];

	static int speedmode = 1;	
	static int speedmodeini = 1;


	yaw_flight = 0;


	static ofstream fout;
	static int openini = 1;
	if (openini == 1) {
		openini = 0;
		fout.open("output.txt");
	}else fout.open("output.txt", ios::app);

#ifdef flightPIDdetail
		static int count = 0;
		static PIDperformance PIDperformancedatex;
		static PIDperformance PIDperformancedatey;
		count ++;
		switch(key1){
		case 'x': {state = 1; stateini = 1; }break;
		case 'c':state = 0; break;
		case 'w':Px += 0.01;break;   
		case 's':Px -= 0.01;break; 
		case 'e':Ix += 0.001;break;   
		case 'd':Ix -= 0.001;break;
		case 'r':Dx += 0.001;break;   
		case 'f':Dx -= 0.001;break;
		case 't':Py += 0.01;break;   
		case 'g':Py -= 0.01;break;
		case 'y':Iy += 0.001;break;   
		case 'h':Iy -= 0.001;break;
		case 'u':Dy += 0.001;break;   
		case 'j':Dy -= 0.001;break;
		}

#ifdef gearctrlstate
		if (ctrlstate == 1 && state ==0) { state = 1; stateini = 1; }
		else if (ctrlstate == 0 && state == 1)state = 0;
#endif 	

#endif 	
#ifdef xydirectionallheight
		static int inicount = 0;
		switch (key1) {
		case 'v': {if (detect_mode == 3) { changemode = 4; inicount = 1; } }break;
		case 'b': {if (detect_mode == 4) { changemode = 5; inicount = 1; } } break;
		case 'n': {if (detect_mode == 5) { changemode = 6; inicount = 1; } } break;
		case 'm': {if (detect_mode == 6) { changemode = 7; inicount = 1; } } break;
		}
#endif 	
	if (num_april == 0) {
		#ifdef flightPIDdetail
			cout<<"********************************"<<endl;
			cout << "state :" << state << "||" << "num : " << count << "||" << "detect_mode: " << detect_mode << "||" << "changemode: " << changemode <<"||"<<"frequence :"<<frequence<< endl;
			cout<<"PIDx: "<<Px<<"||"<<Ix<<"||"<<Dx<<endl;
			cout<<"PIDy: "<<Py<<"||"<<Iy<<"||"<<Dy<<endl;
			cout<<" $$$$$$$$$$$$$$$$$ can't find apriltags $$$$$$$$$$$$$$$$$$$$$$"<<endl;

			fout<< "********************************" << endl;
			fout << "state :" << state << "||" << "num : " << count << "||" << "detect_mode: " << detect_mode << "||" << "changemode: " << changemode << "||" << "frequence :" << frequence << endl;
			fout << "PIDx: " << Px << "||" << Ix << "||" << Dx << endl;
			fout << "PIDy: " << Py << "||" << Iy << "||" << Dy << endl;
			fout << "height: "<<bd.pos.height<<endl;
			fout << " $$$$$$$$$$$$$$$$$ can't find apriltags $$$$$$$$$$$$$$$$$$$$$$" << endl;

		#endif 	
			if(changemode !=8){
				error_x[0] = 0;
				error_x[1] = 0;
				error_x[2] = 0;
				error_x[3] = 0;
				x_ctrl_p[1] = x_ctrl_p[0] = 0;
				x_ctrl_i[1] = x_ctrl_i[0] = 0;
				x_ctrl_d[1] = x_ctrl_d[0] = 0;
				y_ctrl_p[1] = y_ctrl_p[0] = 0;
				y_ctrl_i[1] = y_ctrl_i[0] = 0;
				y_ctrl_d[1] = y_ctrl_d[0] = 0;
				error_y[0] = 0;
				error_y[1] = 0;
				error_y[2] = 0;
				error_y[3] = 0;

				speedmodeini = 1;

				//ctrlspeed(1, 0, 0, 0, yaw_flight, script);
			}
			else { 
				fout << " @@@@@@@@@@@@@@@@ landing @@@@@@@@@@@@@@@@@@@" << endl;
				//if (飞机距离地面距离小于0.5m ) {
					ctrlfight(1, x_ctrl, y_ctrl, -2.0, yaw_flight, script);              // todo!!!

				//}else {
				//	ctrlspeed(1, 0, 0, 0, yaw_flight, script);
				//	cout<<"$$$$$$$$$$$$$$$$ "<<"lanfail"<<" $$$$$$$$$$$$$$$$"<<endl;
				//}              
			}
	}
	else {
		//********************filter*************
		static double error_x_temp[4] = { 0,0,0,0 };

		if (detect_mode == 3 || detect_mode == 4) error_x_temp[0] = x_flight;
		else if (detect_mode == 5 || detect_mode == 6) error_x_temp[0] = x_flight-0.3;

		//error_x_temp[0] = x_flight;
		if (error_x_temp[1] != 0 && error_x_temp[2] != 0 && error_x_temp[3] != 0 && fabs(error_x_temp[0] - error_x_temp[1]) < 0.8) {
			error_x[0] = 0.6365*error_x_temp[0] + 0.2118*error_x_temp[1] + 0.1011 *error_x_temp[2] + 0.0506*error_x_temp[3];
		}
		else error_x[0] = error_x_temp[0];

		error_x_temp[3] = error_x_temp[2];
		error_x_temp[2] = error_x_temp[1];
		error_x_temp[1] = error_x_temp[0];

		static double error_y_temp[4] = { 0,0,0,0 };
		error_y_temp[0] = y_flight;
		if (error_y_temp[1] != 0 && error_y_temp[2] != 0 && error_y_temp[3] != 0 && fabs(error_y_temp[0] - error_y_temp[1]) < 0.8) {
			error_y[0] = 0.6365*error_y_temp[0] + 0.2118*error_y_temp[1] + 0.1011 *error_y_temp[2] + 0.0506*error_y_temp[3];
		}
		else error_y[0] = error_y_temp[0];

		error_y_temp[3] = error_y_temp[2];
		error_y_temp[2] = error_y_temp[1];
		error_y_temp[1] = error_y_temp[0];
		//********************filter*************
#ifdef flightPIDdetail	
		if (state == 1) {
			if (stateini == 1) {
				PIDperformancedatex = calPIDperformance(error_x[0], count, true);
				PIDperformancedatey = calPIDperformance(error_y[0], count, true);
				stateini = 0;
				error_x[1] = 0;
				error_x[2] = 0;
				error_x[3] = 0;
				x_ctrl_p[1] = x_ctrl_p[0] = 0;
				x_ctrl_i[1] = x_ctrl_i[0] = 0;
				x_ctrl_d[1] = x_ctrl_d[0] = 0;
				y_ctrl_p[1] = y_ctrl_p[0] = 0;
				y_ctrl_i[1] = y_ctrl_i[0] = 0;
				y_ctrl_d[1] = y_ctrl_d[0] = 0;
				error_y[1] = 0;
				error_y[2] = 0;
				error_y[3] = 0;
			}
			else {
				PIDperformancedatex = calPIDperformance(error_x[0], count, false);
				PIDperformancedatey = calPIDperformance(error_y[0], count, false);
			}
		}
		else {
			detect_mode = 2;
			changemode = 4;
			error_x[0] = 0;
			error_x[1] = 0;
			error_x[2] = 0;
			error_x[3] = 0;
			x_ctrl_p[1] = x_ctrl_p[0] = 0;
			x_ctrl_i[1] = x_ctrl_i[0] = 0;
			x_ctrl_d[1] = x_ctrl_d[0] = 0;
			y_ctrl_p[1] = y_ctrl_p[0] = 0;
			y_ctrl_i[1] = y_ctrl_i[0] = 0;
			y_ctrl_d[1] = y_ctrl_d[0] = 0;
			error_y[0] = 0;
			error_y[1] = 0;
			error_y[2] = 0;
			error_y[3] = 0;
		}
#endif 


#ifdef flightPIDdetail
		if (state == 1) {
#endif
			const double a = 0.1;
			static double x_ctrl_ini = 0;
			static double y_ctrl_ini = 0;

			switch (detect_mode) {
			case 3: {
				Px = 1.0, Ix = 0.010, Dx = 0.035;
				Py = 1.0, Iy = 0, Dy = 0;
				speedmode = 1;
				speedmodeini = 1;
			}break;
			case 4: {
				Px = 1.0, Ix = 0.005, Dx = 0.035;
				Py = 1.0, Iy = 0, Dy = 0;
				speedmode = 1;
				speedmodeini = 1; 
			}break;
			case 5: {
				Px = 1.1, Ix = 0, Dx = 0;
				Py = 1.0, Iy = 0, Dy = 0;
				speedmode = 1;
			}break;
			case 6: {
				Px = 1.0, Ix = 0, Dx = 0;
				Py = 1.0, Iy = 0, Dy = 0;
				speedmode = 1;
			}break;
			}

			if (speedmode == 1) {
				ctrlfight = ctrlspeed;
			}else ctrlfight = ctrlangle;

			if (speedmode == 0 && speedmodeini == 1) {
				drone_pitch = 0;
				drone_roll = 0;
				x_ctrl_p[1] = -drone_pitch;
				y_ctrl_p[1] = drone_roll;
				fout <<  "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl;
				fout << "pitch" << drone_pitch << "||" << "drone_roll " << drone_roll << endl;
				fout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl;
				x_ctrl_i[1] = 0;
				x_ctrl_d[1] = 0;
				y_ctrl_i[1] = 0;
				y_ctrl_d[1] = 0;
				//error_x[0] = 0;
				error_x[1] = 0;
				error_x[2] = 0;
				error_x[3] = 0;
				//error_y[0] = 0;
				error_y[1] = 0;
				error_y[2] = 0;
				error_y[3] = 0;
				speedmodeini = 0;
			}

			fout<<"error:"<<error_x[0]<<"  "<<error_x[1]<<"  "<<error_x[2]<<endl;

			Uxp = Px * (error_x[0] - error_x[1]);
			Uxi = Ix * error_x[0];
			Uxd = Dx * (error_x[0] - 2 * error_x[1] + error_x[2]);
			x_ctrl_p[0] = x_ctrl_p[1] + Uxp;
			x_ctrl_i[0] = x_ctrl_i[1] + Uxi;
			x_ctrl_d[0] = x_ctrl_d[1] + Uxd;
			x_ctrl = x_ctrl_p[0] + x_ctrl_i[0] + a * x_ctrl_d[1] + (1 - a)*x_ctrl_d[0];

			Uyp = Py * (error_y[0] - error_y[1]);
			Uyi = Iy * error_y[0];
			Uyd = Dy * (error_y[0] - 2 * error_y[1] + error_y[2]);
			y_ctrl_p[0] = y_ctrl_p[1] + Uyp;
			y_ctrl_i[0] = y_ctrl_i[1] + Uyi;
			y_ctrl_d[0] = y_ctrl_d[1] + Uyd;
			y_ctrl = y_ctrl_p[0] + y_ctrl_i[0] + a * y_ctrl_d[1] + (1 - a)*y_ctrl_d[0];

			x_ctrl_p[1] = x_ctrl_p[0];
			x_ctrl_i[1] = x_ctrl_i[0];
			x_ctrl_d[1] = x_ctrl_d[0];
			y_ctrl_p[1] = y_ctrl_p[0];
			y_ctrl_i[1] = y_ctrl_i[0];
			y_ctrl_d[1] = y_ctrl_d[0];
			error_x[3] = error_x[2];
			error_x[2] = error_x[1];
			error_x[1] = error_x[0];
			error_y[3] = error_y[2];
			error_y[2] = error_y[1];
			error_y[1] = error_y[0];
			z_ctrl = Droneheight;
			yaw_ctrl = yaw_flight;

			if (speedmode == 1) {
				if (x_ctrl > speedcnst_x)x_ctrl = speedcnst_x;
				else if (x_ctrl < -speedcnst_x)x_ctrl = -speedcnst_x;
				if (y_ctrl > speedcnst_y)y_ctrl = speedcnst_y;
				else if (y_ctrl < -speedcnst_y)y_ctrl = -speedcnst_y;
			}else {
				if (x_ctrl > roll_cnst)x_ctrl = roll_cnst;
				else if (x_ctrl < -roll_cnst)x_ctrl = -roll_cnst;
				if (y_ctrl > pitch_cnst)y_ctrl = pitch_cnst;
				else if (y_ctrl < -pitch_cnst)y_ctrl = -pitch_cnst;
			}

			if (fabs(y_ctrl) < 0.13)y_ctrl = 0;				//不应该是0，应该是不变
			//if (fabs(x_ctrl) < 0.13)x_ctrl = 0;

#ifdef xonly
			ctrlspeed(0, x_ctrl, 0, Droneheight, yawdirection, script);
#endif 
#ifdef yonly
			ctrlspeed(0, 0, y_ctrl, Droneheight, yawdirection, script);
#endif 
#ifdef directiononly
			ctrlspeed(0, 0, 0, Droneheight, yaw_flight, script);
#endif 
#ifdef xyall
			ctrlspeed(0, x_ctrl, y_ctrl, Droneheight, yawdirection, script);
#endif 
#ifdef xydirectionall
			ctrlspeed(0, x_ctrl, y_ctrl, Droneheight, yaw_flight, script);
#endif 
#ifdef xydirectionallheight
			
			switch (changemode) {
			case 4:
				static int beginlandini = 0;
				static int land = 0;
				if (fabs(error_y[0]) < 0.25 && fabs(error_x[0]) < 0.5) {
					beginlandini++;
				}
				else beginlandini = 0;
				if (beginlandini > 50) land = 1;
				fout << "************beginlandini*************  " << beginlandini << "||" << land << endl;
				if (fabs(error_y[0])<0.25 && fabs(error_x[0]) < 0.4 && z_flight>3.5 && land ==1) { landstate = 1; ctrlfight(1, x_ctrl, y_ctrl, -0.5, yaw_flight, script); }
				else {
					if (fabs(error_y[0])<0.25 && fabs(error_x[0]) < 0.4 && z_flight < 3.5 &&inicount == 1) { inicount = 0; height_flight = bd.pos.height; detect_mode = 4; changemode = 5;inicount = 1;}
					landstate = 0;
					ctrlfight(0, x_ctrl, y_ctrl, height_flight, yaw_flight, script);
				} break;
			case 5:if (fabs(error_y[0])<0.25 && fabs(error_x[0]) < 0.5 && z_flight>1.70) { landstate = 1; ctrlfight(1, x_ctrl, y_ctrl, -0.5, yaw_flight, script); }
				   else {
					   static int landini = 0;
					   if (fabs(error_y[0])<0.25 && fabs(error_x[0]) < 0.5 && z_flight < 1.70 &&inicount == 1) {
						   landini++;
						   fout << "************landini*************  " << landini << endl;
						   if (landini > 40) {
							   inicount = 0;
							   height_flight = bd.pos.height;
							   detect_mode = 5;
							   changemode = 6;
							   inicount = 1;
						   }
					   }
					   else {
						   landini = 0;
					   }
					   landstate = 0;
					   ctrlfight(0, x_ctrl, y_ctrl, height_flight, yaw_flight, script);
				   } break;
			case 6:if (fabs(error_y[0])<0.25 && fabs(error_x[0]) < 0.6 && z_flight>0.75) { landstate = 1; ctrlfight(1, x_ctrl, y_ctrl, -0.1, yaw_flight, script); }
				   else {
					   if (fabs(error_y[0])<0.25 && fabs(error_x[0]) < 0.6 && z_flight < 0.75 &&inicount == 1) { inicount = 0; height_flight = bd.pos.height; detect_mode = 6; changemode = 8;inicount = 1; }
					   landstate = 0;
					   ctrlfight(0, x_ctrl, y_ctrl, height_flight, yaw_flight, script);
				   } break;
			case 7:if (fabs(error_y[0])<0.25 && fabs(error_x[0]) < 0.6 && z_flight>0.4) { landstate = 1; ctrlfight(1, x_ctrl, y_ctrl, -2.0, yaw_flight, script); }
				   else {
					   if (fabs(error_y[0])<0.25 && fabs(error_x[0]) < 0.6 && z_flight < 0.4 &&inicount == 1) { inicount = 0; height_flight = bd.pos.height; changemode = 8; }
					   landstate = 0;
					   ctrlfight(0, x_ctrl, y_ctrl, height_flight, yaw_flight, script);
				   } break;
			case 8: { landstate = 1; ctrlfight(1, x_ctrl, y_ctrl, -2.0, yaw_flight, script); }
			default:  ctrlfight(1, x_ctrl, y_ctrl, 0, yaw_flight, script); break;
			}

#ifndef flightPIDdetail
			cout << "********************************" << endl;
			cout << fixed << setprecision(3);
			cout << "detect_mode: " << detect_mode << "||" << "changemode: " << changemode << "||" << "frequence :" << frequence << endl;
			cout << "x: " << x_flight << "||" << x_ctrl << endl;
			cout << "y: " << y_flight << "||" << y_ctrl << endl;
#endif
#endif

#ifdef flightPIDdetail
		}
#endif
 
#ifdef flightPIDdetail	
		//计算超调量和调节时间
		cout << "********************************" << endl;
		cout << fixed << setprecision(4);
		cout << "state :" << state << "||" << "num : " << count << "||" << "detect_mode: " << detect_mode << "||" << "changemode: " << changemode <<"||"<<"landstate:"<< landstate <<"||" << "frequence :" << frequence << endl;
		cout << "PIDx: " << Px << "||" << Ix << "||" << Dx << endl;
		cout << "PIDy: " << Py << "||" << Iy << "||" << Dy << endl;
		cout << "x: " << x_flight << "||" << x_ctrl << "||" << drone_speed_x << "||" << drone_pitch << endl;
		cout << "y: " << y_flight << "||" << y_ctrl << "||" << drone_speed_y << "||" << drone_roll << endl;
		cout << "z: " << z_flight << endl;
		cout << "PIDperformancex: " << PIDperformancedatex.Mp << "||" << PIDperformancedatex.tp << "||" << PIDperformancedatex.ts << "||" << PIDperformancedatex.ess << "||" << "inierror" << PIDperformancedatex.inierror << endl;
		cout << "PIDperformancey: " << PIDperformancedatey.Mp << "||" << PIDperformancedatey.tp << "||" << PIDperformancedatey.ts << "||" << PIDperformancedatey.ess << "||" << "inierror" << PIDperformancedatey.inierror << endl;

		fout << "********************************" << endl;
		fout << fixed << setprecision(4);
		fout << "state :" << state << "||" << "num : " << count << "||" << "detect_mode: " << detect_mode << "||" << "changemode: " << changemode << "||" << "landstate:" << landstate << "||" << "frequence :" << frequence << endl;
		fout << "PIDx: " << Px << "||" << Ix << "||" << Dx << endl;
		fout << "PIDy: " << Py << "||" << Iy << "||" << Dy << endl;
		fout << "x: " << x_flight << "||" << error_x[0] << "||" << x_ctrl << "||" << drone_speed_x << "||" << drone_pitch << endl;
		fout << "y: " << y_flight << "||" << error_y[0] << "||" << y_ctrl << "||" << drone_speed_y << "||" << drone_roll << endl;
		fout << "z: " << z_flight << endl;
		fout << "gimbal.pitch: " << bd.gimbal.pitch << endl;
		fout << "PIDperformancex: " << PIDperformancedatex.Mp << "||" << PIDperformancedatex.tp << "||" << PIDperformancedatex.ts << "||" << PIDperformancedatex.ess << "||" << "inierror" << PIDperformancedatex.inierror << endl;
		fout << "PIDperformancey: " << PIDperformancedatey.Mp << "||" << PIDperformancedatey.tp << "||" << PIDperformancedatey.ts << "||" << PIDperformancedatey.ess << "||" << "inierror" << PIDperformancedatey.inierror << endl;

#endif 	

	}
	fout.close();
}
void land_in_car(ConboardSDKScript* script, double height) {
	GPS gps;
	gps.latitude = 43.2299043;
	gps.longitude = -75.4180322;

	detect_mode = 2;
	double time_old,time_new,time_dura;
	                      
	double T_time = (double)1.0 / frequence;
	update_april(script);
	cout<<"num_april :"<<num_april<<endl;
	int laststate = 0;
	while (1) {
		updatebd(script);
		if (bd.pos.height > 15.0)break;
		else ctrlspeed(1, 0, 0, 1.0, 0, script);
		usleep(20000);
		cout<<"toheifht"<<endl;
	}
	while (1) {
		T_time = (double)1.0 / frequence;
		if (num_april ==0) {
			if(laststate == 1){laststate = 0; sem_post(&main_april);}
			if (key1 == 'q'||iDataFromMobile == 3) { detect_mode = 2; return; }			
			update_april(script);
			GPScontrol_one(gps, Droneheight, script);
#ifdef cameramove
			camera_to_april_speed(script);
#endif
#ifdef flightmove
			Speedctrl(script);
#endif
			cout << "wait for apriltags" << endl;
			usleep(20000);
		}
		else{
			time_old = tic();
			laststate = 1;
			if (key1 == 'q'||iDataFromMobile ==3) { detect_mode = 2; sem_post(&main_april); return; }
			if (detect_mode == 2)detect_mode = 3;
			update_april(script);
			calculate_distance(script);
#ifdef cameramove
			camera_to_april_speed(script);
#endif
#ifdef flightmove
			Speedctrl(script);
#endif
			time_new = tic();
			time_dura = time_new - time_old;
			if (time_dura > T_time) {
				#ifdef errortime	
					cout << "using time error :"<<time_dura-T_time<<endl;  
				#endif       
			}
			else {
				#ifdef errortime
					cout<<"sleep time :"<<T_time - time_dura<<"frequence :"<<frequence<<endl;
				#endif	
				usleep((T_time - time_dura) * 1000000);
			}	
		}
	}
	detect_mode = 2;
	sem_post(&main_april);
}



















//Navigate------------------------------------------------------------------------

#include "tc.h"

#pragma pack(1) //using in data sending

TThreadController ttc;

int iNvgState;

#define M_COUTDETAIL
#define M_FOUTDETAIL
#define M_LOOKFORAPRILTAGS
#define M_ENABLEQUITMSG
//#define M_CALCCORNERGPS
#define M_LOWBATBREAK

//#define C_EARTH    ((double)(6378137.0))
#define C_PI ((double)(3.1415926535897932))

//#define C_NVGSPEED ((double)(0.7))
#define C_NVGSTEPLENGTH ((double)(4.0))

#define C_SA_SAFE   (2)
#define C_SA_LENGTH (28)
#define C_SA_WIDTH  (28)
#define C_SA_OFA_L  (C_SA_LENGTH + C_SA_SAFE)
#define C_SA_OFA_W  (C_SA_WIDTH + C_SA_SAFE)

#define CL_RADTODEG(rad) ((rad)*(180)/(C_PI))

#define NVG_INVALID    -1
#define NVG_INIT       0
#define NVG_INITALTI   1
#define NVG_FLYTOBEG   2
#define NVG_ADJSTALTI  3
#define NVG_NORMAL     4
#define NVG_PREPAREYAW 5
#define NVG_YAWING     6
#define NVG_OBSTACLE   7
#define NVG_BACKTOLINE 8
#define NVG_BTLFORWARD 9
#define NVG_OUTOFAREA  10

#define NVG_MV_FORWARD 0
#define NVG_MV_LEFT    1
#define NVG_MV_RIGHT   2

#define GDC_FORWARD 0
#define GDC_LEFT    1
#define GDC_RIGHT   2

struct TGPS
{
	double dLatitude;  //Latitude
	double dLongitude; //Longitude
};

struct TBaseLine
{
	TGPS tgpsOrg;
	TGPS tgpsPnt;
	double dX;
	double dY;
	double dK;
};

#ifdef M_FOUTDETAIL
ofstream fout("Navigate.log");
#endif

TGPS GetCrntGPS(const ConboardSDKScript * pScript)
{
	TGPS tgpsCrnt;

	BroadcastData bd = pScript->getApi()->getBroadcastData();

	tgpsCrnt.dLatitude = bd.pos.latitude;
	tgpsCrnt.dLongitude = bd.pos.longitude;

	return tgpsCrnt;
}

void CalcOffset(double & x, double & y, const TGPS & tgpsBase, const TGPS & tgpsDest)
{
	double dLatitude  = tgpsDest.dLatitude  - tgpsBase.dLatitude;
	double dLongitude = tgpsDest.dLongitude - tgpsBase.dLongitude;

	x = dLatitude * C_EARTH;
	y = dLongitude * C_EARTH * cos(tgpsDest.dLatitude / 2.0 + tgpsBase.dLatitude / 2.0);

	return;
}

void CalcOffset(double & x, double & y, const TGPS & tgpsDest, const ConboardSDKScript * pScript)
{
	CalcOffset(x, y, GetCrntGPS(pScript), tgpsDest);
	return;
}

TGPS CalcGPS(const TGPS & tgpsBase, const double & x, const double & y)
{
	TGPS tgpsDest;

	tgpsDest.dLatitude  = x / C_EARTH + tgpsBase.dLatitude;
	tgpsDest.dLongitude = y / (C_EARTH * cos(tgpsDest.dLatitude / 2.0 + tgpsBase.dLatitude / 2.0)) + tgpsBase.dLongitude;

	return tgpsDest;
}

float CalcYaw(const double & x, const double & y)
{
	float fYaw = 0; //Default direction is North

	/*

	Yaw angle calculate way:

	# : rad_to_deg(rad) (rad * 180 / PI)
	1 : x > 0, y > 0 :  90 - rad_to_deg(arctan(x/y))
	2 : x > 0, y < 0 : -90 + rad_to_deg(arctan(x/-y))
	3 : x > 0, y = 0 :   0
	4 : x < 0, y > 0 :  90 + rad_to_deg(arctan(-x/y))
	6 : x < 0, y = 0 : 180
	5 : x < 0, y < 0 : -90 - rad_to_deg(arctan(x/y))
	7 : x = 0, y > 0 :  90
	8 : x = 0, y < 0 : -90
	9 : x = 0, y = 0 : Error

	*/

	if (0 == y)
	{
		if (x > 0)
		{
			fYaw = 0;
		}
		else if (x < 0)
		{
			fYaw = 180;
		}
		else
		{
			cerr << "Navigate Error : In function CalcYaw : Both x and y value is zero, now set yaw with 0." << endl;
			fYaw = 0;
		}
	}
	else
	{
		fYaw = ((y > 0) ? 90 : -90) - CL_RADTODEG(atan(x / y));
	}

	return fYaw;
}

float CalcYaw(const TGPS & tgpsBase, const TGPS & tgpsDest)
{
	double x, y;
	CalcOffset(x, y, tgpsBase, tgpsDest);
	return CalcYaw(x, y);
}

float CalcYaw(const TGPS & tgpsDest, const ConboardSDKScript * pScript)
{
	return CalcYaw(GetCrntGPS(pScript), tgpsDest);
}

bool IsNearDest(const TGPS & tgpsDest, const ConboardSDKScript * pScript)
{
	double x, y;

	CalcOffset(x, y, tgpsDest, pScript);
	
	if ((fabs(x) < 0.5) && (fabs(y) < 0.5))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void CalcBaseLine(TBaseLine & tbl, const TGPS & tgpsOrg, const TGPS & tgpsPnt)
{
	tbl.tgpsOrg = tgpsOrg;
	tbl.tgpsPnt = tgpsPnt;

	CalcOffset(tbl.dX, tbl.dY, tgpsOrg, tgpsPnt);
	
	if (fabs(tbl.dX) < 0.000001)
	{
		tbl.dX = ((tbl.dX < 0) ? -1 : 1) * 0.000001;
	}

	tbl.dK = tbl.dY / tbl.dX;
}

double CalcDistance(TBaseLine & tbl, const ConboardSDKScript * pScript)
{
	double dDis;
	double x, y;

	CalcOffset(x, y, tbl.tgpsOrg, GetCrntGPS(pScript));

	dDis = (tbl.dK * x - y) / (sqrt(tbl.dK * tbl.dK + 1));

	return dDis;
}

int IsAltitudeInRange(float fMin, float fMax, const ConboardSDKScript * pScript)
{
	BroadcastData bd = pScript->getApi()->getBroadcastData();
	//fout << bd.pos.height << endl;//
	if(bd.pos.height < fMin) return -1;
	if(bd.pos.height > fMax) return 1;
	return 0;
}

void NvgFlying(TGPS tgpsDest, const ConboardSDKScript * pScript)
{
	FlightData fd;
	ttc.fYawAngle = CalcYaw(tgpsDest, pScript);
	fd.flag = 0x53;
	fd.x    = ttc.fNvgSpeed;
	fd.y    = 0;
	fd.z    = ttc.fFlyHeight;
	fd.yaw  = ttc.fYawAngle;
	pScript->getFlight()->setFlight(&fd);
}

void NvgFlyingToBeg(TGPS tgpsDest, const ConboardSDKScript * pScript)
{
	FlightData fd;
	ttc.fYawAngle = CalcYaw(tgpsDest, pScript);
	fd.flag = 0x53;
	fd.x = 3;
	fd.y = 0;
	fd.z = ttc.fFlyHeight;
	fd.yaw = ttc.fYawAngle;
	pScript->getFlight()->setFlight(&fd);
}

void NvgYawing(float fYaw, const ConboardSDKScript * pScript)
{
	FlightData fd;
	/*double x, y;
	CalcOffset(x, y, GetCrntGPS(pScript), pScript);
	if (fabs(x) > 1)
	{
		x = ((x > 0) ? 1 : -1);
	}
	if (fabs(y) > 1)
	{
		y = ((y > 0) ? 1 : -1);
	}*/
	ttc.fYawAngle = fYaw;
	fd.flag = 0x91;
	/*fd.x    = x;
	fd.y    = y;*/

	fd.x = 0;//
	fd.y = 0;//

	fd.z    = ttc.fFlyHeight;
	fd.yaw  = ttc.fYawAngle;
	pScript->getFlight()->setFlight(&fd);
}

void NvgMoving(int iDirection, const ConboardSDKScript * pScript)
{
	FlightData fd;
	fd.flag = 0x53;
	fd.z    = ttc.fFlyHeight;
	fd.yaw  = ttc.fYawAngle;

	switch (iDirection)
	{
	case NVG_MV_FORWARD: //Moving Forward
		fd.x = ttc.fNvgSpeed;
		fd.y = 0;
		break;

	case NVG_MV_LEFT: //Moving Left
		fd.x = 0;
		fd.y = -ttc.fNvgSpeed;
		break;

	case NVG_MV_RIGHT: //Moving Right
		fd.x = 0;
		fd.y = ttc.fNvgSpeed;
		break;

	default: //Error
		fd.x = 0;
		fd.y = 0;
		break;
	}

	pScript->getFlight()->setFlight(&fd);
}

void NvgAltituding(bool bIsUping, const ConboardSDKScript * pScript)
{
	FlightData fd;
	fd.flag = 0x43;
	fd.x = 0;
	fd.y = 0;
	fd.z = bIsUping ? 0.7 : -0.7;	
	fd.yaw = ttc.fYawAngle;
	pScript->getFlight()->setFlight(&fd);
	
	//fout << "Tic:" << tic() << endl;
	//fout << "ttc.YawAngle:" << ttc.fYawAngle << endl;
	//fout << "ttc.iCrntCtrl:" << ttc.iCrntCtrl << endl;
}

void SetCameraAngle(double dYaw, double dRoll, double dPitch, const ConboardSDKScript * pScript)
{
	void * data = (UserData)("0 0 -900 10");
	stringstream s;
	s << (char *)data;
	GimbalAngleData a;
	s >> a.yaw >> a.roll >> a.pitch >> a.duration;
	a.mode     = 1;
	a.yaw      = dYaw * 10;
	a.roll     = dRoll * 10;
	a.pitch    = dPitch * 10;
	a.duration = 10;
	pScript->getCamera()->setGimbalAngle(&a);
}

/*void SetCameraSpeed(double dYaw, double dRoll, double dPitch, ConboardSDKScript* pScript)
{
	GimbalSpeedData a;
	a.yaw   = dYaw * 10;
	a.roll  = dRoll * 10;
	a.pitch = dPitch * 10;
	pScript->getCamera()->setGimbalSpeed(&a);
}*/

void AdjustCamera(const ConboardSDKScript * pScript)
{
	static int iCrntState = NVG_INVALID, iPrevState = NVG_INVALID;
	
	static bool bCrntUltrasonicLeft = false, bCrntUltrasonicRight = false;
	static bool bPrevUltrasonicLeft = false, bPrevUltrasonicRight = false;
	
	iPrevState = iCrntState;
	iCrntState = iNvgState;
	
	bPrevUltrasonicLeft = bCrntUltrasonicLeft;
	bCrntUltrasonicLeft = ttc.bUltrasonicLeft;
	bPrevUltrasonicRight = bCrntUltrasonicRight;
	bCrntUltrasonicRight = ttc.bUltrasonicRight;
	
	if (iPrevState == iCrntState)
	{
		if (iCrntState == NVG_NORMAL)
		{
			if (bPrevUltrasonicLeft  != bCrntUltrasonicLeft &&
				bPrevUltrasonicRight != bCrntUltrasonicRight)
			{
				double dCmrYaw = 0;
				if (bCrntUltrasonicLeft)
				{
					dCmrYaw -= 45;
				}
				if (bCrntUltrasonicRight)
				{
					dCmrYaw += 45;
				}
				SetCameraAngle(dCmrYaw, 0, -45, pScript);
			}
		}
	}
	else
	{
		switch (iCrntState)
		{
		case NVG_INIT:
			{
				SetCameraAngle(0, 0, -45, pScript);
			}
			break;
		
		case NVG_NORMAL:
			{
				double dCmrYaw = 0;
				if (bCrntUltrasonicLeft)
				{
					dCmrYaw -= 45;
				}
				if (bCrntUltrasonicRight)
				{
					dCmrYaw += 45;
				}
				SetCameraAngle(dCmrYaw, 0, -45, pScript);
			}
			break;

			case NVG_OBSTACLE:
			{
				SetCameraAngle(0, 0, -45, pScript);
			}
			break;
			
			case NVG_BACKTOLINE:
			{
				SetCameraAngle(180, 0, -45, pScript);				
			}
			break;
			
			case NVG_BTLFORWARD:
			{
				if (bCrntUltrasonicLeft)
				{
					SetCameraAngle(-45, 0, -45, pScript);	
				}		
				else if (bCrntUltrasonicRight)
				{
					SetCameraAngle(45, 0, -45, pScript);	
				}
				else
				{
					SetCameraAngle(0, 0, -45, pScript); //Error
				}
			}
			break;
		}
	}
}

bool IsInSearchArea(const TGPS tgpsCorner[4], const ConboardSDKScript * pScript)
{
	TBaseLine tblLength, tblWidth;
	CalcBaseLine(tblLength, tgpsCorner[0], tgpsCorner[1]);
	CalcBaseLine(tblWidth,  tgpsCorner[0], tgpsCorner[3]);

	double dDistToLength, dDistToWidth;
	dDistToLength = CalcDistance(tblLength, pScript);
	dDistToWidth  = CalcDistance(tblWidth,  pScript);

	#ifdef M_COUTDETAIL
	cout << "Dist to length:" << dDistToLength << " || " << "Dist to width:" << dDistToWidth << endl;
	#endif
	#ifdef M_FOUTDETAIL
	fout << "Dist to length:" << dDistToLength << " || " << "Dist to width:" << dDistToWidth << endl;
	#endif

	/*if (dDistToLength > 0 || dDistToLength < -C_SA_WIDTH)
	{
		return false;
	}

	if (dDistToWidth < 0 || dDistToWidth > C_SA_LENGTH)
	{
		return false;
	}*/

	if (dDistToLength < -C_SA_SAFE || dDistToLength > C_SA_OFA_W)
	{
		return false;
	}

	if (dDistToWidth > C_SA_SAFE || dDistToWidth < -C_SA_OFA_L)
	{
		return false;
	}

	return true;
}


#ifdef M_LOOKFORAPRILTAGS
int LookforApriltags(/*const */ConboardSDKScript * pScript)
{
	static int iFinded = 0;
	static int iID[5] = { -1, -1, -1, -1, -1 };

	union UTagsInfo	
	{
		struct TTagsInfo
		{
			int iID;
			float fTagsCorner[4][2];
			double dLatitude;
			double dLongitude;
		} tti;
		unsigned char byData[sizeof(tti)];
	} uti;
	int iCrntID = -1;

	update_april(pScript);

	if (! num_april)
	{
		goto RTN_LFA;
	}

	iCrntID = april_result.ID;

	for (int i = 0; i < iFinded; i++)
	{
		if (iCrntID == iID[i])
		{
			goto RTN_LFA;
		}
	}

	//A new block, to avoid error of "crosses initialization of.."

	{
		updatebd(pScript);
		calculate_distance(pScript);

		double dTagsOffsetX = x_flight * sin(drone_yaw) + y_flight * cos(drone_yaw);
		double dTagsOffsetY = x_flight * cos(drone_yaw) - y_flight * sin(drone_yaw);
		TGPS tgpsApriltag = CalcGPS(GetCrntGPS(pScript), dTagsOffsetX, dTagsOffsetY);

		//Send info to mobile

		unsigned char byData[sizeof(uti.tti)];
		unsigned int iCounter = 0;

		uti.tti.iID = iCrntID;
		for(int i = iCounter; i < iCounter + sizeof(int); i++)
		{
			byData[i] = uti.byData[iCounter + sizeof(int) - 1 - (i - iCounter)];
		}
		iCounter += sizeof(int);

		for (int i1 = 0; i1 < 4; i1++)
		{
			for (int i2 = 0; i2 < 2; i2++)
			{
				uti.tti.fTagsCorner[i1][i2] = april_result.corner[i1][i2];
				for(int i = iCounter; i < iCounter + sizeof(float); i++)
				{
					byData[i] = uti.byData[iCounter + sizeof(float) - 1 - (i - iCounter)];
				}
				iCounter += sizeof(float);
				//fout << uti.tti.fTagsCorner[i1][i2] << "||";
			}
			//fout << endl;
		}
		
		uti.tti.dLatitude  = tgpsApriltag.dLatitude;
		uti.tti.dLongitude = tgpsApriltag.dLongitude;
		for(int i = iCounter; i < iCounter + sizeof(double); i++)
		{
			byData[i] = uti.byData[iCounter + sizeof(double) - 1 - (i - iCounter)];
		}
		iCounter += sizeof(double);
		for(int i = iCounter; i < iCounter + sizeof(double); i++)
		{
			byData[i] = uti.byData[iCounter + sizeof(double) - 1 - (i - iCounter)];
		}
		iCounter += sizeof(double);

		for(int i = 0; i < 3; i++)
		{
			pScript->getApi()->sendToMobile((uint8_t*)(&byData), (uint8_t)sizeof(uti));
			usleep(20000);
		}

		//output info

		#ifdef M_COUTDETAIL
		cout << "Navigate : Find a new Apriltag.." << endl;
		cout << "ID:" << uti.tti.iID               << endl
			 << "Latitude: " << uti.tti.dLatitude  << endl
			 << "Longitude:" << uti.tti.dLongitude << endl;
		cout << endl;
		#endif

		#ifdef M_FOUTDETAIL
		fout << "Navigate : Find a new Apriltag.." << endl;
		fout << "ID:" << uti.tti.iID               << endl
			 << "Latitude: " << uti.tti.dLatitude  << endl
			 << "Longitude:" << uti.tti.dLongitude << endl;
		fout << endl;
		#endif

		//Wait for mobile shot screen

		double dWaiting = 5.0;
		
		#ifdef M_COUTDETAIL
		cout << "Navigate : Waiting for mobile shot screen.." << endl;
		cout << "Waiting:" << dWaiting << "s" << endl;
		cout << "Waiting:" << dWaiting * 1000000 << "us" << endl;
		cout << endl;
		#endif

		#ifdef M_FOUTDETAIL
		fout << "Navigate : Waiting for mobile shot screen.." << endl;
		fout << "Waiting:" << dWaiting << "s" << endl;
		fout << "Waiting:" << dWaiting * 1000000 << "us" << endl;
		fout << endl;
		#endif
	
		usleep(dWaiting * 1000000);
		
		if((iDataFromMobile - 10) == iCrntID)
		{
			iID[iFinded] = iCrntID;
			iFinded++;
		}
		else
		{
			#ifdef M_COUTDETAIL
			cout << "Navigate : Do not recv mobile respose.." << endl;
			cout << endl;
			#endif

			#ifdef M_FOUTDETAIL
			fout << "Navigate : Do not recv mobile respose.." << endl;
			fout << endl;
			#endif
		}
	}

RTN_LFA:
	return iFinded;
}
#endif

int GetBattery(const ConboardSDKScript * pScript)
{
	BroadcastData bd = pScript->getApi()->getBroadcastData();
	return bd.battery;
}


bool Navigate(const ConboardSDKScript * pScript)
{
	static unsigned long long ullIndex = 0;
	#ifdef M_COUTDETAIL
	cout << "Navigate : " << ullIndex << ".." << endl;
	#endif
	#ifdef M_FOUTDETAIL 
	fout << "Navigate : " << ullIndex << ".." << endl;
	#endif
	ullIndex++;

	static TGPS tgpsCorner[4]; //4 corners GPS

	static int iState = NVG_INIT; //Current state

	static unsigned int uiGPSTotal = 0, uiGPSCounter = 0;

	static double dStepOffsetX = 0, dStepOffsetY = 0;

	static TGPS  tgpsNext;
	static float fYaw;
	static TBaseLine tbl;

	static bool bOutOfSearchArea = false;

	if (1 == ttc.iRequestControl)
	{
		if (
			NVG_NORMAL     == iState ||
			NVG_BTLFORWARD == iState /*||
			NVG_OUTOFAREA  == iState*/
			)
		{
			ttc.iCrntCtrl = CTRL_OBSTACLE; //Obstacle Avoidance control
			iState = NVG_OBSTACLE;
		}
	}

	iNvgState = iState;

	switch (iState)
	{
	case NVG_INIT:
		{
			#ifdef M_CALCCORNERGPS
			tgpsCorner[0] = GetCrntGPS(pScript);
			
			tgpsCorner[0].dLatitude  = 0.556527397255627;//
			tgpsCorner[0].dLongitude = 2.07362961098558;//
			
			tgpsCorner[1] = CalcGPS(tgpsCorner[0],  -2, 28);
			tgpsCorner[2] = CalcGPS(tgpsCorner[0], 10, 28);
			tgpsCorner[3] = CalcGPS(tgpsCorner[0], 10,  0);
			#else
			tgpsCorner[0].dLatitude  = 43.2304999;
			tgpsCorner[0].dLongitude = -75.4197515;
			
			tgpsCorner[1].dLatitude  = 43.2305196; 
			tgpsCorner[1].dLongitude = -75.4191453;
			
			tgpsCorner[2].dLatitude  = 43.2309682;
			tgpsCorner[2].dLongitude = -75.4191223;
			
			tgpsCorner[3].dLatitude  = 43.2309511;
			tgpsCorner[3].dLongitude = -75.4197423;
			#endif

			CalcOffset(dStepOffsetX, dStepOffsetY, tgpsCorner[1], tgpsCorner[2]);

			double dLength = sqrt(dStepOffsetX * dStepOffsetX + dStepOffsetY * dStepOffsetY);

			uiGPSTotal = (unsigned int)((2 * dLength / C_NVGSTEPLENGTH) + 1);

			dStepOffsetX = dStepOffsetX * C_NVGSTEPLENGTH / dLength;
			dStepOffsetY = dStepOffsetY * C_NVGSTEPLENGTH / dLength;

			ttc.iCrntCtrl = CTRL_NAVIGATE;
			ttc.iRequestControl = 0;

			ttc.fYawAngle  = CalcYaw(GetCrntGPS(pScript), tgpsCorner[0]);
			ttc.fNvgSpeed  = 1.0;

			ttc.iGuidance  = GDC_FORWARD;
			ttc.bBTLUnsafe = false;

			ttc.bUltrasonicLeft  = false;
			ttc.bUltrasonicRight = false;

			ttc.iPian = PianNone;

			fYaw = ttc.fYawAngle;
			tgpsNext = tgpsCorner[0];
			CalcBaseLine(tbl, tgpsCorner[0], tgpsCorner[1]);

			iState = NVG_INITALTI;
			//iState = NVG_PREPAREYAW;

			#ifdef M_COUTDETAIL
			cout << setprecision(15);
			cout << "State: Init." << endl;
			cout << "GPS Corners:" << endl;
			cout << "[0]->" << endl
				 << "Latitude:  " << tgpsCorner[0].dLatitude  << endl
				 << "Longitude: " << tgpsCorner[0].dLongitude << endl;
			cout << "[1]->" << endl
				 << "Latitude:  " << tgpsCorner[1].dLatitude  << endl
				 << "Longitude: " << tgpsCorner[1].dLongitude << endl;
			cout << "[2]->" << endl
				 << "Latitude:  " << tgpsCorner[2].dLatitude  << endl
				 << "Longitude: " << tgpsCorner[2].dLongitude << endl;
			cout << "[3]->" << endl
				 << "Latitude:  " << tgpsCorner[3].dLatitude  << endl
				 << "Longitude: " << tgpsCorner[3].dLongitude << endl;
			cout << "Total GPS counts:" << uiGPSTotal << endl;
			cout << "StepOffsetX:" << dStepOffsetX << endl
				 << "StepOffsetY:" << dStepOffsetY << endl;
			#endif
			#ifdef M_FOUTDETAIL
			fout << setprecision(15);
			fout << "State: Init." << endl;
			fout << "GPS Corners:" << endl;
			fout << "[0]->" << endl
				 << "Latitude:  " << tgpsCorner[0].dLatitude  << endl
				 << "Longitude: " << tgpsCorner[0].dLongitude << endl;
			fout << "[1]->" << endl
				 << "Latitude:  " << tgpsCorner[1].dLatitude  << endl
				 << "Longitude: " << tgpsCorner[1].dLongitude << endl;
			fout << "[2]->" << endl
				 << "Latitude:  " << tgpsCorner[2].dLatitude  << endl
				 << "Longitude: " << tgpsCorner[2].dLongitude << endl;
			fout << "[3]->" << endl
				 << "Latitude:  " << tgpsCorner[3].dLatitude  << endl
				 << "Longitude: " << tgpsCorner[3].dLongitude << endl;
			fout << "Total GPS counts:" << uiGPSTotal << endl;
			fout << "StepOffsetX:" << dStepOffsetX << endl
				 << "StepOffsetY:" << dStepOffsetY << endl;
			#endif
		}
		break;

	case NVG_INITALTI:
		{
			ttc.fFlyHeight = 15.0;
			
			float fMin = ttc.fFlyHeight, fMax = ttc.fFlyHeight + 0.5;
			int iRet = IsAltitudeInRange(fMin, fMax, pScript);
		
			if(iRet)
			{
				//fout << "%%%%init alti" << endl;
				NvgAltituding((iRet < 0), pScript);
			}
			else
			{
				iState = NVG_FLYTOBEG;
			}

			#ifdef M_COUTDETAIL
			cout << "State: Init altitude." << endl;
			cout << "Range:" << fMin << "~" << fMax << ", Crnt:" << ((iRet < 0) ? "Low" : "High") << endl;
			#endif
			#ifdef M_FOUTDETAIL
			fout << "State: Init altitude." << endl;
			fout << "Range:" << fMin << "~" << fMax << ", Crnt:" << ((iRet < 0) ? "Low" : "High") << endl;
			#endif
		}
		break;

	case NVG_FLYTOBEG:
		{
			static unsigned int uiWaiting = 0;

			if (IsNearDest(tgpsCorner[0], pScript))
			{
				if (uiWaiting > 2)
				{
					uiWaiting = 0;
					iState = NVG_ADJSTALTI;
				}
				else
				{
					uiWaiting++;
				}
			}
			else
			{
				uiWaiting = 0;
				NvgFlyingToBeg(tgpsCorner[0], pScript);
			}

			#ifdef M_COUTDETAIL
			cout << "State: Fly to tgpsCorner[0]." << endl;
			cout << "uiWaiting:" << uiWaiting << endl;
			#endif
			#ifdef M_FOUTDETAIL
			fout << "State: Fly to tgpsCorner[0]." << endl;
			fout << "uiWaiting:" << uiWaiting << endl;
			#endif
		}
		break;
	
	case NVG_ADJSTALTI:
		{
			ttc.fFlyHeight = 1.7;
			
			float fMin = ttc.fFlyHeight, fMax = ttc.fFlyHeight + 0.5;
			int iRet = IsAltitudeInRange(fMin, fMax, pScript);
		
			if(iRet)
			{
				NvgAltituding((iRet < 0), pScript);
			}
			else
			{
				iState = NVG_PREPAREYAW;
			}

			#ifdef M_COUTDETAIL
			cout << "State: Adjust altitude." << endl;
			cout << "Range:" << fMin << "~" << fMax << ", Crnt:" << ((iRet < 0) ? "Low" : "High") << endl;
			#endif
			#ifdef M_FOUTDETAIL
			fout << "State: Adjust altitude." << endl;
			fout << "Range:" << fMin << "~" << fMax << ", Crnt:" << ((iRet < 0) ? "Low" : "High") << endl;
			#endif
		}
		break;
	
	case NVG_NORMAL:
		{
			static unsigned int uiWaiting = 0;

			if (IsNearDest(tgpsNext, pScript))
			{
				if (uiWaiting > 2)
				{
					uiWaiting = 0;
					iState = NVG_PREPAREYAW;
				}
				else
				{
					uiWaiting++;
				}
			}
			else
			{
				uiWaiting = 0;
				NvgFlying(tgpsNext, pScript);
			}

			#ifdef M_COUTDETAIL
			cout << "State: Forward." << endl;
			cout << "uiWaiting:" << uiWaiting << endl;
			#endif
			#ifdef M_FOUTDETAIL
			fout << "State: Forward." << endl;
			fout << "uiWaiting:" << uiWaiting << endl;
			#endif
		}
		break;

	case NVG_PREPAREYAW:
		{
			if (uiGPSCounter >= uiGPSTotal)
			{
				return false; //Navigate finished
			}

			double dOffsetX, dOffsetY;
			TGPS tgpsCrnt; //Theoretical Current GPS

			switch (uiGPSCounter % 4)
			{
			case 0:
				{
					CalcOffset(dOffsetX, dOffsetY, tgpsCorner[0], tgpsCorner[1]);

					tgpsCrnt = CalcGPS(tgpsCorner[0],
						((unsigned int)(uiGPSCounter / 2)) * dStepOffsetX,
						((unsigned int)(uiGPSCounter / 2)) * dStepOffsetY);

					tgpsNext = CalcGPS(tgpsCrnt, dOffsetX, dOffsetY);

					CalcBaseLine(tbl, tgpsCrnt, tgpsNext);
				}
				break;

			case 1:
				{
					dOffsetX = dStepOffsetX;
					dOffsetY = dStepOffsetY;

					tgpsCrnt = CalcGPS(tgpsCorner[1],
						((unsigned int)(uiGPSCounter / 2)) * dStepOffsetX,
						((unsigned int)(uiGPSCounter / 2)) * dStepOffsetY);

					tgpsNext = CalcGPS(tgpsCrnt, dOffsetX, dOffsetY);

					CalcBaseLine(tbl, tgpsCorner[1], tgpsCorner[2]);
				}
				break;

			case 2:
				{
					CalcOffset(dOffsetX, dOffsetY, tgpsCorner[1], tgpsCorner[0]);

					tgpsCrnt = CalcGPS(tgpsCorner[1],
						((unsigned int)(uiGPSCounter / 2)) * dStepOffsetX,
						((unsigned int)(uiGPSCounter / 2)) * dStepOffsetY);

					tgpsNext = CalcGPS(tgpsCrnt, dOffsetX, dOffsetY);

					CalcBaseLine(tbl, tgpsCrnt, tgpsNext);
				}
				break;

			case 3:
				{
					dOffsetX = dStepOffsetX;
					dOffsetY = dStepOffsetY;

					tgpsCrnt = CalcGPS(tgpsCorner[0],
						((unsigned int)(uiGPSCounter / 2)) * dStepOffsetX,
						((unsigned int)(uiGPSCounter / 2)) * dStepOffsetY);

					tgpsNext = CalcGPS(tgpsCrnt, dOffsetX, dOffsetY);

					CalcBaseLine(tbl, tgpsCorner[0], tgpsCorner[3]);
				}
				break;
			}

			//fYaw = CalcYaw(tgpsCrnt, tgpsNext);
			fYaw = CalcYaw(GetCrntGPS(pScript), tgpsNext);
			ttc.fYawAngle = fYaw;

			uiGPSCounter++;

			bOutOfSearchArea = false;

			iState = NVG_YAWING;

			#ifdef M_COUTDETAIL
			cout << "State: Prepare yaw." << endl;
			cout << "OffsetX:" << dOffsetX << endl
				 << "OffsetY:" << dOffsetY << endl;
			cout << "Theoretical Current GPS ->" << endl
				 << "Latitude:  " << tgpsCrnt.dLatitude  << endl
				 << "Longitude: " << tgpsCrnt.dLongitude << endl;
			cout << "Theoretical Next GPS ->" << endl
				 << "Latitude:  " << tgpsNext.dLatitude  << endl
				 << "Longitude: " << tgpsNext.dLongitude << endl;
			cout << "Base Line K:" << tbl.dK << endl;
			cout << "Yaw:" << fYaw << endl;
			#endif
			#ifdef M_FOUTDETAIL
			fout << "State: Prepare yaw." << endl;
			fout << "OffsetX:" << dOffsetX << endl
				 << "OffsetY:" << dOffsetY << endl;
			fout << "Theoretical Current GPS ->" << endl
				 << "Latitude:  " << tgpsCrnt.dLatitude  << endl
				 << "Longitude: " << tgpsCrnt.dLongitude << endl;
			fout << "Theoretical Next GPS ->" << endl
				 << "Latitude:  " << tgpsNext.dLatitude  << endl
				 << "Longitude: " << tgpsNext.dLongitude << endl;
			fout << "Base Line K:" << tbl.dK << endl;
			fout << "Yaw:" << fYaw << endl;
			#endif

		}
		break;

	case NVG_YAWING:
		{
			BroadcastData bd = pScript->getApi()->getBroadcastData();
			float fCrntYaw = atan2(2.0 * (bd.q.q3 * bd.q.q0 + bd.q.q1 * bd.q.q2), 
				-1.0 + 2.0 * (bd.q.q0 * bd.q.q0 + bd.q.q1 * bd.q.q1));
			fCrntYaw = CL_RADTODEG(fCrntYaw);

			if (fabs(fCrntYaw - fYaw) < 3)
			{
				iState = NVG_NORMAL;
				//iState = NVG_BACKTOLINE;
			}
			else
			{
				NvgYawing(fYaw, pScript);
			}

			#ifdef M_COUTDETAIL
			cout << "State: Yawing." << endl;
			cout << "Dest yaw angle:" << fYaw
				 << "\t"
				 << "Crnt yaw angle:" << fCrntYaw
				 << endl;
			#endif
			#ifdef M_FOUTDETAIL
			fout << "State: Yawing." << endl;
			fout << "Dest yaw angle:" << fYaw
				 << "\t"
				 << "Crnt yaw angle:" << fCrntYaw
				 << endl;
			#endif
		}
		break;

	case NVG_OBSTACLE:
		{
			if (1 != ttc.iRequestControl)
			{
				ttc.iCrntCtrl = CTRL_NAVIGATE;
				if (bOutOfSearchArea)
				{
					//ttc.iGuidance = GDC_FORWARD;
					//iState = NVG_OUTOFAREA;
				}
				else
				{
					//ttc.iGuidance = (CalcDistance(tbl, pScript) > 0) ? GDC_RIGHT : GDC_LEFT;//==
					//ttc.iGuidance = (CalcDistance(tbl, pScript) < 0) ? GDC_RIGHT : GDC_LEFT;//==
					if (PianRight == ttc.iPian)
					{
						ttc.iGuidance = GDC_LEFT;
					}
					else if (PianLeft == ttc.iPian)
					{
						ttc.iGuidance = GDC_RIGHT;
					}
					iState = NVG_BACKTOLINE;
				}
			}

			#ifdef M_COUTDETAIL
			cout << "State: Obstacle Avoidance." << endl;
			#endif
			#ifdef M_FOUTDETAIL
			fout << "State: Obstacle Avoidance." << endl;
			#endif
		}
		break;

	case NVG_BACKTOLINE:
		{
			double dDistance = CalcDistance(tbl, pScript);

			if (ttc.iPian) //ttc.iPian != PianNone
			{
				if (fabs(dDistance) < 0.3)
				{
					ttc.iGuidance = GDC_FORWARD;
					iState = NVG_NORMAL;
					ttc.iPian = PianNone;
				}
				else
				{
					if (ttc.bBTLUnsafe)
					{
						iState = NVG_BTLFORWARD;
					}
					else
					{
						//NvgMoving(dDistance > 0 ? NVG_MV_RIGHT : NVG_MV_LEFT, pScript);//==
						//NvgMoving(dDistance < 0 ? NVG_MV_RIGHT : NVG_MV_LEFT, pScript);//==
						if (PianRight == ttc.iPian)
						{
							NvgMoving(NVG_MV_LEFT, pScript);
						}
						else if (PianLeft == ttc.iPian)
						{
							NvgMoving(NVG_MV_RIGHT, pScript);
						}
					}
				}

				#ifdef M_COUTDETAIL
				cout << "State: Back to line." << endl;
				cout << "Distance:" << dDistance << endl;
				#endif
				#ifdef M_FOUTDETAIL
				fout << "State: Back to line." << endl;
				fout << "Distance:" << dDistance << endl;
				#endif
			}
			else
			{
				ttc.iGuidance = GDC_FORWARD;
				iState = NVG_NORMAL;

				#ifdef M_COUTDETAIL
				cout << "State: Back to line." << endl;
				cout << "Adjust altitude, distance:" << dDistance << endl;
				#endif
				#ifdef M_FOUTDETAIL
				fout << "State: Back to line." << endl;
				fout << "Adjust altitude, distance:" << dDistance << endl;
				#endif
			}
		}
		break;

	case NVG_BTLFORWARD:
		{
			static unsigned int uiWaitingBTL = 0;

			if (ttc.bBTLUnsafe)
			{
				NvgMoving(NVG_MV_FORWARD, pScript);
				uiWaitingBTL = 0;
			}
			else
			{
				if (uiWaitingBTL > 50)
				{
					iState = NVG_BACKTOLINE;
					uiWaitingBTL = 0;
				}
				else
				{
					NvgMoving(NVG_MV_FORWARD, pScript);
					uiWaitingBTL++;
				}
				
			}

			if (IsNearDest(tgpsNext, pScript)) //
			{
				ttc.iGuidance = GDC_FORWARD;
				iState = NVG_PREPAREYAW;
				uiWaitingBTL = 0;
			}

			#ifdef M_COUTDETAIL
			cout << "State: BTL forward." << endl;
			cout << "uiWaitingBTL:" << uiWaitingBTL << endl;
			#endif
			#ifdef M_FOUTDETAIL
			fout << "State: BTL forward." << endl;
			fout << "uiWaitingBTL:" << uiWaitingBTL << endl;
			#endif
		}
		break;
	}

	//
	/*if (NVG_NORMAL     == iState ||
		NVG_BACKTOLINE == iState ||
		NVG_BTLFORWARD == iState)
	{
		if (!IsInSearchArea(tgpsCorner, pScript))
		{
			cout << "##OFA" << endl;
			fout << "##OFA" << endl;
			//bOutOfSearchArea = true;
			//iState = NVG_OUTOFAREA;
		}
		else
		{
			cout << "##IA" << endl;
			fout << "##IA" << endl;
		}
	}*/

	#ifdef M_COUTDETAIL
	cout << endl;
	#endif
	#ifdef M_FOUTDETAIL
	fout << endl;
	#endif

	usleep(20000);
	return true;
}

void SearchInArea(/*const */ConboardSDKScript * pScript)
{
	while (Navigate(pScript))
	{
		#ifdef M_ENABLEQUITMSG
		if ('q' == key1)
		{
			ttc.iCrntCtrl = CTRL_HOMEWARD;
			#ifdef M_COUTDETAIL
			cout << "Navigate : Respose Quit message..\n" << endl;
			#endif
			#ifdef M_FOUTDETAIL
			fout << "Navigate : Respose Quit message..\n" << endl;
			#endif
			break;
		}
		#endif

		#ifdef M_LOOKFORAPRILTAGS
		if (LookforApriltags(pScript) >= 5)
		{
			ttc.iCrntCtrl = CTRL_HOMEWARD;
			#ifdef M_COUTDETAIL
			cout << "Navigate : Finded all Apriltags..\n" << endl;
			#endif
			#ifdef M_FOUTDETAIL
			fout << "Navigate : Finded all Apriltags..\n" << endl;
			#endif
			break;
		}
		#endif

		if (iDataFromMobile == 2)
		{
			ttc.iCrntCtrl = CTRL_HOMEWARD;
			#ifdef M_COUTDETAIL
			cout << "Navigate : Give up misson.\n" << endl;
			#endif
			#ifdef M_FOUTDETAIL
			fout << "Navigate : Give up misson.\n" << endl;
			#endif
			break;
		}

		#ifdef M_LOWBATBREAK
		//cout << "Battery:" << GetBattery(pScript) << endl; //
		//fout << "Battery:" << GetBattery(pScript) << endl; //
		if (GetBattery(pScript) < 35)
		{
			ttc.iCrntCtrl = CTRL_HOMEWARD;
			#ifdef M_COUTDETAIL
			cout << "Navigate : Low battery.\n" << endl;
			#endif
			#ifdef M_FOUTDETAIL
			fout << "Navigate : Low battery.\n" << endl;
			#endif
			break;
		}
		#endif
	}

	#ifdef M_COUTDETAIL
	cout << "Navigate : SearchInArea Finished..\n" << endl;
	#endif
	#ifdef M_FOUTDETAIL
	fout << "Navigate : SearchInArea Finished..\n" << endl;
	#endif

	while (RQST_HWFIN != ttc.iRequestControl)
	{
		#ifdef M_ENABLEQUITMSG
		if ('q' == key1)
		{
			ttc.iCrntCtrl = CTRL_HOMEWARD;
			#ifdef M_COUTDETAIL
			cout << "Navigate : (W)Respose Quit message..\n" << endl;
			#endif
			#ifdef M_FOUTDETAIL
			fout << "Navigate : (W)Respose Quit message..\n" << endl;
			#endif
			break;
		}
		#endif
	
		#ifdef M_COUTDETAIL
		cout << "Navigate : Wait for Guidance quit..\n" << endl;
		#endif
		#ifdef M_FOUTDETAIL
		fout << "Navigate : Wait for Guidance quit..\n" << endl;
		#endif
		
		sleep(1);
	}

	return;
}

void GetDataFromMobile(DJI::onboardSDK::CoreAPI * API, Header * pHeader, void * pData)
{
	iDataFromMobile = ((int)(((char*)(pHeader))[14]));

	cout << "Received data from moblie : " << iDataFromMobile << endl << endl;
	fout << "Received data from moblie : " << iDataFromMobile << endl << endl;
}











