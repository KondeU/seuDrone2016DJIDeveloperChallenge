#pragma once

#define CTRL_NAVIGATE 0
#define CTRL_OBSTACLE 1
#define CTRL_HOMEWARD -1

#define RQST_HWFIN -1

#define PianNone  0
#define PianRight 1
#define PianLeft  2

struct TThreadController
{
	int iCrntCtrl;//navi       //Current controller
	int iRequestControl; //ob  //Obstacle Avoidance request control
	int iPian;

	float fFlyHeight;//ob
	float fYawAngle;//navi
	float fNvgSpeed;//ob

	int  iGuidance;//navi
	bool bBTLUnsafe;//ob

	bool bUltrasonicLeft;  //ob
	bool bUltrasonicRight; //ob
	
};

