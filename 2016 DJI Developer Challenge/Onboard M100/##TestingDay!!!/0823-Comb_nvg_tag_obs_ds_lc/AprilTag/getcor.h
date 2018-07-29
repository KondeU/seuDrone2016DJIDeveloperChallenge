typedef struct april_res
{

	int ID;
	double x,y,z,yaw,pitch,roll;
	double c[2];
	double corner[4][2];
}april_res;

typedef struct april_res_all
{
	int num;
	april_res april_result[20];
}april_res_all;
