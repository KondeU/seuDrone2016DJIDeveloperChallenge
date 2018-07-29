#include <vector>
#include <iostream>
#include <cmath>
#include "opencv2/opencv.hpp"
#include "getcor.h"
using namespace std;

extern "C" april_res print_detection(double p[4][2], int size, double aprilsize);

cv::Mat getRelativeTransform(double tag_size, double fx, double fy, double px, double py,double p[4][2]){
  std::vector<cv::Point3f> objPts;
  std::vector<cv::Point2f> imgPts;
  double s = tag_size/2.;
  objPts.push_back(cv::Point3f(-s,-s, 0));
  objPts.push_back(cv::Point3f( s,-s, 0));
  objPts.push_back(cv::Point3f( s, s, 0));
  objPts.push_back(cv::Point3f(-s, s, 0));

 /* std::pair<float, float> p1 = p[0];     //改一下
  std::pair<float, float> p2 = p[1];
  std::pair<float, float> p3 = p[2];
  std::pair<float, float> p4 = p[3];*/
  imgPts.push_back(cv::Point2f(p[0][1], p[0][0]));
  imgPts.push_back(cv::Point2f(p[3][1], p[3][0]));
  imgPts.push_back(cv::Point2f(p[2][1], p[2][0]));
  imgPts.push_back(cv::Point2f(p[1][1], p[1][0]));

  cv::Mat rvec, tvec;
  cv::Matx33f cameraMatrix(
                           fx, 0, px,
                           0, fy, py,
                           0,  0,  1);
  cv::Vec4f distParam(0,0,0,0); // all 0?
  cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
  cv::Matx33d r;
  cv::Rodrigues(rvec, r);
  //cv::Mat wRo=r;
  
  cv::Mat T(4,4,CV_64F,cv::Scalar(0));
  for(int i=0;i<3;i++){
	  for(int j=0;j<3;j++){
	  T.at<double>(i,j)= r(i,j); 
	  }  
  }

   T.at<double>(0,3)=tvec.at<double>(0);
   T.at<double>(1,3)=tvec.at<double>(1);
   T.at<double>(2,3)=tvec.at<double>(2);
	
   T.at<double>(3,0)=0;
   T.at<double>(3,1)=0;
   T.at<double>(3,2)=0;
   T.at<double>(3,3)=1;
  
   return T;
}


void getRelativeTranslationRotation(double tag_size, double fx, double fy, double px, double py,cv::Mat& trans, cv::Mat& rot,double p[4][2]){
  cv::Mat T =
    getRelativeTransform(tag_size, fx, fy, px, py,p);
  
  // converting from camera frame (z forward, x right, y down) to
  // object frame (x forward, y left, z up)
  
  cv::Mat M(4,4,CV_64F,cv::Scalar(0));
  M.at<double>(0,2)=1;
  M.at<double>(1,0)=-1;
  M.at<double>(2,1)=-1;
  M.at<double>(3,3)=1;
  
  cv::Mat MT = M*T;
  // translation vector from camera to the April tag
  trans.at<double>(0)=MT.at<double>(0,3);
  trans.at<double>(1)=MT.at<double>(1,3);
  trans.at<double>(2)=MT.at<double>(2,3);
  
  for(int i=0;i<3;i++){
	  for(int j=0;j<3;j++){
	  rot.at<double>(i,j)= T.at<double>(i,j); 
	  }  
  }
  // orientation of April tag with respect to camera: the camera
  // convention makes more sense here, because yaw,pitch,roll then
  // naturally agree with the orientation of the object
}

const double PI = 3.14159265358979323846;
const double TWOPI = 2.0*PI;

/*inline */double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}
void wRo_to_euler(const cv::Mat& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo.at<double>(1,0), wRo.at<double>(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo.at<double>(2,0), wRo.at<double>(0,0)*c + wRo.at<double>(1,0)*s));
    roll  = standardRad(atan2(wRo.at<double>(0,2)*s - wRo.at<double>(1,2)*c, -wRo.at<double>(0,1)*s + wRo.at<double>(1,1)*c));
}

april_res print_detection(double p[4][2], int size, double aprilsize)
{
	cv::Mat translation(3,1,CV_64F,cv::Scalar(0));
	cv::Mat rotation(3,3,CV_64F,cv::Scalar(0));

	if (size == 1) {       //1为size=640*360 
		getRelativeTranslationRotation(aprilsize / 100, 391.31443,  391.55329, 176.30369, 327.03717, translation, rotation, p);
		//getRelativeTranslationRotation(aprilsize / 100, 376.131303, 375.875133, 177.714067, 326.928107, translation, rotation, p);		
		//getRelativeTranslationRotation(aprilsize / 100, 367.71485, 375.04881, 183.11699, 323.94710, translation, rotation, p);
	}
	else if (size == 2) { //2为size = 320*180 
		getRelativeTranslationRotation(aprilsize / 100, 195.65722, 195.77664,88.151845, 163.51858, translation, rotation, p);
		//getRelativeTranslationRotation(aprilsize / 100, 188.06565, 187.937567, 88.857033, 163.464053, translation, rotation, p);
		//getRelativeTranslationRotation(aprilsize / 100, 186.53985, 193.14345, 86.77146, 166.85657, translation, rotation, p);
	}
   
											 
	cv::Mat F(3,3,CV_64F,cv::Scalar(0));
	F.at<double>(0,0)=1;
	F.at<double>(1,1)=-1;
	F.at<double>(2,2)=1;
	
    cv::Mat fixed_rot = F*rotation;
    double yaw, pitch, roll;
    wRo_to_euler(fixed_rot, yaw, pitch, roll);

	april_res april_result;
	april_result.x=translation.at<double>(0);
	april_result.y=-translation.at<double>(2);
	april_result.z=-translation.at<double>(1);
	april_result.yaw=yaw/3.14159265357*180;
	april_result.pitch=pitch/3.14159265357*180;
	april_result.roll=roll/3.14159265357*180;
	return april_result;
  }
