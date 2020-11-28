#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctype.h>
using namespace cv;
using namespace std;

Mat cameraMatrix = Mat::eye(3, 3, 
		cv::DataType<double>::type);  //intrinsic of camera
Mat rvecM1 = Mat::eye(3, 3, 
		cv::DataType<double>::type);  //rotation matrix
Mat tvec1 = Mat::eye(3, 1, 
		cv::DataType<double>::type);  // translation vector
double s = 203067;


void pointToWorld(const Point& point, Point3f& worldPoint,
                  const Mat& rvecM1, const Mat& tvec1,
                  const Mat& cameraMatrix, const double& s);
void ConfigFileRead(Mat rvecM1, Mat tvec1, Mat cameraMatrix, double s);
 
void ConfigFileWrite();