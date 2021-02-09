/**
 * created by Jason 
 * 2020.6.13
 * fetch the blocks from gray gimage
 */

#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h> // atan
#include "car.h"

#define PI 3.14159265

// define the bound of base
#define MAX_DISTANCE 29.3
#define MIN_DISTANCE 24.3
#define MINIMUM 1

enum feature {SIDE, BOTTOM};

using namespace cv;
using namespace std;

class PointAttri
{
public:
	Point2f point;
	int value;
};

// define graph
typedef Point2f Vertex; // vertex is point type
typedef double WeightValue; // weight value

class EdgeNode
{
public:
	Vertex V1, V2; // full undirected graph
	WeightValue Weight;
};

class point {
        public:
        point(float a, float b):first(a), second(b){}
        float first;
        float second;
        bool operator < (const point &m)const {
                return first < m.first;
        }
};

bool less_second(const point & m1, const point & m2) {
	if (m1.first == m2.first)
    	return m1.second < m2.second;
	else
		return m1.first < m2.first;
}

int thresh = 30; // gray
// int thresh = 255; // colour
bool init_slope_flag = true;
RNG rng(12345);
Mat src_gray;
Mat cimage;

bool Cmp(const Point2f a, const Point2f b);
void ThreshCallBack(const Mat &src_gray, 
		vector<Point2f> &pointSet);
float GetCross(Point2f &p1, Point2f &p2, Point2f &p);
bool IsPointInRotatedRect(RotatedRect &rotated_rect, Point2f &p);
// bool PointNeighbourVector(const Point2f currentPoint, 
// 		vector<Point2f>	double_dup_point_set);
bool Exist(Car& car, vector<Car>& carStateSet, Car& lastCar);
bool NeighbourPoint(Point2f pointA, Point2f pointB);
void ClassificationCar(vector<Point2f> *pointSet, 
		vector<Car> &carSet);
double GetPixelDistance(Point2f pointA, Point2f pointB);
void GetCarKeyAttribution(Car& car);
int FindKeyPoint(Point2f& point, Point2f& tempPoint, 
		vector<Point2f>& pointSet);

void DetermineTriangleVertex(vector<Point2f>& 
		pointSet, Car& car);
double GetAbsoluteOrientation(vector<Point2f>& 
		pointSet, Car& car);
double GetSlope(Point2f first, Point2f second);

double Get3dSlope(Point3f first, Point3f second);

void DeleteCar(Car& car, vector<Car>& carStateSet);

#endif //IMAGE_PROCESS_H
