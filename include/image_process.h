/**
 * created by Jason 
 * 2020.6.13
 * fetch the colorful blocks from image
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

#define PI 3.14159265

// define the bound of base
#define MAX_DISTANCE 29.3
#define MIN_DISTANCE 24.3
#define MINIMUM 1


using namespace cv;
using namespace std;

class PointAttri
{
public:
	Point2f point;
	int value;
};

class Car
{
public:
	int marker_; // marker
	Point medianPoint; // medianPoint
	Point2f first; // vertex
	Point2f second; // one point of bottom line
	Point2f third; // the other point of bottom line
	double slope; // slope
	Point3f accelerate; // accelerate, x, y
	double angularVelocity;// angular velocity, yaw
	vector<Point2f> pointSet; 
	Point3f target; // target Point
	float p_;
	float i_;
	float d_;
	string ip_;
	int port_;

public:
	Car(int marker, float P, float I, float D, 
			string ip, int port);
	Car();
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

int thresh = 120; // gray
// int thresh = 255; // colour
RNG rng(12345);
Mat src_gray;
Mat cimage;

bool cmp(const Point2f a, const Point2f b);
void thresh_callback(const Mat& src_gray, 
		vector<Point2f>& newPointSet);
bool exist(Car& car, vector<Car>& carStateSet, Car& lastCar);
void refine_point_set(const vector<Point2f>& pointSet, 
		vector<Point2f>& newPointSet);
bool neighbourPoint(Point2f pointA, Point2f pointB);
void classificationCar(vector<Point2f>* pointSet, 
		vector<Car>& carSet);
double getPixelDistance(Point2f pointA, Point2f pointB);
void getCarKeyAttribution(Car& car);
int findKeyPoint(Point2f& point, Point2f& tempPoint, 
		vector<Point2f>& pointSet);

void determineTriangleVertex(vector<Point2f>& 
		pointSet, Car& car);
double getAbsoluteOrientation(vector<Point2f>& 
		pointSet, Car& car);
double getSlope(Point first, Point second);

double get3dSlope(Point3f first, Point3f second);

void deleteCar(Car& car, vector<Car>& carStateSet);

#endif //IMAGE_PROCESS_H
