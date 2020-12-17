/**
 * created by Jason 
 * 2020.12.15
 * class car
 */

#ifndef CAR_H
#define CAR_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>

using namespace cv;
using namespace std;


class Car
{
public:
	int marker_; // marker
	Point2f medianPoint_; // medianPoint
	Point2f first_; // vertex
	Point2f second_; // one point of bottom line
	Point2f third_; // the other point of bottom line
	double slope_; // slope
	float speed_; // accumulated x y
	double angularVelocity_;// angular velocity, yaw
	vector<Point2f> pointSet; 
	Point3f target_; // target Point
	double target_slope_; // target slope
	float target_speed_; // target speed
	float p_;
	float i_;
	float d_;
	string ip_;
	int port_;

public:
	Car(int marker, float P, float I, float D, 
			string ip, int port, double target_slope,
			float target_speed);
	Car();

	void update_parameters(float P, float I, float D, 
			string ip);

	void set_median_point(Point2f center) { medianPoint_ = center; }
	void set_first(Point2f first) { first_ = first; }
	void set_second(Point2f second) { second_ = second; }
	void set_third(Point2f third) { third_ = third; }
	void set_slope(double slope) { slope_ = slope; }
	void set_speed(float speed) { speed_ = speed; }
	void set_target_slope(double target_slope) 
			{ target_slope_ = target_slope; }
	void set_target_speed(float target_speed) 
			{ target_speed_ = target_speed; }
	void set_p(float P) { p_ = P; }
	void set_i(float I) { i_ = I; }
	void set_d(float D) { d_ = D; }
	void set_ip(string ip) { ip_ = ip; }

	int get_marker() { return marker_; }
	string get_ip() { return ip_; }
	Point2f get_median_point() { return medianPoint_; }
	Point2f get_first() { return first_; }
	double get_slope() { return slope_; }
	float get_speed() { return speed_; }
	vector<Point2f> get_point_set() { return pointSet; } 
	Point3f get_target() { return target_; }
	double get_target_slope() { return target_slope_; }
	float get_target_speed() { return target_speed_; }
	float get_p() { return p_; }
	float get_i() { return i_; }
	float get_d() { return d_; }
};

#endif //CAR_H
