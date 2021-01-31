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
#include "kalman.h"

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
	float slope_p_;
	float slope_i_;
	float slope_d_;
	float speed_p_;
	float speed_i_;
	float speed_d_;
	string ip_;
	int port_;
	Kalman filter_;
	float speed_error1_ = 0;
	float speed_error2_ = 0;
	float speed_output_ = 0;
	float last_error_;
    float integrator_;
    float last_derivative_;
    time_t last_tim_;

public:
	Car(int marker, float slope_P, float slope_I, 
		float slope_D, float speed_P, float speed_I, 
		float speed_D, string ip, int port, double target_slope,
		float target_speed);
	Car();

	void update_parameters(float slope_P, float slope_I, 
			float slope_D, float speed_P, float speed_I, 
			float speed_D, string ip, double target_slope,
			float target_speed);

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
	void set_slope_p(float P) { slope_p_ = P; }
	void set_slope_i(float I) { slope_i_ = I; }
	void set_slope_d(float D) { slope_d_ = D; }
	void set_speed_p(float P) { speed_p_ = P; }
	void set_speed_i(float I) { speed_i_ = I; }
	void set_speed_d(float D) { speed_d_ = D; }
	void set_ip(string ip) { ip_ = ip; }
	void set_speed_error1(float error1) 
			{ speed_error1_ = error1; }
	void set_speed_error2(float error2) 
			{ speed_error2_ = error2; }
	void set_speed_output(float output) 
			{ speed_output_ = output; }
	void set_last_error(float last_error) 
			{ last_error_ = last_error; }
	void set_integrator(float integrator) 
			{ integrator_ = integrator; }
	void set_last_derivative(float last_derivative) 
			{ last_derivative_ = last_derivative; }
    void set_last_tim(time_t last_tim) 
    		{ last_tim_ = last_tim; }


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
	float get_slope_p() { return slope_p_; }
	float get_slope_i() { return slope_i_; }
	float get_slope_d() { return slope_d_; }
	float get_speed_p() { return speed_p_; }
	float get_speed_i() { return speed_i_; }
	float get_speed_d() { return speed_d_; }
	float get_speed_error1() { return speed_error1_; }
	float get_speed_error2() { return speed_error2_; }
	float get_speed_output() { return speed_output_; }
	float get_last_error() { return last_error_; }
	float get_integrator() { return integrator_; }
	float get_last_derivative() { return last_derivative_; }
    time_t get_last_tim() { return last_tim_; }
};

#endif //CAR_H
