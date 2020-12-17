#include "car.h"

Car::Car(int marker, float P, float I, float D, 
			string ip, int port, double target_slope,
			float target_speed)
{ 
	marker_ = marker; p_ = P; 
	i_ = I; d_ = D; ip_ = ip; 
	port_ = port;
	target_slope_ = target_slope;
	target_speed_ = target_speed;
}

Car::Car() {}

void Car::update_parameters(float P, float I, float D, 
		string ip)
{
	set_p(P);
	set_i(I);
	set_d(D);
	set_ip(ip);
}