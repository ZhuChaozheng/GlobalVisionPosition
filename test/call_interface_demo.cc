#include "control_interface.h"

int main()
{	
	/*
	 * Car:
	 *     Point2f target_point, 
	 *	   float target_speed, 
	 *	   float target_slope, 
	 *	   int stop_flag 
	 */
	Point2f target_point = Point2f(1000.21, 100);
	float target_speed = 30;
	float target_slope = 47;
	float stop_flag = 0;
	Car car;
	car.set_target_point(target_point);
	car.set_target_speed(target_speed);
	car.set_target_slope(target_slope);
	car.set_stop_flag(stop_flag);
	vector<Car> cars_control_set;
	for(int i = 0; i <= 9; i ++)
		cars_control_set.push_back(car);
	cout << cars_control_set.size() << endl;
	interface_udp(cars_control_set);
	// vector<float> float_set;
	// float_set.push_back(target_point.x);
	// float_set.push_back(target_point.y);
	// float_set.push_back(target_speed);
	// float_set.push_back(target_slope);
	// float_set.push_back(stop_flag);
	// float_set.push_back(target_point.x);
	// float_set.push_back(target_point.y);
	// float_set.push_back(target_speed);
	// float_set.push_back(target_slope);
	// float_set.push_back(stop_flag);
	// float_set.push_back(target_point.x);
	// float_set.push_back(target_point.y);
	// float_set.push_back(target_speed);
	// float_set.push_back(target_slope);
	// float_set.push_back(stop_flag);
	// float_set.push_back(target_point.x);
	// float_set.push_back(target_point.y);
	// float_set.push_back(target_speed);
	// float_set.push_back(target_slope);
	// float_set.push_back(stop_flag);
	
	// unsigned char a = (unsigned char)(target_point.x);
	// char src[80];
	// char *temp;
	// int i = 0;
	// for(auto iter = float_set.begin(); iter != float_set.end();)
	// {		
	// 	temp = (char*)(&(*iter));
	// 	src[i] = temp[0];
	// 	src[i + 1] = temp[1];
	// 	src[i + 2] = temp[2];
	// 	src[i + 3] = temp[3];
	// 	i = i + 4;
	// 	iter ++;
	// }
	// float *w;
	// char2float(src, w);  
	return 0;
}