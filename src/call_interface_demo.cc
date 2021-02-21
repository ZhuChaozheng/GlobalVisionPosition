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
	Point2f target_point = Point2f(100, 100);
	float target_speed = 30;
	float target_slope = 47;
	int stop_flag = 0;
	Car car;
	car.set_target_point(target_point);
	car.set_target_speed(target_speed);
	car.set_target_slope(target_slope);
	car.set_stop_flag(stop_flag);
	vector<Car> cars_control_set;
	for(int i = 0; i <= 9; i ++)
		cars_control_set.push_back(car);
	cout << cars_control_set.size() << endl;
	hardware_control_interface(cars_control_set);
	return 0;
}