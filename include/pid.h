#ifndef CTRL_TEST_H_
#define CTRL_TEST_H_

#include <iostream>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include "car.h"
#include "udp.h"

using namespace std;

#define pi acos(-1)

class pid
{
public:
    pid(float slope_P = 0, float slope_I = 0, 
            float slope_D = 0, float speed_P = 0, 
            float speed_I = 0, float speed_D = 0, 
            float i_max = 0 )
    {
        slope_p = slope_P;
        slope_i = slope_I;
        slope_d = slope_D;
        speed_p = speed_P;
        speed_i = speed_I;
        speed_d = speed_D;
        _i_max = abs(i_max);
        _last_error = _integrator = _last_tim = 0;
        _last_derivative = 0;
        _rc = 1/(2* pi* 20);
    }

    void set_slope_pid(float slope_P, float slope_I, 
            float slope_D) { slope_p = slope_P; 
            slope_i = slope_I; slope_d = slope_D; }

    float get_slope_pid(float error, float scalar);

    void set_speed_pid(float speed_P, float speed_I, 
            float speed_D) { speed_p = speed_P; 
            speed_i = speed_I; speed_d = speed_D; }

    float get_speed_pid(float error, float scalar);

    void controlSpeedAndAngular(Car &car);
    
private:
    float slope_p;
    float slope_i;
    float slope_d;
    float speed_p;
    float speed_i;
    float speed_d;
    float _i_max;
    float _last_error;
    float _integrator;
    float _last_derivative;
    time_t _last_tim;
    float _rc;
    int _flag=0;
    double wp_angel;
    double wp_move;
};

#endif
