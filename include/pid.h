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
    pid(float P = 0, float I = 0, float D = 0, float i_max = 0 )
    {
        _p = P;
        _i = I;
        _d = D;
        _i_max = abs(i_max);
        _last_error = _integrator = _last_tim = 0;
        _last_derivative = 0;
        _rc = 1/(2* pi* 20);
    }

    void set_pid(float P, float I, float D) { _p = P;
            _i = I; _d = D; }

    float get_pid(float error, float scalar);

    void controlSpeedAndAngular(Car &car);
    
private:
    float _p;
    float _i;
    float _d;
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
