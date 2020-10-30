#ifndef CTRL_TEST_H_
#define CTRL_TEST_H_

#include <iostream>
#include <time.h>
#include <sys/time.h>
#include <math.h>

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

        void set_pid(float P, float I, float D)
        {
            _p = P;
            _i = I;
            _d = D;
        }

        float get_pid(float error, float scalar){
            // cout << _p << " hhh " << _i << " " << _d << endl;
            //Get now time
            struct timeval tv;
	        gettimeofday( &tv, NULL );
	        time_t tnow = tv.tv_sec * 1000 + tv.tv_usec / 1000;
            
            int dt = tnow - _last_tim; // on ms
            float output = 0;
            
            if( _last_tim == 0 || dt > 1000 ) {
                dt = 0;
                _integrator = 0;
                _last_derivative = 0;
            }
            
            _last_tim = tnow;
            float delta_time = dt / 1000; // on s
            output += error * _p;
            if ( abs(_d) > 0 && dt > 0 ) {
                float derivative = 0;
                if ( _last_derivative == 0 ){
                    derivative = 0;
                    _last_derivative = 0;
                }
                else{
                    derivative = (error - _last_error) / delta_time;
                }
                derivative = _last_derivative + (
                                (delta_time) / (_rc + delta_time) * (derivative - _last_derivative)
                            );
                _last_error = error;
                _last_derivative = derivative;
                output += _d * derivative; 
            }
            output *= scalar;
            if( abs(_i) > 0 && dt > 0 ) {
                _integrator += ( error * _i ) * scalar * delta_time;
                if( _integrator < -_i_max )
                    _integrator = -_i_max;
                else if( _integrator > _i_max )
                    _integrator = _i_max;
                output += _integrator;
            }
            return output;
        }
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
};

#endif
