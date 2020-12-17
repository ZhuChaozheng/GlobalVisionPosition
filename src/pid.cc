#include "pid.h"

float pid::get_pid(float error, float scalar){
    //Get now time
    struct timeval tv;
    gettimeofday( &tv, NULL );
    time_t tnow = tv.tv_sec * 1000 + tv.tv_usec / 1000;
//         if( _flag==0)
//          {
//            _flag=1;
//           _last_tim=tnow;
//           }
    int dt = tnow - _last_tim; // on ms
    float output = 0;
    
    if( _last_tim == 0 || dt > 1000 ) {
        dt = 0;
        _integrator = 0;
        _last_derivative = 0;
    }
    
    _last_tim = tnow;
    float delta_time = dt / 1000.0; // on s
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

/**
 *
 * control car through targets of speed and angular
 * input: target speed, angular, current speed, 
 * angular, car
 * 
**/
void pid::controlSpeedAndAngular(Car &car)
{
    double target_slope = car.get_target_slope();
    if (target_slope > 180)
    {
        target_slope = target_slope-360;
    }
    double current_slope = car.get_slope();
    double error_angel = current_slope - target_slope;
     
    if( error_angel < -180)
        error_angel = error_angel+360;
    else if( error_angel > 180)
        error_angel = error_angel-360;
    _p = car.get_p();
    _i = car.get_i();
    _d = car.get_d();
    // update parameter
    pid pid_turn;
    pid_turn.set_pid(_p, _i, _d);
 //    if (abs(error_angel) > 5)
 //        pid_turn.set_pid(param_turn_p - 0.4, param_turn_i, 
 //           param_turn_d + 0.02);    
    wp_angel = pid_turn.get_pid(error_angel, 100);
    if (wp_angel > 2000)
        wp_angel = 2000;
    else if (wp_angel < -2000)
        wp_angel = -2000;
    // cout << "wp_angel: " << wp_angel << endl;
    //float error_move = (test_point.x * worldPoint.x + test_point.y * worldPoint.y)
    //              / sqrt(pow(worldPoint.x,2) + pow(worldPoint.y,2));
    float target_speed = car.get_target_speed();
 //   float error_move = sqrt(pow(target_speed.x - 
 //           currentSpeed.x, 2) + pow(target_speed.y - 
 //           currentSpeed.y, 2));
  //  error_move=target_speed-currentSpeed;

    // Normalized on specifical value, here is 1m
    //error_move /= 1000;
   // error_move = (abs(error_move) > 1 )? 1:error_move;
    // Point3f speed_move = target_speed - currentSpeed;
    
  //  wp_move = pid_move.get_pid(error_move, 100);
    //wp_angel = 0;
    wp_move = 0;   
    float duty_left = -wp_angel/2 + wp_move; 
    duty_left *= -1;
    float duty_right = wp_angel/2 + wp_move;

    /* define comm format*/
    char a[6] = {0x11,0x00,0x00,0x00,0x00,0x22};
    // combinate the corresponding command
    a[1] = (unsigned char)((((short)duty_left)>>8) & 0xff);
    a[2] = (unsigned char)(((short)duty_left) & 0xff);
    a[3] = (unsigned char)((((short)duty_right)>>8) & 0xff);
    a[4] = (unsigned char)(((short)duty_right) & 0xff);
    // build communication based on udp
    string ip = car.get_ip();
    udp udp_comm;
    int sock_fd = udp_comm.udp_init(ip);
    // send data through udp
    udp_comm.send_data(sock_fd, a);
}