#include "pid.h"

float pid::get_slope_pid(float error, float scalar){
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
    output += error * slope_p;
    if ( abs(slope_d) > 0 && dt > 0 ) {
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
        output += slope_d * derivative; 
    }
    output *= scalar;

    if( abs(slope_i) > 0 && dt > 0 ) {
        _integrator += ( error * slope_i ) * scalar * delta_time;

        if( _integrator < -400 ) 
            _integrator = -400;
        else if( _integrator > 400 )
            _integrator = 400;

        output += _integrator;
    }
    return output;
}

float pid::get_speed_pid(float error, float scalar){
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
    output += error * speed_p;
    cout << "speed_p: " << speed_p << endl;
    cout << "output1: " << output << endl;
    if ( abs(speed_d) > 0 && dt > 0 ) {
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
        output += speed_d * derivative; 
    }
    output *= scalar;

    if( abs(speed_i) > 0 && dt > 0 ) {
        _integrator += ( error * speed_i ) * scalar * delta_time;

        if( _integrator < -400 ) 
            _integrator = -400;
        else if( _integrator > 400 )
            _integrator = 400;

        output += _integrator;
    }
    output = abs(output);
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
    slope_p = car.get_slope_p();
    slope_i = car.get_slope_i();
    slope_d = car.get_slope_d();
    // update parameter
    pid pid_turn;
    pid_turn.set_slope_pid(slope_p, slope_i, slope_d);
 //    if (abs(error_angel) > 5)
 //        pid_turn.set_pid(param_turn_p - 0.4, param_turn_i, 
 //           param_turn_d + 0.02);    
    wp_angel = pid_turn.get_slope_pid(error_angel, 100);

    
 //    if (abs(error_angel) > 5)
 //        pid_turn.set_pid(param_turn_p - 0.4, param_turn_i, 
 //           param_turn_d + 0.02);    
    // wp_move = pid_turn.get_speed_pid(error_angel, 100);


    if (wp_angel > 2000)
        wp_angel = 2000;
    else if (wp_angel < -2000)
        wp_angel = -2000;
    // cout << "wp_angel: " << wp_angel << endl;
    //float error_move = (test_point.x * worldPoint.x + test_point.y * worldPoint.y)
    //              / sqrt(pow(worldPoint.x,2) + pow(worldPoint.y,2));
    float target_speed = car.get_target_speed();
    float currentSpeed = car.get_speed();
    float error_move=target_speed-currentSpeed;
    cout << "target_speed: " << target_speed << endl;
    cout << "currentSpeed: " << currentSpeed << endl;
    // Normalized on specifical value, here is 1m
    error_move /= 1000;
    error_move = (abs(error_move) > 1 )? 1:error_move;
    
    // speed pid
    speed_p = car.get_speed_p();
    speed_i = car.get_speed_i();
    speed_d = car.get_speed_d();

    pid pid_move;   
    pid_move.set_speed_pid(speed_p, speed_i, speed_d);
    cout << "error_move: " << error_move << endl;
    wp_move = pid_move.get_speed_pid(error_move, 100);
    cout << "wp_move: " << wp_move << endl;
    //wp_angel = 0;
   // wp_move = 0;   
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