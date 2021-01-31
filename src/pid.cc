#include "pid.h"

float pid::get_slope_pid(float error, Car &car, float scalar){
    
    //Get now time
    struct timeval tv;
    gettimeofday( &tv, NULL );
    time_t tnow = tv.tv_sec * 1000 + tv.tv_usec / 1000;
//         if( _flag==0)
//          {
//            _flag=1;
//           _last_tim=tnow;
//           }
    time_t _last_tim = car.get_last_tim();
    int dt = tnow - _last_tim; // on ms
    float output = 0;
    
    float _last_derivative = car.get_last_derivative();
    if( _last_tim == 0 || dt > 1000 ) {
        dt = 0;
        // _integrator = 0;
        car.set_integrator(0);
        // _last_derivative = 0;
        car.set_last_derivative(0);
    }
    
    // _last_tim = tnow;
    car.set_last_tim(tnow);
    float delta_time = dt / 1000.0; // on s
    output += error * slope_p;
    _last_derivative = car.get_last_derivative();
    float _last_error = car.get_last_error();
    
    if ( abs(slope_d) > 0 && dt > 0 ) {
        float derivative = 0;
        if ( _last_derivative == 0 ){
            derivative = 0;
            // _last_derivative = 0;
            car.set_last_derivative(0);
        }
        else{
            derivative = (error - _last_error) / delta_time;
        }
        _last_derivative = car.get_last_derivative();
        derivative = _last_derivative + (
                        (delta_time) / (_rc + delta_time) * 
                        (derivative - _last_derivative)
                    );
        // _last_error = error;
        // cout << "error: " << error << endl;
        car.set_last_error(error);
        _last_error = car.get_last_error();
        // cout << "52_last_error: " << _last_error << endl;

        // _last_derivative = derivative;
        car.set_last_derivative(derivative);
        output += slope_d * derivative; 
    }
    output *= scalar;
    float _integrator = car.get_integrator();
    if( abs(slope_i) > 0 && dt > 0 ) {
        // _integrator = _integrator + error;
        _integrator += ( error * slope_i ) * 
                scalar * delta_time;
        cout<<"_integrator:"<<_integrator<<endl;
        if( _integrator < -500 ) 
            _integrator = -500;
        else if( _integrator > 500 )
            _integrator = 500;
        output += _integrator;
        cout << "output: " << output << endl;
        car.set_integrator(_integrator);
    }
    // _integrator = car.get_integrator();
    // cout << "_integrator: " << _integrator << endl;
    // _last_error = car.get_last_error();
    // cout << "_last_error: " << _last_error << endl;
    // _last_derivative = car.get_last_derivative();
    // cout << "_last_derivative: " << _last_derivative << endl;
    // _last_tim = car.get_last_tim();
    // cout << "_last_tim: " << _last_tim << endl;
    return output;
}

float pid::get_speed_pid_incre(float error, Car &car, float scalar)
{
    //   PID计算输出增量
    float Speed_error1 = car.get_speed_error1();
    float Speed_error2 = car.get_speed_error2();
    float Speed_i = car.get_speed_i();
    // cout << "Speed_i: " << Speed_i << endl;

    float Speed_output_increment = (speed_p * 
            (error - Speed_error1) + speed_i * 
            error + speed_d * (error + Speed_error2 - 
            2 * Speed_error1)) * scalar;
    // cout << "speed_p: " << speed_p << endl;
    // cout << "speed_i: " << speed_i << endl;
    // cout << "speed_d: " << speed_d << endl;
    // cout << "Speed_error1: " << Speed_error1 << endl;
    // cout << "Speed_error2: " << Speed_error2 << endl;
    // cout << "error: " << error << endl;
    car.set_speed_error2(Speed_error1);
    car.set_speed_error1(error);
    // Speed_error2 = Speed_error1;
    Speed_error1 = car.get_speed_error1();
    Speed_error2 = car.get_speed_error2();
    // cout << "Speed_error1: " << Speed_error1 << endl;
    // cout << "Speed_error2: " << Speed_error2 << endl;  
    // Speed_error1 = error;
    // cout << "Speed_output_increment: " << Speed_output_increment << endl; 
    
    //输出速度增量限幅，防止速度过快出现过冲等现象，具体的speed_output_limit需要根据实际调节
//   if( Speed_output_increment < -speed_output_incre_limit)
//      Speed_output_increment = -speed_output_incre_limit;
//   else if(Speed_output_increment > speed_output_incre_limit)
//       Speed_output_increment = speed_output_incre_limit;
    
    //更新本次需要输出的速度量
    float Speed_output = car.get_speed_output();
    // cout << "Speed_output: " << Speed_output << endl;
    Speed_output = Speed_output + Speed_output_increment;
    
    //对输出的速度进行限幅，防止越限
    // if   (Speed_output < -speed_output_limit)
    //     Speed_output = -speed_output_limit;
    // else if(Speed_output > speed_output_limit)
    //     Speed_output = speed_output_limit;
    car.set_speed_output(Speed_output);
    Speed_output = car.get_speed_output();
    // cout << "Speed_output: " << Speed_output << endl;
    
    return Speed_output;    
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
    cout << "slope_p: " << slope_p << endl;
    cout << "slope_i: " << slope_i << endl;
    cout << "slope_d: " << slope_d << endl;
    // update parameter
    pid pid_turn;
    pid_turn.set_slope_pid(slope_p, slope_i, slope_d);
 //    if (abs(error_angel) > 5)
 //        pid_turn.set_pid(param_turn_p - 0.4, param_turn_i, 
 //           param_turn_d + 0.02);   
    cout << "error_angel: " << error_angel << endl; 
    wp_angel = pid_turn.get_slope_pid(error_angel, car, 100);

    
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
    // cout << "currentSpeed: " << currentSpeed << endl;
    // Normalized on specifical value, here is 1m
    error_move /= 1000;
    error_move = (abs(error_move) > 1 )? 1:error_move;
    
    // speed pid
    speed_p = car.get_speed_p();
    speed_i = car.get_speed_i();
    speed_d = car.get_speed_d();

    pid pid_move;   
    pid_move.set_speed_pid(speed_p, speed_i, speed_d);
    // cout << "error_move: " << error_move << endl;
    wp_move = pid_move.get_speed_pid_incre(error_move, 
            car, 100);
    // cout << "wp_move: " << wp_move << endl;
    //wp_angel = 0;
   // wp_move = 0;   
    float duty_left = -wp_angel/2 + wp_move; 
    duty_left *= -1;
    float duty_right = wp_angel/2 + wp_move;

    /*
//防止输出越限，在左右输出减去相同的量。即保证转向角度控制，弱化速度控制。代码需测试 
//    if(duty_left<-900.0)
 //   {
 //       duty_left=-900.0;
        duty_right=duty_right-900-duty_left;
    }
    else if(duty_left>900.0)    
    {
        duty_left=900;
        duty_right=duty_right+900-duty_left;
    }
    else if (duty_right<-900.0)
    {
        duty_right=-900.0;
        duty_left=duty_left-900-duty_right;
    }
    else if(duty_right>900.0)
    {
        duty_right=900.0;
        duty_left=duty_left+900-duty_right;
    }
*/
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