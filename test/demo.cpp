/**
 * created by Jason, Hohai university
 * 2020.12.16
 * core functions of launching sub-timer-thread of updating parameters 
 * of each car and image process in real-time, while calculating
 * the state of each car, setting its abbtributions and send different
 * motion instructions.
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <sys/time.h>
#include <signal.h>
#include <opencv2/plot.hpp>
#include "timer.h"
#include "kalman.h"
#include "car.h"
#include "udp.h"
#include "pid.h"
#include "image_process.h"
#include "init_camera.h"
#include "2d_transformation_3d.h"

using namespace cv;
using namespace std;

enum Choice
{
    RECTANGLE = 1,
    CIRCLE = 2,
    TRIANGLE = 3 
};
// classification of car
vector<Car> car_set;


bool app_stopped = false;
void sigint_handler(int sig) {
    if (sig == SIGINT) 
    {
        std::cout << "C pressed!" << endl;
        app_stopped = true;
    }
}

void StopCars()
{
    
    /* define comm format*/
    char a[6] = {0x11,0x00,0x00,0x00,0x00,0x22};
    // build communication based on udp
    for (auto iter = car_set.begin(); 
                iter != car_set.end();)
    {
        int marker = (*iter).get_marker();
        string ip = (*iter).get_ip();
        udp udp_comm;
        int sock_fd = udp_comm.udp_init(ip);
        // send data through udp
        udp_comm.send_data(sock_fd, a);
        cout << marker << " stopped!" << endl;
        iter ++;
    }
    
}

/*
 * modify car's slope and sleep, according to 
 * the marker.
 * 
 * 
 */
void UpdateSlope(const double target_slope,
        const int marker)
{
    for (auto iter = car_set.begin(); 
            iter != car_set.end();)
    {
        int marker_ = (*iter).get_marker();
        if (marker_ == marker)
        {
            (*iter).set_target_slope(target_slope);
            break;
        }
    }
}


/* 
 * 
 * define multiple trajectory modules, loop 0, 90, 180, 270
 * to form a rectangle. 
 *
 *
 */
// void TimerTrajectory(const int trajectory_type, 
//         const int marker)
// {
//     switch(trajectory_type):
//         case RECTANGLE:
//             // TODO
//             UpdateSlope(0, marker);
//             Timer timer;
//             timer.startOnce(4000, UpdateSlope(90, marker));           ;
//             // Sleep(2)
//             break;
//         case CIRCLE:
//             // TODO
//             break;
//         case TRIANGLE:
//             // TODO
//             break;

// }


void ConfigParamtersRead()
{
    FileStorage fs("../config/configure.yaml", 
            FileStorage::READ);
    string front_str;
    int marker_;
    float slope_p_, slope_i_, slope_d_,
            speed_p_, speed_i_, speed_d_;
    string ip_;
    int port_;
    double target_slope_; 
    float target_speed_; 
    bool flag = false; // match existing car
    // clear old car set
    
    // car_set.erase(car_set.begin(), car_set.end());
    for (int i = 5; i <= 5; i ++) {
        string front_str = "marker_";
        string combined_str = front_str + to_string(i);
        fs[combined_str] >> marker_;
        
        front_str = "slope_p_";
        combined_str = front_str + to_string(i);
        fs[combined_str] >> slope_p_;

        front_str = "slope_i_";
        combined_str = front_str + to_string(i);
        fs[combined_str] >> slope_i_;

        front_str = "slope_d_";
        combined_str = front_str + to_string(i);
        fs[combined_str] >> slope_d_;

        front_str = "speed_p_";
        combined_str = front_str + to_string(i);
        fs[combined_str] >> speed_p_;

        front_str = "speed_i_";
        combined_str = front_str + to_string(i);
        fs[combined_str] >> speed_i_;

        front_str = "speed_d_";
        combined_str = front_str + to_string(i);
        fs[combined_str] >> speed_d_;

        front_str = "ip_";
        combined_str = front_str + to_string(i);
        fs[combined_str] >> ip_;

        front_str = "port_";
        combined_str = front_str + to_string(i);
        fs[combined_str] >> port_;

        front_str = "target_slope_";
        combined_str = front_str + to_string(i);
        fs[combined_str] >> target_slope_;

        front_str = "target_speed_";
        combined_str = front_str + to_string(i);
        fs[combined_str] >> target_speed_;
        cout << "read target_speed_: " << target_speed_ << endl;
        for (auto iter = car_set.begin(); 
                iter != car_set.end();)
        {
            int marker = (*iter).get_marker();
            if (marker == i)
            {
                (*iter).update_parameters(slope_p_, slope_i_, 
                        slope_d_, speed_p_, speed_i_, 
                        speed_d_, ip_, target_slope_, 
                        target_speed_);
                flag = true;
                break;
            }
        }
        if (!flag)
        {
            Car *car = new Car(marker_, slope_p_, slope_i_, 
                slope_d_, speed_p_, speed_i_, speed_d_, ip_, 
                port_, target_slope_, target_speed_);
            car_set.push_back(*car);
        }
    }
    fs.release();
    std::cout << "File Read Finished!" << std::endl;
}

void init() 
{
    // load parameter from file
    ConfigFileRead(rvecM1, tvec1, cameraMatrix, s);
    EnumDevice();
    InitCamera();
}

int main()
{
    // TimerTrajectory(RECTANGLE, 5);
    // ***** dynamicly tune parameter thread ********
    // intialize timer thread
    Timer timer;
    // execute task every 2000 microsecond
    timer.start(2000, std::bind(ConfigParamtersRead));

    // *********** main thread ******************
    // register signal ctrl+c
    signal(SIGINT, sigint_handler);
    // clock_t lastTime = clock();
    struct timeval lastTime, currentTime;
    double consumeTime;
    gettimeofday(&lastTime,NULL);
    vector<Car> carStateSet;
    init();
    
    // Initialize Data
    // define the angle range of 0-360
    vector<double> sine_angle;
    for( int t = 0; t < 100; t++ ){
        sine_angle.push_back( (rand() % 360));
    }
    // Create Ploter
    cv::Mat data_angle( sine_angle );
    Ptr<cv::plot::Plot2d> plot_angle = 
            cv::plot::Plot2d::create( data_angle );

    // define the speed range of 0-100
    vector<double> sine_speed;
    for( int t = 0; t < 100; t++ ){
        sine_speed.push_back( (rand() % 300));
    }
    // Create Ploter
    cv::Mat data_speed( sine_speed );
    Ptr<cv::plot::Plot2d> plot_speed = 
            cv::plot::Plot2d::create( data_speed );

    // define the manhattan distance range of 0-100
    vector<double> sine_manhattan_distance;
    for( int t = 0; t < 100; t++ ){
        sine_manhattan_distance.push_back( (rand() % 300));
    }
    // Create Ploter
    cv::Mat data_manhattan_distance( sine_manhattan_distance );
    Ptr<cv::plot::Plot2d> plot_manhattan_distance = 
            cv::plot::Plot2d::create( data_manhattan_distance );
    
    // init parameter of pid
    ConfigParamtersRead();
    // return 0;
    // Kalman
    Kalman myFilterSpeed = Kalman(0.125,32,1023,0);
    double filtered_speed; 
    
    Mat src;
    Mat src_thresh;
    double sum_time;
    int i = 0;
    
    while (1) 
    {

        cout << i++ << endl;
        if (app_stopped) break;
        StartGrabStream(src);
        if (src.empty())
        {
            cout << "could not read the image." << endl;
            return 0;
        }
        // TODO
        // warpAffine
        // Mat warp_rotate_dst;
        // cout << cameraMatrix << endl;
        // // warpAffine( src, warp_rotate_dst, cameraMatrix, src.size() );
        // warpAffine(src, warp_rotate_dst, cameraMatrix, src.size(), INTER_CUBIC);
        // imwrite("warp_dst.jpg", src);
        // imwrite("warp_rotate_dst.jpg", warp_rotate_dst);
        // return 0;
        threshold(src, src_thresh, 110, 255, THRESH_BINARY);
        imshow("src_thresh", src_thresh);

        vector<Point2f> pointSet;
        ThreshCallBack(src_thresh, pointSet);
      
        int num = 0;
        // loop all the point to classify them 
        for(;;) 
        {
            if (pointSet.size() != 0)
            {
                ClassificationCar(&pointSet, 
                      car_set); // carSet -> global variable
            }
            else
               break;
        }



        // output car attribution
        for (auto iter = car_set.begin(); iter != car_set.end();)
        {
            double target_slope = (*iter).get_target_slope();
            cout << "target_slope " << target_slope << endl;
            GetCarKeyAttribution(*iter);
            Point2f medianPoint = (*iter).get_median_point();
            int marker = (*iter).get_marker();
            double slope = (*iter).get_slope();
            Point3f targetPoint = (*iter).get_target();

            cout << "marker: " << marker << endl;
            cout << "slope: " << slope << endl;
            double filteredSlope = (*iter).filter_.getFilteredValue(slope);
            // cout << "filteredSlope: " << filteredSlope << endl;
            // plot curve
            sine_angle.erase(sine_angle.begin());
            sine_angle.push_back(slope);        
            // Render Plot Image
            //Mat image;
            //plot_angle->render( image );
            // Show Image
            //imshow("curve_angle", image ); 
            double target_speed = (*iter).get_target_speed();
            // plot standard curve
            // sine_manhattan_distance.erase(sine_manhattan_distance.begin());
            // sine_manhattan_distance.push_back(target_speed);        
            // // Render Plot Image
            // Mat image_manhattan_distance;
            // plot_manhattan_distance->render( image_manhattan_distance );
            // // Show Image
            // imshow("curve_standard_speed", image_manhattan_distance ); 
            cout << "medianPoint: " << medianPoint << endl;
            // loaded from the first line
            // Point3f worldPoint;
            // PointToWorld(medianPoint, worldPoint, rvecM1, 
            //              tvec1, cameraMatrix, s);
            Car lastCar;

            if (Exist((*iter), carStateSet, lastCar))
            {
                Point2f lastMedianPoint = lastCar.get_median_point();
                cout << "lastMedianPoint: " << lastMedianPoint << endl;
                double lastSlope = lastCar.get_slope();
                // Point3f lastWorldPoint;
                // loaded parameters from the first line
                // PointToWorld(lastMedianPoint, lastWorldPoint, 
                //         rvecM1, tvec1, cameraMatrix, s);  
            
                // Point3f speed_three_dim = (worldPoint -
                //         lastWorldPoint) / consumeTime * 1000;
                //cout << "speed_three_dim: " << speed_three_dim << endl;
                // filter data from kalman
                // add calculated value into Kalman when moving
                // if (lastMedianPoint != medianPoint)
                //    slope = getSlope(lastMedianPoint, medianPoint);
                // filteredSlope = myFilter.getFilteredValue(slope);
                float speed = sqrt(pow(medianPoint.x - 
                    lastMedianPoint.x, 2) + pow(medianPoint.y - 
                    lastMedianPoint.y, 2)) / consumeTime;
                // float speed = sqrt(pow(worldPoint.x - 
                //     lastWorldPoint.x, 2) + pow(worldPoint.y - 
                //     lastWorldPoint.y, 2)) / consumeTime * 1000;
                cout << "speed: " << speed  << "px/s" << endl;                
                (*iter).set_speed(speed);
                filtered_speed = myFilterSpeed.getFilteredValue(speed);
                cout << "filtered_speed: " << filtered_speed << "px/s" << endl;
                // plot curve
                sine_speed.erase(sine_speed.begin());
                sine_speed.push_back(filtered_speed);        
                // Render Plot Image
                // Mat image_speed;
                // plot_speed->render( image_speed );
                // // Show Image
                // imshow("curve_speed", image_speed ); 
                
                DeleteCar(*iter, carStateSet);

                // TODO
                /*
                 * define the target_speed and target_angel
                 * global variable of target_angel
                 * note: target_angel derived from two 2d points,
                 * you can also try to use 3d, but may it has 
                 * subtle error.
                 */
                pid pid_control;
                pid_control.controlSpeedAndAngular(*iter);
            }
            carStateSet.push_back((*iter));
            iter ++;
          
        }   
        // consume time 
        // clock_t currentTime = clock();
        cout << "lastTime: " << lastTime.tv_usec << endl;
        gettimeofday(&currentTime, NULL);
        consumeTime = (currentTime.tv_sec - lastTime.tv_sec) + 
                (double)(currentTime.tv_usec - lastTime.tv_usec) / 1000000.0;
        cout << "time" << consumeTime << "s" << endl;
        cout << "fps: " << 1 / consumeTime << "Hz" << endl;
        // update the last state
        lastTime = currentTime;
        cout << "lastTime: " << lastTime.tv_usec << endl;
    }
    // send stop command to cars
    StopCars();
    return 0;

}
    
