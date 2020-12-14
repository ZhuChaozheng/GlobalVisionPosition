#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctype.h>
#include <vector>
#include <opencv2/plot.hpp>
#include "timer.h"
#include "kalman.h"
#include "image_process.h"
#include "init_camera.h"
#include "2d_transformation_3d.h"
#include "ctrl_test.h"
#include "comm_test.h"
#include <iostream>
#include <utility>
#include <thread>
#include <chrono>
#include <functional>
#include <atomic>


using namespace cv;
using namespace std;


// classification of car
vector<Car> car_set;
Mat src;

// -0.55 0 0.11
pid pid_turn(-1.2, 0, 0.2);
pid pid_move(0, 0, 0);
udp comm1;
/* define comm format*/
char a[6] = {0x11,0x00,0x00,0x00,0x00,0x22};
/* define target value */
double target_angel = 90;
double wp_angel = 0;
double temp_sin=0;
double temp_cos=0;

double target_move = 0;
double wp_move = 0;
// Point3f test_point(-42810.9, 26903.5, 19346);
Point test_point(594, 306); 
Point medianPoint;

// initialize parameters for PID controler
double param_turn_p, param_turn_i, param_turn_d, 
        param_move_p, param_move_i, param_move_d;

void ConfigParamtersRead()
{
    FileStorage fs("../config/configure.yaml", 
            FileStorage::READ);
    string front_str;
    int marker_;
    float p_, i_, d_;
    string ip_;
    int port_;
    // clear old car set
    car_set.erase(car_set.begin(), car_set.end());
    for (int i = 0; i <= 0; i ++) {
        string front_str = "marker_";
        string combined_str = front_str + to_string(i);
        fs[combined_str] >> marker_;
        
        front_str = "p_";
        combined_str = front_str + to_string(i);
        fs[combined_str] >> p_;

        front_str = "i_";
        combined_str = front_str + to_string(i);
        fs[combined_str] >> i_;

        front_str = "d_";
        combined_str = front_str + to_string(i);
        fs[combined_str] >> d_;

        front_str = "ip_";
        combined_str = front_str + to_string(i);
        fs[combined_str] >> ip_;

        front_str = "port_";
        combined_str = front_str + to_string(i);
        fs[combined_str] >> port_;
        
        Car *car = new Car(marker_, p_, i_, d_, ip_, port_);
        car_set.push_back(*car);
    }
    fs.release();
    std::cout << "File Read Finished!" << std::endl;
}

void CtrlParametersUpdateTask(int n)
{
    while(1)
    {
        std::this_thread::sleep_for(
                std::chrono::milliseconds(3000));
        ConfigParamtersRead(); 
    }
}


void init() 
{
    // load parameter from file
    ConfigFileRead(rvecM1, tvec1, cameraMatrix, s);
    EnumDevice();
    InitCamera();
}

/**
 *
 * control car through targets of speed and angular
 * input: target speed, angular, current speed, 
 * angular, marker
 * 
**/
void controlSpeedAndAngular(Point3f target_speed, 
        double target_angel, Point3f currentSpeed, 
        double filteredSlope, int marker)
{
    if (target_angel > 180)
    {
        target_angel = target_angel-360;
    }
    double error_angel = filteredSlope - target_angel;
    // double error_angel = filteredSlope - target_angel;
     
    if( error_angel < -180)
        error_angel = error_angel+360;
    else if( error_angel > 180)
        error_angel = error_angel-360;
    // search the corresponding parameters from marker
    // search 
    for (auto iter = car_set.begin(); iter != car_set.end();)
    {
        if ((*iter).marker_ == marker)
        {
            param_turn_p = (*iter).p_;
            param_turn_i = (*iter).i_;
            param_turn_d = (*iter).d_;
            break;
        }
        iter ++;
    }
    cout << param_turn_p << " " << param_turn_i 
            << " " << param_turn_d << endl;
    // update parameter
    pid_turn.set_pid(param_turn_p, param_turn_i, param_turn_d);
 //    if (abs(error_angel) > 5)
 //        pid_turn.set_pid(param_turn_p - 0.4, param_turn_i, 
 //           param_turn_d + 0.02);    
    wp_angel = pid_turn.get_pid(error_angel, 100);
    if (wp_angel > 2000)
        wp_angel = 2000;
    else if (wp_angel < -2000)
        wp_angel = -2000;
    // cout << "wp_angel: " << wp_angel << endl;
    //oat error_move = (test_point.x * worldPoint.x + test_point.y * worldPoint.y)
    //              / sqrt(pow(worldPoint.x,2) + pow(worldPoint.y,2));
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
    // combinate the corresponding command
    a[1] = (unsigned char)((((short)duty_left)>>8) & 0xff);
    a[2] = (unsigned char)(((short)duty_left) & 0xff);
    a[3] = (unsigned char)((((short)duty_right)>>8) & 0xff);
    a[4] = (unsigned char)(((short)duty_right) & 0xff);
    
    comm1.send_data(a);
}

void f1(int n)
{
    wp_move=0; target_angel=90;
 //   std::this_thread::sleep_for(std::chrono::milliseconds(10000));
 //   wp_move=300;
 //   std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  //  target_angel=90;
 //   std::this_thread::sleep_for(std::chrono::milliseconds(3000));
 //   target_angel=180;
  //  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
 //   target_angel=270;
  //  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
 //   target_angel=0;
}


int main()
{
    // ***** dynamicly tune parameter thread ********
    // intialize timer thread
    Timer timer;
    // execute task every 2000 microsecond
    // timer.start(2000, std::bind(CtrlParametersUpdateTask));

    thread t2(CtrlParametersUpdateTask, 1);
    // *********** main thread ******************
    clock_t lastTime = clock();
    vector<Car> carStateSet;
    Point3f gap_point(0, 10000, 0);
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
        sine_speed.push_back( (rand() % 100));
    }
    // Create Ploter
    cv::Mat data_speed( sine_speed );
    Ptr<cv::plot::Plot2d> plot_speed = 
            cv::plot::Plot2d::create( data_speed );

    // define the manhattan distance range of 0-100
    vector<double> sine_manhattan_distance;
    for( int t = 0; t < 100; t++ ){
        sine_manhattan_distance.push_back( (rand() % 360));
    }
    // Create Ploter
    cv::Mat data_manhattan_distance( sine_manhattan_distance );
    Ptr<cv::plot::Plot2d> plot_manhattan_distance = 
            cv::plot::Plot2d::create( data_manhattan_distance );
    
    // init parameter of pid
    ConfigParamtersRead();
    // kalman
    double filteredSlope;
    double filtered_speed;
    //suggested initial values for high noise filtering
    Kalman myFilter(0.125, 32, 1023, 0);
    Kalman myFilterSpeed(0.125, 32, 1023, 0);
    
    
    Mat src_thresh;
  	while (1) 
    {
        StartGrabStream(src);
        // src = imread( "../img/src_gray.jpg", 1 );
        if (src.empty())
        {
            cout << "could not read the image." << endl;
            return 0;
        }
        threshold(src, src_thresh, 60, 255, THRESH_BINARY);
        imshow("src_thresh", src_thresh);
        vector<Point2f> pointSet;
        thresh_callback(src_thresh, pointSet);
      
        int num = 0;
        // loop all the point to classify them 
        for(;;) 
        {
            if (pointSet.size() != 0)
            {
                classificationCar(&pointSet, 
                      car_set); // carSet -> global variable
            }
            else
               break;
        }

        // consume time 
        clock_t currentTime = clock();
        double consumeTime = 1000 * (currentTime - 
            lastTime) / (double)CLOCKS_PER_SEC;
        cout << "time" << consumeTime << "ms" << endl;
        cout << "fps: " << 1000 / consumeTime << "Hz" << endl;

        // output car attribution
        for (auto iter = car_set.begin(); iter != car_set.end();)
        {
            // cout << "point_size: " << (*iter).pointSet.size() << endl;
            // if ((*iter).pointSet.size() == 0)
            //     continue;
            getCarKeyAttribution(*iter);
            Point2f medianPoint = (*iter).medianPoint;
            int marker = (*iter).marker_;
            double slope = (*iter).slope;
            Point3f targetPoint = (*iter).target;

            cout << "marker: " << marker << endl;
            // if (marker != 0 | marker != 1)
            //   continue;
            cout << "slope: " << slope << endl;  
            // plot curve
            sine_angle.erase(sine_angle.begin());
            sine_angle.push_back(slope);        
            // Render Plot Image
            Mat image;
            plot_angle->render( image );
            // Show Image
            imshow("curve_angle", image ); 

            // plot standard curve
            sine_manhattan_distance.erase(sine_manhattan_distance.begin());
            sine_manhattan_distance.push_back(target_angel);        
            // Render Plot Image
            Mat image_manhattan_distance;
            plot_manhattan_distance->render( image_manhattan_distance );
            // Show Image
            imshow("curve_standard_angle", image_manhattan_distance ); 
            
            // loaded from the first line
            Point3f worldPoint;
            pointToWorld(medianPoint, worldPoint, rvecM1, 
                         tvec1, cameraMatrix, s);
            // cout << "worldPoint: " << worldPoint << "mm" << endl;
            cout << medianPoint << endl;
            //if (targetPoint.y == 0) 
               // targetPoint = worldPoint + gap_point;
            //cout << "target: " << targetPoint << endl;
            Car lastCar;

            if (exist((*iter), carStateSet, lastCar))
            {
                Point lastMedianPoint = lastCar.medianPoint;
                double lastSlope = lastCar.slope;
                Point3f lastWorldPoint;
                // loaded parameters from the first line
                pointToWorld(lastMedianPoint, lastWorldPoint, 
                        rvecM1, tvec1, cameraMatrix, s);  
            
                Point3f speed_three_dim = (worldPoint -
                        lastWorldPoint) / consumeTime * 1000;
                //cout << "speed_three_dim: " << speed_three_dim << endl;
                // filter data from kalman
                // add calculated value into Kalman when moving
                //if (lastMedianPoint != medianPoint)
                  //  slope = getSlope(lastMedianPoint, medianPoint);
                filteredSlope = myFilter.getFilteredValue(slope);
        
                float speed = sqrt(pow(worldPoint.x - 
                    lastWorldPoint.x, 2) + pow(worldPoint.y - 
                    lastWorldPoint.y, 2)) / consumeTime * 1000;
                cout << "speed: " << speed  << "mm" << endl;
                // plot curve
                sine_speed.erase(sine_speed.begin());
                sine_speed.push_back(speed);        
                // Render Plot Image
                // Mat image_speed;
                // plot_speed->render( image_speed );
                // // Show Image
                // imshow("curve_speed", image_speed ); 
                // filtered_speed = myFilterSpeed.getFilteredValue(speed);
                // cout << "filteredSlope: " << filteredSlope << endl;
                //cout << "filtered_speed: " << filtered_speed  << "mm" << endl;
                deleteCar(*iter, carStateSet);

                // TODO
                /*
                 * define the target_speed and target_angel
                 * global variable of target_angel
                 * note: target_angel derived from two 2d points,
                 * you can also try to use 3d, but may it has 
                 * subtle error.
                 */
                // target_angel = getSlope(medianPoint, test_point);
                // target_angel = 180;
                //cout << "target_angel" << target_angel << endl;
                Point3f target_speed = Point3f(-2254.89, -19356.3, 0);
                
                // Point test_point(594, 306); 
                controlSpeedAndAngular(target_speed, 
                        target_angel, speed_three_dim, 
                        slope, marker);
            }
            
            carStateSet.push_back((*iter));

          iter ++;
        }   
        
        // update the last state
        lastTime = currentTime;
    }
    return 0;

}
    
