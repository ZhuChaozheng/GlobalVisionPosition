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


// classification of car
vector<Car> car_set;

// double target_angel = 90.0;

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
    // clear old car set
    car_set.erase(car_set.begin(), car_set.end());
    for (int i = 1; i <= 1; i ++) {
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
        
        Car *car = new Car(marker_, slope_p_, slope_i_, 
                slope_d_, speed_p_, speed_i_, speed_d_, ip_, 
                port_, target_slope_, target_speed_);
        car_set.push_back(*car);
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
    // ***** dynamicly tune parameter thread ********
    // intialize timer thread
    Timer timer;
    // execute task every 2000 microsecond
    timer.start(2000, std::bind(ConfigParamtersRead));
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
    
    Mat src;
    Mat src_thresh;
    while (1) 
    {
        StartGrabStream(src);
        if (src.empty())
        {
            cout << "could not read the image." << endl;
            return 0;
        }
        threshold(src, src_thresh, 60, 255, THRESH_BINARY);
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
            getCarKeyAttribution(*iter);
            Point2f medianPoint = (*iter).get_median_point();
            int marker = (*iter).get_marker();
            double slope = (*iter).get_slope();
            Point3f targetPoint = (*iter).get_target();

            cout << "marker: " << marker << endl;
            cout << "slope: " << slope << endl;  
            // plot curve
            // sine_angle.erase(sine_angle.begin());
            // sine_angle.push_back(slope);        
            // // Render Plot Image
            // Mat image;
            // plot_angle->render( image );
            // // Show Image
            // imshow("curve_angle", image ); 

            // target
            // (*iter).set_target_slope(90.0);
            // (*iter).set_target_speed(13.0);
            double target_speed = (*iter).get_target_speed();
            // plot standard curve
            sine_manhattan_distance.erase(sine_manhattan_distance.begin());
            sine_manhattan_distance.push_back(target_speed);        
            // Render Plot Image
            Mat image_manhattan_distance;
            plot_manhattan_distance->render( image_manhattan_distance );
            // Show Image
            imshow("curve_standard_speed", image_manhattan_distance ); 
            
            // loaded from the first line
            Point3f worldPoint;
            pointToWorld(medianPoint, worldPoint, rvecM1, 
                         tvec1, cameraMatrix, s);
            Car lastCar;

            if (exist((*iter), carStateSet, lastCar))
            {
                Point lastMedianPoint = lastCar.get_median_point();
                double lastSlope = lastCar.get_slope();
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
                // filteredSlope = myFilter.getFilteredValue(slope);
        
                float speed = sqrt(pow(worldPoint.x - 
                    lastWorldPoint.x, 2) + pow(worldPoint.y - 
                    lastWorldPoint.y, 2)) / consumeTime * 1000;
                cout << "speed: " << speed  << "mm" << endl;
                (*iter).set_speed(speed);
                // plot curve
                sine_speed.erase(sine_speed.begin());
                sine_speed.push_back(speed);        
                // Render Plot Image
                Mat image_speed;
                plot_speed->render( image_speed );
                // Show Image
                imshow("curve_speed", image_speed ); 
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
                pid pid_control;
                pid_control.controlSpeedAndAngular(*iter);
            }
            carStateSet.push_back((*iter));
            iter ++;
          
        }   
        
        // update the last state
        lastTime = currentTime;
    }
    return 0;

}
    
