#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctype.h>
#include <vector>
#include "timer.h"
#include "kalman.h"
#include "image_process.h"
#include "init_camera.h"
#include "2d_transformation_3d.h"
#include "ctrl_test.h"
#include "comm_test.h"

using namespace cv;
using namespace std;

Mat src, thresh_output;

/* define target value */
double target_angel = 0;

int edgeThresh = 100;
int marker_;
float p_, i_, d_;
string ip_;
int port_;

vector<Car> car_set;

void ConfigParamtersRead(int marker_, float p_, 
        float i_, float d_, string ip_, 
        int port_)
{
    cv::FileStorage fs("../config/configure.yaml", 
            cv::FileStorage::READ);
    string front_str;
    for (int i = 0; i <= 7; i ++) {
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
    cout << "parameter size: " << car_set.size() << endl;
    fs.release();
    std::cout << "File Read Finished!" << std::endl;
}

int main()
{
    VideoCapture capture;
    Mat src;
    src = capture.open("../img/1.avi");

    vector<Car> carStateSet;

    // kalman
    double filteredSlope;
    double filtered_speed;
    //suggested initial values for high noise filtering
    Kalman myFilter(0.125, 32, 1023, 0);
    Kalman myFilterSpeed(0.125, 32, 1023, 0);
    // call time
    clock_t lastTime = clock();

    ConfigParamtersRead(marker_, p_, i_, 
            d_, ip_, port_);
    // cout parameter data
    for(auto iter = car_set.begin(); iter!= car_set.end();)
    {
        cout << (*iter).marker_ << endl;
        cout << (*iter).p_ << endl;
        cout << (*iter).i_ << endl;
        cout << (*iter).d_ << endl;
        cout << (*iter).ip_ << endl;
        cout << (*iter).pointSet.size() << endl;
        iter ++;
    }
    // return 0;
    Mat src_blur;
    EnumDevice();
    InitCamera();
    Mat kernel = getStructuringElement(
            MORPH_RECT, Size(2, 2));
    // while (capture.read(src))
    while (1) 
    {
        StartGrabStream(src);
        // src = imread( "../img/src_gray.jpg", 1 );
        if (src.empty())
        {
          cout << "could not read the image." << endl;
          return 0;
        }
        imwrite("src_gray_latest.jpg", src);
        return 0;
        // src_gray is global variable
        // cvtColor(src, src_gray, COLOR_BGR2GRAY);
        medianBlur( src, src_blur, 3 );
        // imshow("src_blur", src_blur);
        threshold(src_blur, src_blur, 60, 255, THRESH_BINARY);
        // imshow("threshold", src_blur);
        erode(src_blur, src_blur, kernel);
        // imshow("dilate", src_blur);
        vector<Point2f> pointSet;
        thresh_callback( src_blur, pointSet);
      
        int num = 0;
        // loop all the point to classify them 
        for(;;) 
        {
            if (pointSet.size() != 0)
               classificationCar(&pointSet, 
                      car_set); // carSet -> global variable
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
            // if ((*iter).pointSet.size() <= 0)
            //     continue;
            getCarKeyAttribution(*iter);
            Point2f medianPoint = (*iter).medianPoint;
            int marker = (*iter).marker_;
            double slope = (*iter).slope;
            Point3f targetPoint = (*iter).target;

            // cout << "marker: " << marker << endl;
            // // if (marker == 0)
            // //   return 0;
            // cout << "slope: " << slope << endl;  
            
            // loaded from the first line
            Point3f worldPoint;
            pointToWorld(medianPoint, worldPoint, rvecM1, 
                         tvec1, cameraMatrix, s);
            // cout << "worldPoint: " << worldPoint << "mm" << endl;
            // cout << medianPoint << endl;
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
            
                // Point3f speed_three_dim = (worldPoint -
                //         lastWorldPoint) / consumeTime * 1000;
                //cout << "speed_three_dim: " << speed_three_dim << endl;
                // filter data from kalman
                // add calculated value into Kalman when moving
                //if (lastMedianPoint != medianPoint)
                  //  slope = getSlope(lastMedianPoint, medianPoint);
                // filteredSlope = myFilter.getFilteredValue(slope);
        
                float speed = sqrt(pow(worldPoint.x - 
                    lastWorldPoint.x, 2) + pow(worldPoint.y - 
                    lastWorldPoint.y, 2)) / consumeTime * 1000;
                // cout << "speed: " << speed  << "mm" << endl;
                // filtered_speed = myFilterSpeed.getFilteredValue(speed);
                // cout << "filteredSlope: " << filteredSlope << endl;
                //cout << "filtered_speed: " << filtered_speed  << "mm" << endl;
                // deleteCar(*iter, carStateSet);

                // TODO
                /*
                 * define the target_speed and target_angel
                 * global variable of target_angel
                 * note: target_angel derived from two 2d points,
                 * you can also try to use 3d, but may it has 
                 * subtle error.
                 */
                // target_angel = getSlope(medianPoint, test_point);
                target_angel = 90;
                //cout << "target_angel" << target_angel << endl;
                Point3f target_speed = Point3f(-2254.89, -19356.3, 0);
                
                // Point test_point(594, 306); 
                // controlSpeedAndAngular(target_speed, 
                //         target_angel, speed_three_dim, filteredSlope, marker);
            }
            
            carStateSet.push_back((*iter));

          iter ++;
        }   
        
        // update the last state
        lastTime = currentTime;
  }
    capture.release();
    return 0;

}
