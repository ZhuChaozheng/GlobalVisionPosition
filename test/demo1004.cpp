#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctype.h>
#include "timer.h"
#include "kalman.h"
#include "image_process.h"
#include "init_camera.h"
#include "2d_transformation_3d.h"
#include "ctrl_test.h"
#include "comm_test.h"

using namespace cv;
using namespace std;

vector<Car> carSet;
Mat src;

// -0.55 0 0.11
pid pid_turn(-1.2, 0, 0.2);
pid pid_move(0, 0, 0);
udp comm1;
/* define comm format*/
char a[6] = {0x11,0x00,0x00,0x00,0x00,0x22};
/* define target value */
double target_angel = 0;
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


void ConfigParamtersRead( double* param_turn_p, double* param_turn_i,
        double* param_turn_d, double* param_move_p, 
        double* param_move_i, double* param_move_d )
{
    cv::FileStorage fs("../config/configure.yaml", 
            cv::FileStorage::READ);
    fs["TurnP"] >> *param_turn_p;
    fs["TurnI"] >> *param_turn_i;
    fs["TurnD"] >> *param_turn_d;
    fs["MoveP"] >> *param_move_p;
    fs["MoveI"] >> *param_move_i;
    fs["MoveD"] >> *param_move_d;
    // Car car;
    // fs["car0"] >> &car;
    // cout << car.ip << endl;


    fs.release();
    std::cout << "File Read Finished!" << std::endl;
}

void CtrlParametersUpdateTask( double* param_turn_p, double* param_turn_i, double* param_turn_d,
                double* param_move_p, double* param_move_i, double* param_move_d )
{
  ConfigParamtersRead( param_turn_p, param_turn_i, param_turn_d, param_move_p, param_move_i, param_move_d );
  // std::cout << "thread2:" << *param_turn_p << std::endl;
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
    // update parameter
    pid_turn.set_pid(param_turn_p, param_turn_i, param_turn_d);
    if (abs(error_angel) > 5)
        pid_turn.set_pid(param_turn_p - 0.4, param_turn_i, 
            param_turn_d + 0.02);    
    wp_angel = pid_turn.get_pid(error_angel, 100);
    if (wp_angel > 1500)
        wp_angel = 1500;
    else if (wp_angel < -1500)
        wp_angel = -1500;
    // cout << "wp_angel: " << wp_angel << endl;
    //oat error_move = (test_point.x * worldPoint.x + test_point.y * worldPoint.y)
    //              / sqrt(pow(worldPoint.x,2) + pow(worldPoint.y,2));
    float error_move = sqrt(pow(target_speed.x - 
            currentSpeed.x, 2) + pow(target_speed.y - 
            currentSpeed.y, 2));

    // Normalized on specifical value, here is 1m
    error_move /= 1000;
    error_move = (abs(error_move) > 1 )? 1:error_move;
    // Point3f speed_move = target_speed - currentSpeed;
    
    // wp_move = pid_move.get_pid(error_move, 100);
    wp_angel = 0;
    wp_move = 500;
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

int main()
{
    // ***** dynamicly tune parameter thread ********
    // intialize timer thread
    Timer timer;
    // execute task every 2000 microsecond
    timer.start(2000, std::bind(CtrlParametersUpdateTask,
              &param_turn_p, &param_turn_i, &param_turn_d, 
              &param_move_p, &param_move_i, &param_move_d));

    // *********** main thread ******************
    clock_t lastTime = clock();
    vector<Car> carStateSet;
    Point3f gap_point(0, 10000, 0);
    init();
    // Initialize Data
    vector<double> sine;
    // Create Ploter
    Mat data(sine);
    // Ptr<plot::Plot2d> plot = plot::createPlot2d(data);
    int t = 0; // cal num

    // kalman
    double filteredSlope;
    //suggested initial values for high noise filtering
    Kalman myFilter(0.125,32, 1023,0); 
    // namedWindow("src_gray", WINDOW_NORMAL);
    // resizeWindow("src_gray", 800, 600);
  	while (1) 
  	{
        // src -> global variable
        StartGrabStream(src);
        // src = imread( "../img/src_gray.jpg", 1 );
        if (src.empty())
        {
          cout << "could not read the image." << endl;
          return 0;
        }
        // src_gray is global variable
        blur(src, src_gray, Size(1, 1));
        vector<Point2f> pointSet;
        thresh_callback(src_gray, pointSet);
        // classification of car
        vector<Car> carSet;
        int num = 0;
        // loop all the point to classify them 
        for(;;) 
        {
            if (pointSet.size() != 0)
               classificationCar(&pointSet, 
                      &carSet); // carSet -> global variable
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
  	    for (auto iter = carSet.begin(); iter != carSet.end();)
  	    {
	    	getCarKeyAttribution(*iter);
	    	Point medianPoint = (*iter).medianPoint;
	    	int marker = (*iter).marker;
	    	double slope = (*iter).slope;
            Point3f targetPoint = (*iter).target;

	    	cout << "marker: " << marker << endl;
            // if (marker == 0)
            //   return 0;
	    	cout << "slope: " << slope << endl;  
            
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
        
				Point3f speed = (worldPoint -
                        lastWorldPoint) / consumeTime;
				// double angularGap = abs(lastSlope - slope);
                double angularGap = 0;
                // cout << lastSlope << endl;
                // cout << "angularGap" << angularGap << endl;
                // double angularVelocity = angularGap / consumeTime; 

                if (angularGap < 200)
                {
                    // save last state
                    lastCar.medianPoint = medianPoint;
                    lastCar.slope = slope;
                    //filter data from kalman
                    filteredSlope = myFilter.getFilteredValue(slope);
                    // filteredSlope = slope;
                    cout << "filteredSlope: " << filteredSlope << endl;
                }
              	cout << "speed: " << speed  << "mm" << endl;

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
                controlSpeedAndAngular(target_speed, 
                        target_angel, speed, filteredSlope, marker);
			}
  			else
    			carStateSet.push_back((*iter));

    	  iter ++;
      	}   
    	  // update the last state
    		lastTime = currentTime;
	  }

}
