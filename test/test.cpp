#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "image_process.h"
#include "init_camera.h"
#include "2d_transformation_3d.h"
#include <math.h> // atan
#include "kalman.h"

using namespace cv;
using namespace std;

int main()
{
	clock_t lastTime = clock();
    vector<Car> carStateSet;
    Point3f gap_point(0, 10000, 0);
    VideoCapture capture;
    Mat src;
    src = capture.open("../img/2.avi");
    // src = imread("../img/src_gray.jpg", 1);
    // kalman
    double filteredSlope;
    double filtered_speed;
    //suggested initial values for high noise filtering
    Kalman myFilter(0.125, 32, 1023, 0);
    Kalman myFilterSpeed(0.125, 32, 1023, 0);

    // initial adaptive threshold parameter
    int blockSize = 5; // 5 
    int constValue = 13; // 10 
    const int maxVal = 255;
    /* adaptive threshold algorithms
    0：ADAPTIVE_THRESH_MEAN_C
    1: ADAPTIVE_THRESH_GAUSSIAN_C
    */
    int adaptiveMethod = 0;
    /*
    threshold type
    0: THRESH_BINARY
    1: THRESH_BINARY_INV */
    int thresholdType = 1;
    // kernel
    Mat kernel = getStructuringElement(
            MORPH_RECT, Size(2, 2));
    Mat src_blur, src_thresh, src_morph;
    while (capture.read(src))
    {
        // src = imread( "../img/src_gray.jpg", 1 );
        if (src.empty())
        {
          cout << "could not read the image." << endl;
          return 0;
        }
        // src_gray is global variable
        cvtColor(src, src_gray, COLOR_BGR2GRAY);
        GaussianBlur(src_gray, src_blur, Size(7, 7),
                0, 0); 
        adaptiveThreshold(src_blur, src_thresh, 
                maxVal, adaptiveMethod, 
                thresholdType, blockSize, 
                constValue); 
        morphologyEx(src_thresh, src_morph, MORPH_DILATE, 
                kernel);
        vector<Point2f> pointSet;
        thresh_callback(src_morph, pointSet);
        cout << "test: " << pointSet.size() << endl;
        // continue;
        // return 0;
	    // classification of car
	    vector<Car> carSet;
	    int num = 0;
	    // loop all the point to classify them 
	    for(;;) 
	    {
            if ( pointSet.size() != 0)
               classificationCar(&pointSet, 
                      &carSet); // carSet -> global variable
	        else
	           break;
	    }

    	// consume time 
		clock_t currentTime = clock();
		double consumeTime = 1000 * (currentTime - lastTime) / (double)CLOCKS_PER_SEC;
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
            if (marker == 1)
            {
                cout << "marker: " << marker << endl;
                
                cout << "slope: " << slope << endl;  
                if (slope > 120.0)
                {
                    cout << "large than 120" << endl;
                    return 0;
                }
                // loaded from the first line
                Point3f worldPoint;
                pointToWorld(medianPoint, worldPoint, rvecM1, 
                        tvec1, cameraMatrix, s);
                cout << "worldPoint: " << worldPoint << "mm" << endl;
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
                
                    // Point3f speed = (worldPoint -
                    //         lastWorldPoint) / consumeTime;
                    // filter data from kalman
                    // add calculated value into Kalman when moving
                    if (lastMedianPoint != medianPoint)
                        slope = getSlope(lastMedianPoint, medianPoint);
                    filteredSlope = myFilter.getFilteredValue(slope);
                    float speed = sqrt(pow(worldPoint.x - 
                        lastWorldPoint.x, 2) + pow(worldPoint.y - 
                        lastWorldPoint.y, 2)) / consumeTime;
                    filtered_speed = myFilterSpeed.getFilteredValue(speed);
                    cout << "filteredSlope: " << filteredSlope << endl;
                    cout << "filtered_speed: " << filtered_speed  << "mm" << endl;
                    deleteCar(*iter, carStateSet);
                }
                // save last state
                carStateSet.push_back((*iter));
            }
            iter ++;
        }   
        // update the last state
        lastTime = currentTime;
    }
    capture.release();
    return 0;
}
