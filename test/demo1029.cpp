#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "image_process.h"
#include "init_camera.h"
#include "2d_transformation_3d.h"

using namespace cv;
using namespace std;

vector<Car> carSet;
Mat src;



void init() 
{
    // load parameter from file
    ConfigFileRead(rvecM1, tvec1, cameraMatrix, s);

    EnumDevice();
    InitCamera();
}

void loopFunction()
{
    // src -> global variable
    StartGrabStream(src);
    // src = imread( "../img/5.bmp", 1 );
    if (src.empty())
    {
      cout << "could not read the image." << endl;
      return ;
    }
    // src_gray is global variable
    // Mat src_gray;
    // cvtColor( src, src_gray, COLOR_BGR2GRAY );

    blur( src, src_gray, Size(1, 1) );
    // imshow("src_gray", src_gray);
      
    const int max_thresh = 255;
    vector<Point2f> newPointSet;
    
    thresh_callback( src_gray, newPointSet );   

    // classification of car
    
    // int num = 0;
    // // loop all the point to classify them 
    // for(;;) 
    // {
    //     if ( newPointSet.size() != 0)
    //        classificationCar(&newPointSet, 
    //               &carSet); // carSet -> global variable
    //     else
    //        break;
    // }
}

int main()
{
    clock_t lastTime = clock();
    vector<Car> carStateSet;
    Point3f gap_point(0, 10000, 0);
    init();
    while (1) 
    {
        loopFunction();
        // consume time 
            clock_t currentTime = clock();
            double consumeTime = 1000 * (currentTime - 
                lastTime) / (double)CLOCKS_PER_SEC;
            cout << "time" << consumeTime << "ms" << endl;
            cout << "fps: " << 1000 / consumeTime << "Hz" << endl;

        // output car attribution
        // for (auto iter = carSet.begin(); iter != carSet.end();)
        // {
        //         getCarKeyAttribution(*iter);
        //         Point medianPoint = (*iter).medianPoint;
        //         int marker = (*iter).marker;
        //         double slope = (*iter).slope;
        //     Point3f targetPoint = (*iter).target;

        //         cout << "marker: " << marker << endl;
        //         cout << "slope: " << slope << endl;            

        //         Point3f worldPoint;
        //         // loaded from the first line
        //         pointToWorld(medianPoint, worldPoint, rvecM1, 
        //                      tvec1, cameraMatrix, s);
        //         cout << "worldPoint: " << worldPoint / 100 << 
        //             "mm" << endl;
        //         if (targetPoint.y == 0) 
        //         targetPoint = worldPoint + gap_point;
        //     cout << "target: " << targetPoint << endl;
        //     Car lastCar;

        //         if (exist((*iter), carStateSet, lastCar))
        //         {
        //                 Point lastMedianPoint = lastCar.medianPoint;
        //                 double lastSlope = lastCar.slope;

        //                 Point3f lastWorldPoint;
        //                 // loaded parameters from the first line
        //                 pointToWorld(lastMedianPoint, lastWorldPoint, 
        //               rvecM1, tvec1, cameraMatrix, s);  

        //                 Point3f accelerate = (lastWorldPoint - 
        //               worldPoint) / consumeTime;
        //                 double angularVelocity = (lastSlope - 
        //               slope) / consumeTime; 
        //                 cout << "accelerate: " << accelerate / 100 << 
        //               "mm" << endl;
        //                 cout << "angularVelocity: " << angularVelocity <<
        //               endl;

        //                 // save last state
        //                 lastCar.medianPoint = medianPoint;
        //                 lastCar.slope = slope;
        //         }
        //         else
        //                 carStateSet.push_back((*iter));

        //             iter ++;
        //   }   
            // update the last state
              lastTime = currentTime;
      }

}