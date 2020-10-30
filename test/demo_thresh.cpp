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
Mat src, canny_output;

int edgeThresh = 100;

// define a trackbar callback
static void onTrackbar(int, void*)
{
    // 初始化自适应阈值参数
    int blockSize = 5;  
    int constValue = 10;  
    const int maxVal = 255;
    /* 自适应阈值算法
  0：ADAPTIVE_THRESH_MEAN_C
  1: ADAPTIVE_THRESH_GAUSSIAN_C
  */
    int adaptiveMethod = 0;
  /*
  阈值类型
  0: THRESH_BINARY
  1: THRESH_BINARY_INV */
    int thresholdType = 1;
  // 图像自适应阈值操作
    Mat thresh_img;
    cv::adaptiveThreshold(src_gray, thresh_img, 
            edgeThresh, adaptiveMethod, 
            thresholdType, blockSize, 
            constValue); 
    //threshold(src_gray, thresh_img, edgeThresh, 255, 
       //     THRESH_BINARY_INV);
    
    //blur(thresh_img, thresh_img, Size(3, 3));
    imshow("thresh_img", thresh_img);
    waitKey(50);
    // Run the edge detector on grayscale
    // Canny(edge, edge, edgeThresh, edgeThresh*3, 3);
    Canny( src_gray, canny_output, edgeThresh, edgeThresh * 2 );
    imshow("canny_output", canny_output);
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
    init();
    namedWindow("src_gray", WINDOW_NORMAL);
    namedWindow("canny_output", WINDOW_NORMAL);
    namedWindow("thresh_img", WINDOW_NORMAL);
    resizeWindow("thresh_img", 800, 600);
    resizeWindow("src_gray", 800, 600);
    resizeWindow("canny_output", 800, 600);
        // Create a window
    namedWindow("Edge map", 1);
        // create a toolbar
    
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
        //blur(src, src_gray, Size(2, 2));
	medianBlur(src, src_gray, 3);
        imshow("src_gray", src_gray);
	createTrackbar("Canny threshold", "Edge map", 
             &edgeThresh, 255, onTrackbar);
        // Show the image
    	onTrackbar(0, 0);
        
        // classification of car
  //       vector<Car> carSet;
  //       int num = 0;
  //       // loop all the point to classify them 
  //       for(;;) 
  //       {
  //           if (pointSet.size() != 0)
  //              classificationCar(&pointSet, 
  //                     &carSet); // carSet -> global variable
  //           else
  //              break;
  //       }
  // 	    // consume time 
		// clock_t currentTime = clock();
		// double consumeTime = 1000 * (currentTime - 
  //           lastTime) / (double)CLOCKS_PER_SEC;
		// cout << "time" << consumeTime << "ms" << endl;
		// cout << "fps: " << 1000 / consumeTime << "Hz" << endl;

  // 	    // output car attribution
  // 	    for (auto iter = carSet.begin(); iter != carSet.end();)
  // 	    {
	 //    	getCarKeyAttribution(*iter);
	 //    	Point medianPoint = (*iter).medianPoint;
	 //    	int marker = (*iter).marker;
	 //    	double slope = (*iter).slope;
  //           Point3f targetPoint = (*iter).target;

	 //    	cout << "marker: " << marker << endl;
  //           // if (marker == 0)
  //           //   return 0;
	 //    	cout << "slope: " << slope << endl;  
            
  // 			// loaded from the first line
  //           Point3f worldPoint;
  // 			pointToWorld(medianPoint, worldPoint, rvecM1, 
 	// 					 tvec1, cameraMatrix, s);
 	// 		// cout << "worldPoint: " << worldPoint << "mm" << endl;
  // 			cout << medianPoint << endl;
  //           //if (targetPoint.y == 0) 
  //              // targetPoint = worldPoint + gap_point;
  //           //cout << "target: " << targetPoint << endl;
  //           Car lastCar;

  // 			if (exist((*iter), carStateSet, lastCar))
  // 			{
		// 		Point lastMedianPoint = lastCar.medianPoint;
  //               double lastSlope = lastCar.slope;
  //               Point3f lastWorldPoint;
  //               // loaded parameters from the first line
  //               pointToWorld(lastMedianPoint, lastWorldPoint, 
  //                       rvecM1, tvec1, cameraMatrix, s);  
            
  //               Point3f speed_three_dim = (worldPoint -
  //                       lastWorldPoint) / consumeTime * 1000;
		// //cout << "speed_three_dim: " << speed_three_dim << endl;
  //               // filter data from kalman
  //               // add calculated value into Kalman when moving
  //               //if (lastMedianPoint != medianPoint)
  //                 //  slope = getSlope(lastMedianPoint, medianPoint);
  //               filteredSlope = myFilter.getFilteredValue(slope);
		
  //               float speed = sqrt(pow(worldPoint.x - 
  //                   lastWorldPoint.x, 2) + pow(worldPoint.y - 
  //                   lastWorldPoint.y, 2)) / consumeTime * 1000;
  //               cout << "speed: " << speed  << "mm" << endl;
  //               filtered_speed = myFilterSpeed.getFilteredValue(speed);
  //               cout << "filteredSlope: " << filteredSlope << endl;
  //               //cout << "filtered_speed: " << filtered_speed  << "mm" << endl;
  //               deleteCar(*iter, carStateSet);

  //               // TODO
  //               /*
  //                * define the target_speed and target_angel
  //                * global variable of target_angel
  //                * note: target_angel derived from two 2d points,
  //                * you can also try to use 3d, but may it has 
  //                * subtle error.
  //                */
  //               // target_angel = getSlope(medianPoint, test_point);
  //               target_angel = 90;
  //               //cout << "target_angel" << target_angel << endl;
  //               Point3f target_speed = Point3f(-2254.89, -19356.3, 0);
                
  //               // Point test_point(594, 306); 
  //               controlSpeedAndAngular(target_speed, 
  //                       target_angel, speed_three_dim, filteredSlope, marker);
		// 	}
  			
  //   		carStateSet.push_back((*iter));

  //   	  iter ++;
  //  m
	  }

}
