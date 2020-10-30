#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
//  #include <opencv2/plot.hpp>
#include "kalman.h"
#include "image_process.h"
#include "init_camera.h"
#include "2d_transformation_3d.h"

using namespace cv;
using namespace std;


int main()
{

	// load parameter from file
	ConfigFileRead(rvecM1, tvec1, cameraMatrix, s);

	clock_t lastTime = clock();

	EnumDevice();
    InitCamera();
    Mat src;
    vector<Car> carStateSet;
    // Initialize Data
    vector<double> sine;
    // Create Ploter
    Mat data(sine);
    // Ptr<plot::Plot2d> plot = plot::createPlot2d(data);
    int t = 0; // cal num

    // kalman
    double filteredSlope;
    //suggested initial values for high noise filtering
    Kalman myFilter(0.125,32,1023,0); 


 //    // namedWindow("src_gray", 0);
	while (1) 
	{
		
	    StartGrabStream(src);
	    // src = imread( "../img/m.png", 1 );
		if (src.empty())
	    {
	    	cout << "could not read the image." << endl;
	    	// continue;
	    	return 0;
	    }
	    // imwrite("w_src.jpg", src);
	    
	    // sleep(0.01);
		// src_gray is global variable
		blur( src, src_gray, Size(1, 1) );
        // imwrite("src_gray.jpg", src_gray);
	    // imshow("src_gray", src_gray);
        
	    vector<Point2f> pointSet;
	    
	    thresh_callback( src_gray, pointSet );
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
	        // if ( newPointSet.size() != 0)
	        //    classificationCar(&newPointSet, 
	        //           &carSet); // carSet -> global variable
	        else
	           break;
	    }

    	// consume time 
		clock_t currentTime = clock();
		double consumeTime = 1000 * (currentTime - lastTime) / (double)CLOCKS_PER_SEC;
		cout << "time" << consumeTime << "ms" << endl;
		cout << "fps: " << 1000 / consumeTime << "Hz" << endl;
		// cout << "carSet: " << carSet.size() << endl;
  	    // output car attribution
  	    for (auto iter = carSet.begin(); iter != carSet.end();)
        {
            getCarKeyAttribution(*iter);
            Point2f medianPoint = (*iter).medianPoint;
            int marker = (*iter).marker;
            double slope = (*iter).slope;
            Point3f targetPoint = (*iter).target;

            cout << "marker: " << marker << endl;
            // if (marker != 5)
            // 	return 0;
            cout << "slope: " << slope << endl;   
            
               
            // plot curve
         //    if (t < 360)
         //    {
		       //  sine.push_back(filteredSlope);
		       //  t++;
         //    }
         //    else
         //    {
         //    	// Rotation Data
		       //  sine.erase(sine.begin());
		       //  sine.push_back(filteredSlope);
         //    }	    
         //    // Render Plot Image
	        // Mat image;
	        // plot->render( image );

	        // // Show Image
	        // imshow("curve", image );
            
            Point3f worldPoint;
            // loaded from the first line
            pointToWorld(medianPoint, worldPoint, rvecM1, 
                   tvec1, cameraMatrix, s);
            // cout << "worldPoint: " << worldPoint / 100 << 
            //         "mm" << endl;
            cout << "medianPoint: " << medianPoint << endl;
            // if (targetPoint.y == 0) 
            // targetPoint = worldPoint + gap_point;
            // cout << "target: " << targetPoint << endl;
            // double angle = get3dSlope(worldPoint, targetPoint);
            // cout << "angle: " << angle << endl;
            Car lastCar;

            if (exist((*iter), carStateSet, lastCar))
            {
                Point2f lastMedianPoint = lastCar.medianPoint;
                double lastSlope = lastCar.slope;

                Point3f lastWorldPoint;
                // loaded parameters from the first line
                pointToWorld(lastMedianPoint, lastWorldPoint, 
                      rvecM1, tvec1, cameraMatrix, s);  

                Point3f accelerate = (lastWorldPoint - 
                      worldPoint) / consumeTime;
                double angularGap = lastSlope - slope;
                double angularVelocity = angularGap / consumeTime; 
                
                if (angularGap < 30)
                {
                    // save last state
                    lastCar.medianPoint = medianPoint;
                    lastCar.slope = slope;
                    //filter data from kalman
                    filteredSlope = myFilter.getFilteredValue(slope);
                    cout << "filteredSlope: " << filteredSlope << endl;
                }
                    
                // cout << "accelerate: " << accelerate / 100 << 
                //       "mm" << endl;
                // cout << "angularVelocity: " << angularVelocity <<
                //       endl;


                
            }
            else
                carStateSet.push_back((*iter));



            iter ++;
        }   
        // update the last state
        lastTime = currentTime;
	    // cout << carStateSet.size() << endl;
	    // return 0;	

	}

}
