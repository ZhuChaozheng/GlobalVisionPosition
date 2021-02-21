#include "file_operation.h"

void ConfigCacheAreaRead(Point2f &left_up, Point2f &right_down)
{
    FileStorage fs("../config/cache_area.yaml", FileStorage::READ);
    float cache_area_left_x, cache_area_left_y, cache_area_right_x,
            cache_area_right_y;    
    fs["cache_area_left_x"] >> cache_area_left_x;
    fs["cache_area_left_y"] >> cache_area_left_y;
    fs["cache_area_right_x"] >> cache_area_right_x;
    fs["cache_area_right_y"] >> cache_area_right_y;
    fs.release();
    cout << "File Read Finished!" << endl;
    left_up = Point2f(cache_area_left_x, cache_area_left_y);
    right_down = Point2f(cache_area_right_x, cache_area_right_y);
}

void ConfigFileRead(Mat rvecM1, Mat tvec1, Mat cameraMatrix, float s)
{
    FileStorage fs("../config/camera_intrinsic.yaml", FileStorage::READ);
    
    fs["rvecM1"] >> rvecM1;
    fs["tvec1"] >> tvec1;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["s"] >> s;
    cout << "file s: " << s;
    fs.release();
    cout << "File Read Finished!" << endl;
}

void ConfigFileWrite(const Mat& rvecM1, const Mat& tvec1,
                     const Mat& cameraMatrix, const float& s)
{
    // initilization
    FileStorage fs("../config/configure.yaml", FileStorage::WRITE);
    
    // start write
    // cameraMatrix = (Mat_<float>(3, 3) << 1000,
    //     0, 320, 0, 1000, 240, 0, 0, 1);
    // distCoeffs = (Mat_<float>(5, 1) << 0.1,
    //     0.01, -0.001, 0, 0);
    fs << "rvecM1" << rvecM1
       << "tvec1" << tvec1 
       << "cameraMatrix" << cameraMatrix
       << "s" << s;
    
    fs.release();

    cout << "file write finished!" << endl;
}

