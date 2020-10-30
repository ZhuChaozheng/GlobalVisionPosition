#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctype.h>
#include "timer.h"
using namespace std;

void ConfigFileRead( double* param_turn_p, double* param_turn_i, double* param_turn_d,
		double* param_move_p, double* param_move_i, double* param_move_d )
{
    cv::FileStorage fs("../config/configure.yaml", cv::FileStorage::READ);
    fs["TurnP"] >> *param_turn_p;
    fs["TurnI"] >> *param_turn_i;
    fs["TurnD"] >> *param_turn_d;
    fs["MoveP"] >> *param_move_p;
    fs["MoveI"] >> *param_move_i;
    fs["MoveD"] >> *param_move_d;

    fs.release();
    std::cout << "File Read Finished!" << std::endl;
}

void CtrlParametersUpdateTask( double* param_turn_p, double* param_turn_i, double* param_turn_d,
                double* param_move_p, double* param_move_i, double* param_move_d )
{
	ConfigFileRead( param_turn_p, param_turn_i, param_turn_d, param_move_p, param_move_i, param_move_d );
	std::cout << "thread2:" << *param_turn_p << std::endl;
}

int main(int argc, char* argv[])
{
	// intialize timer thread
	Timer timer;
	// initialize parameters for PID controler
	double param_turn_p, param_turn_i, param_turn_d, 
            param_move_p, param_move_i, param_move_d;
	// execute task every 2000 microsecond
	timer.start(2000, std::bind(CtrlParametersUpdateTask,
            &param_turn_p, &param_turn_i, &param_turn_d, 
            &param_move_p, &param_move_i, &param_move_d));
	// your main thread
	while(true){
		int num = 10;	
		cout << param_turn_p << endl;
		std::cout << "thread1: my work is image process" << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	}
	return 0;
}
