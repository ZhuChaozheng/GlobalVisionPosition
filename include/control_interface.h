
#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <sys/time.h>
#include <signal.h>
#include <opencv2/plot.hpp>
#include "timer.h"
#include "kalman.h"
#include "car.h"
#include "udp.h"
#include "pid.h"
#include "image_process.h"
#include "init_camera.h"
#include "image_transformation.h"
#include "file_operation.h"

using namespace cv;
using namespace std;

enum trajectory_type
{
    RECTANGLE = 1,
    CIRCLE = 2,
    TRIANGLE = 3 
};
// classification of car
vector<Car> car_set;
Timer traTimer;

bool app_stopped = false;
void sigint_handler(int sig);

void StopCars();

/*
 * modify car's slope and sleep, according to 
 * the marker.
 * 
 * 
 */
void UpdateSlope(const float target_slope,
        const int marker);

void ConfigParamtersRead();

void init();
/* 
 * 
 * define multiple trajectory modules, loop 0, 90, 180, 270
 * to form a rectangle. 
 *
 */
void TimerTrajectory(const trajectory_type type, 
        const int marker);
/*
 * Car:
 *     target_point, target_speed, target_slope, stop_flag
 */
void hardware_control_interface(vector<Car> &cars_control_set);

void interface_udp(vector<Car> &cars_control_set);

#endif //CONTROL_INTERFACE_H