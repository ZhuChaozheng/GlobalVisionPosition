/**
 * created by Jason, Hohai university
 * 2020.12.16
 * core functions of launching sub-timer-thread of updating parameters 
 * of each car and image process in real-time, while calculating
 * the state of each car, setting its abbtributions and send different
 * motion instructions.
 */

#include "control_interface.h"

void sigint_handler(int sig) {
    if (sig == SIGINT) 
    {
        std::cout << "C pressed!" << endl;
        app_stopped = true;
    }
}

void StopCars()
{
    
    /* define comm format*/
    char a[6] = {0x11,0x00,0x00,0x00,0x00,0x22};
    // build communication based on udp
    for (auto iter = car_set.begin(); 
                iter != car_set.end();)
    {
        int marker = (*iter).get_marker();
        string ip = (*iter).get_ip();
        udp udp_comm;
        int sock_fd = udp_comm.udp_init(ip);
        // send data through udp
        udp_comm.send_data(sock_fd, a, sizeof(a));
        cout << marker << " stopped!" << endl;
        iter ++;
    }
    
}

/*
 * modify car's slope and sleep, according to 
 * the marker.
 * 
 * 
 */
void UpdateSlope(const float target_slope,
        const int marker)
{
    for (auto iter = car_set.begin(); 
            iter != car_set.end();)
    {
        int marker_ = (*iter).get_marker();
        if (marker_ == marker)
        {
            (*iter).set_target_slope(target_slope);
            break;
        }
    }
}

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
    float target_slope_; 
    float target_speed_; 
    float target_point_x_, target_point_y_;
    bool flag = false; // match existing car
    // clear old car set
    
    // car_set.erase(car_set.begin(), car_set.end());
    for (int i = 0; i <= 9; i++) {
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

        front_str = "target_point_x_";
        combined_str = front_str + to_string(i);
        fs[combined_str] >> target_point_x_;

        front_str = "target_point_y_";
        combined_str = front_str + to_string(i);
        fs[combined_str] >> target_point_y_;

        // cout << "read target_speed_: " << target_speed_ << endl;
        for (auto iter = car_set.begin(); 
                iter != car_set.end();)
        {
            int marker = (*iter).get_marker();
            if (marker == i)
            {
                (*iter).update_parameters(slope_p_, slope_i_, 
                        slope_d_, speed_p_, speed_i_, 
                        speed_d_, ip_, target_slope_, 
                        target_speed_, target_point_x_, 
                        target_point_y_);
                flag = true;
                break;
            }
            iter ++;
        }
        if (!flag)
        {
            Car *car = new Car(marker_, slope_p_, slope_i_, 
                slope_d_, speed_p_, speed_i_, speed_d_, ip_, 
                port_, target_slope_, target_speed_, 
                target_point_x_, target_point_y_);
            car_set.push_back(*car);
        }
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
    Point2f left_up, right_down;
    ConfigCacheAreaRead(left_up, right_down);
    cout << left_up << endl;
    cout << right_down << endl;
}

/* 
 * 
 * define multiple trajectory modules, loop 0, 90, 180, 270
 * to form a rectangle. 
 *
 *
 */
void TimerTrajectory(const trajectory_type type, 
        const int marker)
{
    switch(type){
        case RECTANGLE:
            // TODO
            // UpdateSlope(0, marker);
            cout << "RECTANGLE!!!!!!!" << endl;
            traTimer.startOnce(0, bind(UpdateSlope, 0, marker));
            traTimer.startOnce(5000, bind(UpdateSlope, 90, marker));
            // traTimer.startOnce(10000, bind(UpdateSlope, 180, marker));
            // traTimer.startOnce(15000, bind(UpdateSlope, 270, marker));
            // timer.startOnce(4000, bind(UpdateSlope, 90, marker));
            // Sleep(2)
            break;
        case CIRCLE:
            // TODO
            UpdateSlope(90, marker);
            break;
        case TRIANGLE:
            // TODO
            traTimer.startOnce(0, bind(UpdateSlope, 0, marker));
            traTimer.startOnce(2000, bind(UpdateSlope, 120, marker));
            traTimer.startOnce(5000, bind(UpdateSlope, 240, marker));
            break;
    }        

}

// stop car 
void StopCar(Car &car)
{
    
    /* define comm format*/
    char a[6] = {0x11,0x00,0x00,0x00,0x00,0x22};
    // build communication based on udp
    string ip = car.get_ip();
    udp udp_comm;
    int sock_fd = udp_comm.udp_init(ip);
    // send data through udp
    udp_comm.send_data(sock_fd, a, sizeof(a));
    int marker = car.get_marker();
    cout << marker << " stopped!" << endl;
}

// convert float to char through memory
void float2char(float source, char *src)
{   
    char *temp;
    char buff[4];
    memset(buff, 0, sizeof(buff));

    for(int i = 0; i < 20; i = i + 4)
    {
        temp = (char*)(&(source));
        src[i] = temp[0];
        src[i + 1] = temp[1];
        src[i + 2] = temp[2];
        src[i + 3] = temp[3];
        cout << i << endl;
    }
}

void char2float(char *src, float *target)
{
    for(int i = 0; i < 200; i = i + 4)
    {
        char temp[4];
        temp[0] = src[i];
        temp[1] = src[i + 1];
        temp[2] = src[i + 2];
        temp[3] = src[i + 3];

        target = (float*)(&temp);
        cout <<"target " << *target << endl;
        cout << "i " << i << endl;
    }
}


void interface_udp(vector<Car> &cars_control_set)
{
    if (cars_control_set.size() != 10)
    {
        cout << "check your size" << endl;
        return;
    }
    vector<float> float_set;
    
    // convert Car vector to float vector
    for (auto iter = cars_control_set.begin(); 
            iter != cars_control_set.end();)
    {           
        Point2f target_point = (*iter).get_target_point();
        float target_slope = (*iter).get_target_slope();
        float target_speed = (*iter).get_target_speed();
        float stop_flag = (*iter).get_stop_flag();
        float_set.push_back(target_point.x);
        float_set.push_back(target_point.y);
        float_set.push_back(target_slope);
        float_set.push_back(target_speed);
        float_set.push_back(stop_flag);
        iter ++;
    }
    // convert float vector to char array, every car is 5*4,
    // the total of ten cars is equal to 5*4*10=200
    char src[200];
    char *temp;
    int i = 0;
    for(auto iter = float_set.begin(); iter != float_set.end();)
    {       
        temp = (char*)(&(*iter));
        src[i] = temp[0];
        src[i + 1] = temp[1];
        src[i + 2] = temp[2];
        src[i + 3] = temp[3];
        i = i + 4;
        iter ++;
    }
    udp udp_comm;
    int sock_fd = udp_comm.udp_init("127.0.0.1");
    // send data through udp
    udp_comm.send_data(sock_fd, src, sizeof(src));
}
/*
 *
 * Car:
 *     target_point, target_speed, target_slope, stop_flag
 *
 */
void hardware_control_interface(vector<Car> &cars_control_set)
{    
    /*
     * according the protocol, the vector's size is 10, each one express
     * a car, and the first of vector is the car of 0 marker. the last of vector
     * is the car of 9 marker.
     *
     * the next code is to map the relational sequence.
     */
    int i = 0;      
    for (auto iter = cars_control_set.begin(); iter != cars_control_set.end();)
    {           
        Point2f target_point = (*iter).get_target_point();
        float target_slope = (*iter).get_target_slope();
        float target_speed = (*iter).get_target_speed();
        cout << "hardware_control_interface: " << target_speed << endl;
        int stop_flag = (*iter).get_stop_flag();
        for (auto car = car_set.begin(); car != car_set.end();)
        {
            int marker = (*car).get_marker();
            if (marker == i)
            {
                (*car).set_target_point(target_point);
                (*car).set_target_slope(target_slope);
                (*car).set_target_speed(target_speed);
                (*car).set_stop_flag(stop_flag);
            }
            car ++;
        }
        i ++;
        iter ++;
    }
}

/*
 * this function to realize navigating the specific point by
 * adjusting continually target slope and judging the distance.
 * Once the distance is less than 5px, the navigation will end 
 * and set the target point is (0, 0) 
 *
 * Also, if you want to close the navigation function, you just
 * set its target point is (0, 0).
 *
 */
void NavigateTargetPoint(Car &car)
{    
    // the center of circumcircle 
    Point2f current_point = car.get_median_point(); 
    cout << "current_point: " << current_point << endl;
    Point2f target_point = car.get_target_point();
    cout << "target_point: " << target_point << endl;
    // exit navigation 
    if (target_point.x == 0 & target_point.y == 0)
        return;
    // the median point between two points
    float orientation_slope = GetSlope(current_point, target_point);
    car.set_target_slope(orientation_slope);
    float distance = GetPixelDistance(current_point, target_point);
    if (distance < 5) // about 10mm, 1px = 2mm
    {
        cout << "Get Goal!********************************************" << endl;
        car.set_target_point(Point2f(0, 0));
        car.set_target_speed(0);
        StopCar(car);
        return;
    }
}
void* say_hello(void* args)
{
    udp udp_server;
    udp_server.udp_server_init();
    return 0;
}

int main()
{
    // pthread_t tids[1];
    // int ret = pthread_create(&tids[0], NULL, say_hello, NULL);
    // if (ret != 0)
    // {
    //    cout << "pthread_create error: error_code=" << ret << endl;
    // }
    udp udp_server;
    udp_server.udp_server_init();
    // TimerTrajectory(RECTANGLE, 5);
    // ***** dynamicly tune parameter thread ********
    // intialize timer thread
    // Timer timer;
    // float target_slope_=0;
    // float time_=0;
    // // // execute task every 2000 microsecond
    // // timer.start(2000, std::bind(ConfigParamtersRead));
    // ConfigParamtersRead();
    // // *********** main thread ******************
    // // register signal ctrl+c
    // signal(SIGINT, sigint_handler);
    // // clock_t lastTime = clock();
    // struct timeval lastTime, currentTime;
    // float consumeTime;
    // gettimeofday(&lastTime,NULL);
    // vector<Car> carStateSet;
    // init();
    
    // // Initialize Data
    // // define the angle range of 0-360
    // // vector<float> sine_angle;
    // // for( int t = 0; t < 100; t++ ){
    // //     sine_angle.push_back( (rand() % 360));
    // // }
    // // // Create Ploter
    // // cv::Mat data_angle( sine_angle );
    // // Ptr<cv::plot::Plot2d> plot_angle = 
    // //         cv::plot::Plot2d::create( data_angle );

    // // // define the speed range of 0-100
    // // vector<float> sine_speed;
    // // for( int t = 0; t < 100; t++ ){
    // //     sine_speed.push_back( (rand() % 300));
    // // }
    // // // Create Ploter
    // // cv::Mat data_speed( sine_speed );
    // // Ptr<cv::plot::Plot2d> plot_speed = 
    // //         cv::plot::Plot2d::create( data_speed );

    // // // define the manhattan distance range of 0-100
    // // vector<float> sine_manhattan_distance;
    // // for( int t = 0; t < 100; t++ ){
    // //     sine_manhattan_distance.push_back( (rand() % 300));
    // // }
    // // // Create Ploter
    // // cv::Mat data_manhattan_distance( sine_manhattan_distance );
    // // Ptr<cv::plot::Plot2d> plot_manhattan_distance = 
    // //         cv::plot::Plot2d::create( data_manhattan_distance );
    
    // // init parameter of pid
    // // ConfigParamtersRead();
    // // return 0;
    // // Kalman
    // Kalman myFilterSpeed = Kalman(0.125,32,1023,0);
    // float filtered_speed; 
    // float accelatespeed =0;
    // Mat src;
    // Mat src_thresh;
    // float sum_time;
    // /*
    //  * Car:
    //  *     Point2f target_point, 
    //  *     float target_speed, 
    //  *     float target_slope, 
    //  *     int stop_flag
    //  */
    // Point2f target_point = Point2f(0, 0);
    // float target_speed = 0;
    // float target_slope = 47;
    // int stop_flag = 0;
    // Car car;
    // car.set_target_point(target_point);
    // car.set_target_speed(target_speed);
    // car.set_target_slope(target_slope);
    // car.set_stop_flag(stop_flag);
    // vector<Car> cars_control_set;
    // for(int i = 0; i <= 9; i ++)
    //     cars_control_set.push_back(car);
    // cout << cars_control_set.size() << endl;
    // hardware_control_interface(cars_control_set);
    // while (1) 
    // {
        
    //     if (app_stopped) break;
    //     StartGrabStream(src);
    //     if (src.empty())
    //     {
    //         cout << "could not read the image." << endl;
    //         return 0;
    //     }
    //     // TODO
    //     // warpAffine
    //     // Mat warp_rotate_dst;
    //     // cout << cameraMatrix << endl;
    //     // // warpAffine( src, warp_rotate_dst, cameraMatrix, src.size() );
    //     // warpAffine(src, warp_rotate_dst, cameraMatrix, src.size(), INTER_CUBIC);
    //     // imwrite("warp_dst.jpg", src);
    //     // imwrite("warp_rotate_dst.jpg", warp_rotate_dst);
    //     // return 0;
    //     threshold(src, src_thresh, 80, 255, THRESH_BINARY);
    //      // namedWindow( "src_thresh", 0 );
    //      // imshow("src_thresh", src_thresh);

    //     vector<Point2f> pointSet;
    //     ThreshCallBack(src_thresh, pointSet);
    //     float difference = 0;
    //     int num = 0;
    //     // loop all the point to classify them 
    //     for(;;) 
    //     {
    //         if (pointSet.size() != 0)
    //         {
    //             ClassificationCar(&pointSet, 
    //                   car_set); // carSet -> global variable
    //         }
    //         else
    //            break;
    //     }
    //     // output car attribution
    //     for (auto iter = car_set.begin(); iter != car_set.end();)
    //     {            
    //         GetCarKeyAttribution(*iter);
    //         Point2f medianPoint = (*iter).get_median_point();
    //         int marker = (*iter).get_marker();
    //         float slope = (*iter).get_slope();
    //         // Point3f targetPoint = (*iter).get_target();
            
    //         cout << "marker: " << marker << endl;
    //         cout << "slope: " << slope << endl;
    //         float filteredSlope = (*iter).filter_.getFilteredValue(slope);
    //         // cout << "filteredSlope: " << filteredSlope << endl;
    //         // plot curve
    //         // sine_angle.erase(sine_angle.begin());
    //         // sine_angle.push_back(slope);   
    //         // Render Plot Image
    //         //Mat image;
    //         //plot_angle->render( image );
    //         // Show Image
    //         //imshow("curve_angle", image );
    //         //waitKey(5);
    //         float target_speed = (*iter).get_target_speed();
    //         cout << "target_speed: " << target_speed << endl;
    //         Point2f target_point = (*iter).get_target_point();
    //         cout << "target_point: " << target_point << endl; 
    //         float target_slope = (*iter).get_target_slope();
    //         cout << "target_slope: " << target_slope << endl;
    //         // plot standard curve
    //         // sine_manhattan_distance.erase(sine_manhattan_distance.begin());
    //         // sine_manhattan_distance.push_back(target_speed);        
    //         // // Render Plot Image
    //         // Mat image_manhattan_distance;
    //         // plot_manhattan_distance->render( image_manhattan_distance );
    //         // // Show Image
    //         // imshow("curve_standard_speed", image_manhattan_distance ); 
    //         cout << "medianPoint: " << medianPoint << endl;
    //         // loaded from the first line
    //         // Point3f worldPoint;
    //         // PointToWorld(medianPoint, worldPoint, rvecM1, 
    //         //              tvec1, cameraMatrix, s);
    //         /****************** Navigation Step ***************************/
    //         NavigateTargetPoint(*iter);
    //         /************************* end Navigation *****************/
    //         Car lastCar;

    //         if (Exist((*iter), carStateSet, lastCar))
    //         {
    //             Point2f lastMedianPoint = lastCar.get_median_point();
    //             cout << "lastMedianPoint: " << lastMedianPoint << endl;
    //             float lastSlope = lastCar.get_slope();
    //             float lastspeed = (*iter).get_speed();
    //             // Point3f lastWorldPoint;
    //             // loaded parameters from the first line
    //             // PointToWorld(lastMedianPoint, lastWorldPoint, 
    //             //         rvecM1, tvec1, cameraMatrix, s);  
            
    //             // Point3f speed_three_dim = (worldPoint -
    //             //         lastWorldPoint) / consumeTime * 1000;
    //             //cout << "speed_three_dim: " << speed_three_dim << endl;
    //             // filter data from kalman
    //             // add calculated value into Kalman when moving
    //             // if (lastMedianPoint != medianPoint)
    //             //    slope = getSlope(lastMedianPoint, medianPoint);
    //             // filteredSlope = myFilter.getFilteredValue(slope);
    //             float speed = sqrt(pow(medianPoint.x - 
    //                 lastMedianPoint.x, 2) + pow(medianPoint.y - 
    //                 lastMedianPoint.y, 2)) / consumeTime;
    //             difference =speed-lastspeed;
    //             //cout << "speed-lastspeed : " << difference  << "px/s" << endl;
    //             // float speed = sqrt(pow(worldPoint.x - 
    //             //     lastWorldPoint.x, 2) + pow(worldPoint.y - 
    //             //     lastWorldPoint.y, 2)) / consumeTime * 1000;
    //             //cout << "speed: " << speed  << "px/s" << endl;                
    //             (*iter).set_speed(speed);
    //             // filtered_speed = myFilterSpeed.getFilteredValue(speed);
    //             // cout << "filtered_speed: " << filtered_speed << "px/s" << endl;
    //             // plot curve
    //             // sine_speed.erase(sine_speed.begin());
    //             // sine_speed.push_back(filtered_speed);        
    //             // Render Plot Image
    //             // Mat image_speed;
    //             // plot_speed->render( image_speed );
    //             // // Show Image
    //             // imshow("curve_speed", image_speed ); 
                
    //             DeleteCar(*iter, carStateSet);

    //             // TODO
    //             /*
    //              * define the target_speed and target_angel
    //              * global variable of target_angel
    //              * note: target_angel derived from two 2d points,
    //              * you can also try to use 3d, but may it has 
    //              * subtle error.
    //              */
    //             pid pid_control;
    //             pid_control.controlSpeedAndAngular(*iter);
    //         }
    //         carStateSet.push_back((*iter));
    //         iter ++;
          
    //     }   
    //     // consume time 
    //     // clock_t currentTime = clock();
    //     cout << "lastTime: " << lastTime.tv_usec << endl;
    //     gettimeofday(&currentTime, NULL);
    //     consumeTime = (currentTime.tv_sec - lastTime.tv_sec) + 
    //             (float)(currentTime.tv_usec - lastTime.tv_usec) / 1000000.0;
    //     cout << "time" << consumeTime << "s" << endl;
    //     cout << "fps: " << 1 / consumeTime << "Hz" << endl;
    //     time_+=consumeTime;
    //     cout << "time_" << time_ << "s" << endl;
    //     // if((accelatespeed<=difference/consumeTime) && difference>0 && difference/consumeTime<600)
    //     //    {
    //     //         accelatespeed=fabs(difference)/consumeTime;
    //     //    } 
    //     //  cout << " accelatespeed : " <<  accelatespeed  << "px/s^2" << endl;
    //     // update the last state
    //     lastTime = currentTime;
    //     cout << "lastTime: " << lastTime.tv_usec << endl;
    // }
    // // send stop command to cars
    // StopCars();

    return 0;

}