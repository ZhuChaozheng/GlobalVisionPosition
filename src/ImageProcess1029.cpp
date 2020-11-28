#include "image_process.h"

Car::Car(int marker, float P, float I, float D, 
			string ip, int port)
{ 
	marker_ = marker; p_ = P; 
	i_ = I; d_ = D; ip_ = ip; 
	port_ = port;
}

Car::Car() {}

Point2f getAveragePoint(vector<Point2f>& pointSet)
{
	Point2f averagePoint;
	double sumX;
	double sumY;
	int num = 0;

	for (auto iter = pointSet.begin(); iter != pointSet.end();)
	{
		sumX += (*iter).x;
		sumY += (*iter).y;
		iter ++;
		num ++;
	}
	averagePoint.x = sumX / num;
	averagePoint.y = sumY / num;

	return averagePoint;
}

// void thresh_callback(const Mat& thresh_output, 
// 	vector<Point2f>& newPointSet)
// {
//     // find contours
//     // vector<Point2f> newPointSet;
//     vector<vector<Point>> contours;
//     findContours( thresh_output, contours, RETR_TREE, 
//             CHAIN_APPROX_SIMPLE);
//     vector<Point2f> pointSet;
//     cimage = Mat::zeros(thresh_output.size(), CV_8UC3); // global
//     for (size_t t = 0; t < contours.size(); t++) 
//     {
//         double area = contourArea(contours[t]);
        
//         // only select specific area
//         // cout << area << endl;
//         // if (area < 4000 | area > 4500) continue;
//         RotatedRect rect = minAreaRect(contours[t]);
//         // geometry analysis
//         float w = rect.size.width;
//         float h = rect.size.height;
//         float rate = w / h;
//         if (rate > 0.85 & area > 2 & area < 20) {
// 	        cout << area << endl;
// 	        drawContours(cimage, contours, static_cast<int>(t), 
// 	                Scalar(0, 0, 255), 2, 8);
// 	        pointSet.push_back(rect.center);
//     	}
//     }
//     imshow("thresh_output", cimage);
//     waitKey(20);
//     refine_point_set(pointSet, newPointSet);
// }

void thresh_callback(const Mat& src_gray, 
	vector<Point2f>& newPointSet)
{
    Mat canny_output;
    Canny( src_gray, canny_output, thresh, thresh * 2 );
   //  cout << "finish imwrite canny_output.jpg" << endl;
  	imshow("canny_output", canny_output);
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(canny_output, contours, hierarchy, 
    		RETR_CCOMP, CHAIN_APPROX_SIMPLE);
    
    vector<Point2f> pointSet;	
    cimage = Mat::zeros(canny_output.size(), CV_8UC3); // global
    float radius = 0;
 //    for(size_t i = 0; i < contours.size(); i++)
	// {
 //  	    size_t count = contours[i].size();
 //  	    // cout << "count: " << count << endl;
 //  	    if( count < 6 )
 //  	        continue;
 //  	    Mat pointsf;
 //  	    Mat(contours[i]).convertTo(pointsf, CV_32F);
 //  	    RotatedRect box = fitEllipse(pointsf);
 //  	    // if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*30 )
 //  	    //     continue;
 //  	    Point2f vtx[4];
 //  	    box.points(vtx);
 //  	    int areaRect = box.size.width * box.size.height;
 //  	    // cout << "areaRect: " << areaRect << endl;
 //  	    // remove large area
 //  	    if (areaRect > 50)
 //  	    	  continue; 
 //  	    for( int j = 0; j < 4; j++ )
 //  	        line(cimage, vtx[j], vtx[(j+1)%4], Scalar(0,255,0), 0.2, LINE_AA);
 //  	    pointSet.push_back(box.center);
	// }
	for (size_t t = 0; t < contours.size(); t++) 
    {
        double area = contourArea(contours[t]);
        
        // only select specific area
        // cout << area << endl;
        // if (area < 4000 | area > 4500) continue;
        RotatedRect rect = minAreaRect(contours[t]);
        // geometry analysis
        float w = rect.size.width;
        float h = rect.size.height;
        float rate = w / h;
        if (rate > 0.85 & area > 2 & area < 15) {
	        cout << area << endl;
	        drawContours(cimage, contours, static_cast<int>(t), 
	                Scalar(0, 0, 255), 2, 8);
	        pointSet.push_back(rect.center);
    	}
    }
    imshow("thresh_output", cimage);
    waitKey(20);
  //	cout << pointSet.size() << endl;

    refine_point_set(pointSet, newPointSet);
}

/**
 * return num of equal pixelValue in vector Pointset and 
 * averagePoint
 */
int countNum(int pixelValue, vector<PointAttri> pointSet)
{
	int num = 0;
	for (auto pointAttri = pointSet.begin(); pointAttri != pointSet.end();)
	{
		// do some optimal operations through 
		// terminating ahead of time
		if (pixelValue == (*pointAttri).value)
		{
			num ++;
		}
		else
		{
			if (num > 0)
			{
				break;
			}
			
		}
		pointAttri ++;
	}

	return num;
}


void deletePointAttriValue(int pixelValue, vector<PointAttri>* pointSet)
{
	for (auto pointAttri = (*pointSet).begin(); pointAttri != (*pointSet).end();)
	{
		if (pixelValue == (*pointAttri).value)
		{
			pointAttri = (*pointSet).erase(pointAttri);
		}
		else
		{
			pointAttri ++;
		}
	}
}

// return w((x_1 - x_2)^2 + (y_1 - y_2)^2)^(1/2)
double getPixelDistance(Point2f pointA, Point2f pointB)
{
    return sqrt((pointA.x - pointB.x) * (pointA.x - pointB.x) 
    	+ (pointA.y - pointB.y) * (pointA.y - pointB.y));
}

/**
 *
 * input: pointSet
 * output: car_set
 * based on the distance among blocks, 35 is the max distance of car
 *
 */
void classificationCar(vector<Point2f>* pointSet, 
		vector<Car>& car_set)
{	
	// if you remount the camera, may be this parameter should be tuned
    double ridus = 35; // mini distance among color blocks of car
    vector<Point2f> point_set;
    int num = 0;
    Point2f firstPoint;   
    
    for (auto point = (*pointSet).begin(); 
    		point != (*pointSet).end();)
	{
		// flag of initialization
		if (num == 0) 
		{
			firstPoint = *point;
			num = 1;
			continue;
		}

		Point2f tempPoint = *point;
		// cout << tempPoint << endl;
		double distance = getPixelDistance(firstPoint, tempPoint);	
		// cout << distance << endl;
		if (distance < ridus) 
		{
			point_set.push_back(tempPoint);
			// delete point
			point = (*pointSet).erase(point);
		}
		else{
			point ++;
		}
	}
	// cout << "classificationCar: " << point_set.size() << endl;
	int marker = point_set.size() - 3;
	
	if (marker >= 0)
	{
		// cout << "marker: " << marker << endl;
		// search 
		for (auto iter = car_set.begin(); iter != car_set.end();)
		{
			if ((*iter).marker_ == marker)
			{
				// clear old pointset
				(*iter).pointSet.clear();
				// copy set from A to B
				for(auto iterator = point_set.begin(); 
						iterator != point_set.end();)
				{
					(*iter).pointSet.push_back(*iterator);
					iterator ++;
				}
				// cout << "car size: " << (*iter).pointSet.size() << endl;
			}
			iter ++;
		}
	}
}

int findKeyPoint(Point2f& point, Point2f& tempPoint, vector<Point2f>& pointSet)
{	
	for (auto iter = pointSet.begin(); iter != pointSet.end();)
	{
		tempPoint = *iter;
		double distance = getPixelDistance(point, tempPoint);
		
		if (distance < MAX_DISTANCE & distance > MIN_DISTANCE)
			return 0;
		iter ++;
	}
	// no proper
	return 1;
}

// return absolute orientation
// getSlope(center, vertex);
double getSlope(Point2f first, Point2f second)
{
	// cout << first << endl;
	// cout << second << endl;
	// double param = double(second.x - first.x) / double(first.y - second.y);	
	float y = first.y - second.y;
	float x = second.x - first.x;
	// cout << "getSlope " << float(y/x) << endl;	
	// distinguish atan & atan2
	double slope = atan2(y, x) * 180 / PI;
	// 0~360
	if (slope < 0)
		slope = 360 + slope;
  	return slope;
}

double get3dSlope(Point3f first, Point3f second)
{
	// double param = double(second.x - first.x) / double(first.y - second.y);	
	float y = float(first.y - second.y);
	float x = float(second.x - first.x);
	// cout << y << endl;
	// cout << x << endl;
	// cout << "getSlope " << float(y/x) << endl;
	// distinguish atan & atan2
	double slope = atan2(y, x) * 180 / PI;
	// 0~360
	if (slope < 0)
		slope = 360 + slope;
  	return slope;
}

Point2f getmedianPoint(Point2f first, Point2f second)
{
	Point2f point;
	point.x = (first.x + second.x ) / 2;
	point.y = (first.y + second.y ) / 2;
	return point;
}

void getCarKeyAttribution(Car& car)
{
	vector<Point2f> pointSet = car.pointSet;
	car.marker_ = car.pointSet.size() - 3;
	
	if (pointSet.size() != 0)
	{
		car.slope = getAbsoluteOrientation(pointSet, car); // stuff a triangle
		// cout << car.second << endl;
		// cout << car.third << endl;
		// car.medianPoint = getmedianPoint(
		// 		car.second, car.third);
		// cout << "medianPoint: " <<  car.medianPoint << endl;
	}
	else
		cout << "size error in getCarKeyAttribution" << endl;
	
}

/**
 * define a undirected graph to save 
 * maximum distance of two vertex
 *
 */
EdgeNode findMaxDistance(const vector<Point2f>& pointSet, 
		double maxDistance)
{
	double tempDistance = 0;
	EdgeNode edgeNode;
	// int num = 0;
	for (auto iter = pointSet.begin(); iter != pointSet.end();)
	{
		Point2f currentPoint = *iter;
		auto iterTemp = iter;
		for (iterTemp++; iterTemp != pointSet.end();)
		{

			Point2f tempPoint = *iterTemp;
			double distance = getPixelDistance(
					currentPoint, tempPoint);
			if ((tempDistance < distance) & 
					(distance != maxDistance)) 
			{
				tempDistance = distance;
				// save in struct				
				edgeNode.V1 = currentPoint;
				edgeNode.V2 = tempPoint;
				// cout << "V2: " << tempPoint << endl;
				edgeNode.Weight = tempDistance;

			}
			iterTemp ++;
		}
		iter ++;
	}
	return edgeNode;
}
	

/**
 * find vertex in points
 * 
 * select top two distance edge, 
 * then determine vertex of triangle
 *
 */
void determineTriangleVertex(vector<Point2f>& pointSet, Car& car)
{
	// search max distance between points, 
	// equals to one isosceles
	EdgeNode edgeNode = findMaxDistance(pointSet, 0);
	double maxDistance = edgeNode.Weight;
	cout << maxDistance << endl;
	// search second max distance between points,
	// equals to another isosceles
	EdgeNode edgeNodeAnother = findMaxDistance(pointSet, 
			maxDistance);
	cout << edgeNodeAnother.Weight << endl;

	if (edgeNode.V1 == edgeNodeAnother.V1 | 
		edgeNode.V2 == edgeNodeAnother.V1)
	{
		car.first = edgeNodeAnother.V2;
		car.second = edgeNode.V2;
		car.third = edgeNode.V1;
	}
	else
	{
		car.first = edgeNodeAnother.V1;
		car.second = edgeNode.V1;
		car.third = edgeNode.V2;
	}    
}

double getAbsoluteOrientation(vector<Point2f>& pointSet,
		Car& car)
{
	if (pointSet.size() == 0)
		return 0;
	determineTriangleVertex(pointSet, car);
    // Find the minimum area enclosing circle
    Point2f center;
    float radius = 0;
    minEnclosingCircle(pointSet, center, radius);
    car.medianPoint = center;
    return getSlope(center, car.first);
}

Point2f getCentroid(Car car)
{
	Point2f centroid;
	double sumX;
	double sumY;
	int num = 0;

	for (auto iter = car.pointSet.begin(); iter != car.pointSet.end();)
	{
		sumX += (*iter).x;
		sumY += (*iter).y;
		iter ++;
		num ++;
	}
	centroid.x = sumX / num;
	centroid.y = sumY / num;

	return centroid;
}


// from distance to judge the neigbhouring points
bool neighbourPoint(Point2f pointA, Point2f pointB)
{
	bool flag = true;
	double distance = getPixelDistance(pointA, pointB);
	if (distance < 4)
		return flag;
	flag = false;
	return flag;
}

bool cmp(const Point2f a, const Point2f b){
	if (a.x > b.x)
		return 0;
	if (a.y > b.y)
		return 0;
    // return a.x==b.x?a.y>b.y:a.x>b.x;
    return 1;
}

/**
 *
 * delete the repeat items and neighbour point
 *
 * first, sort point set
 * second, remove repeat items and neighbour point
 *
 */
void refine_point_set(const vector<Point2f>& pointSet, 
		vector<Point2f>& newPointSet)
{
	// convert to vector<point> type for sorting 
	vector<point> points;
	for (auto iter = pointSet.begin(); 
			iter != pointSet.end();)
	{
		Point2f currentPoint = *iter;
		// cout << currentPoint << endl;
		point my(currentPoint.x, currentPoint.y);
		points.push_back(my);
		iter ++;
	}
	sort(points.begin(), points.end(), less_second);

	// after sorting, convert to vector<Point>
	vector<Point2f> resortPointSet;
	for(int i = 0 ; i < points.size(); i ++) 
	{
        // cout<<"("<<points[i].first<<","<<points[i].second<<")\n";
        Point2f tempPoint = Point2f(points[i].first, 
        		points[i].second);
        resortPointSet.push_back(tempPoint);
	}

    for (auto iter = resortPointSet.begin(); 
			iter != resortPointSet.end();)
	{
		Point2f currentPoint = *iter;
		auto iterTemp = iter;
		bool flag = true;
		// cout << currentPoint << endl;
		for (iterTemp++; iterTemp != resortPointSet.end();)
		{
			Point2f tempPoint = *iterTemp;

			if (neighbourPoint(tempPoint, currentPoint)) {
				flag = false;
				// cout << "currentPoint" << currentPoint << " "
				// 		<< tempPoint << endl;
			}
			iterTemp ++;
		}
		double radius = 1;
		if (flag == true)
		{
			newPointSet.push_back(currentPoint);
			// cout << currentPoint << endl;
			// circle(cimage, currentPoint, cvRound(radius), 
			// 	Scalar(255, 255, 255), 1, LINE_AA);
		}
		iter ++;
	}
	cout << "refine: " << newPointSet.size() << endl;
   //  cout << "finish imwrite w.jpg" << endl;
  	// imwrite("w.jpg", cimage);
}


/**
 *
 * search the corresponding car in the set of car
 * output data: Car lastCar
 *
 */
bool exist(Car& car, vector<Car>& carStateSet, Car& lastCar)
{
	for(auto iter = carStateSet.begin(); 
		iter != carStateSet.end();)
	{
		if (car.marker_ == (*iter).marker_)
		{
			lastCar = *iter;
			return true;
		}

		iter ++;
	}
	return false;
}


void deleteCar(Car& car, vector<Car>& carStateSet)
{
    for(auto iter = carStateSet.begin(); 
            iter != carStateSet.end();)
    {
        if (car.marker_ == (*iter).marker_)
        {
            iter = carStateSet.erase(iter);
            return;
        }
        iter ++;
    }
}
