#include "image_process.h"

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

void thresh_callback(const Mat& src_gray, 
	vector<Point2f>& newPointSet)
{
    Mat canny_output;
    Canny( src_gray, canny_output, thresh, thresh * 2 );
    cout << "finish imwrite canny_output.jpg" << endl;
  	imwrite("canny_output.jpg", canny_output);
    vector<vector<Point>> contours;
    findContours( canny_output, contours, RETR_TREE, 
    		CHAIN_APPROX_SIMPLE);
    vector<Point2f> pointSet;
    cimage = Mat::zeros(canny_output.size(), CV_8UC3); // global
    float radius = 0;
    for(size_t i = 0; i < contours.size(); i++)
	{
  	    size_t count = contours[i].size();
  	    // cout << "count: " << count << endl;
  	    if( count < 6 )
  	        continue;
  	    Mat pointsf;
  	    Mat(contours[i]).convertTo(pointsf, CV_32F);
  	    RotatedRect box = fitEllipse(pointsf);
  	    // if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*30 )
  	    //     continue;
  	    Point2f vtx[4];
  	    box.points(vtx);
  	    int areaRect = box.size.width * box.size.height;
  	    	cout << "areaRect: " << areaRect << endl;
  	    // remove large area
  	    if (areaRect > 30)
  	    	  continue; 
  	    for( int j = 0; j < 4; j++ )
  	        line(cimage, vtx[j], vtx[(j+1)%4], Scalar(0,255,0), 0.2, LINE_AA);
  	    pointSet.push_back(box.center);
	}
	
  	cout << pointSet.size() << endl;

    refine_point_set(pointSet, newPointSet);
}

// find big enclosing rectangle and its corresponding center
// draw a circle of enclosing rectangle.

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
 * output: carSet
 * based on the distance among blocks, 35 is the max distance of car
 *
 */
void classificationCar(vector<Point2f>* pointSet, 
		vector<Car>* carSet)
{	
	// if you remount the camera, may be this parameter should be tuned
    double ridus = 35; // mini distance among color blocks of car
    Car car;
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
			car.pointSet.push_back(tempPoint);
			// delete point
			point = (*pointSet).erase(point);
		}
		else{
			point ++;
		}
	}
	// cout << "classificationCar: " << car.pointSet.size() << endl;
	if (car.pointSet.size() >= 3)
		(*carSet).push_back(car);
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
double getSlope(Point first, Point second)
{
	// cout << first << endl;
	// cout << second << endl;
	// double param = double(second.x - first.x) / double(first.y - second.y);	
	float y = float(first.y - second.y);
	float x = float(second.x - first.x);
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
	car.marker = car.pointSet.size() - 3;
	
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
	// cout << maxDistance << endl;
	// search second max distance between points,
	// equals to another isosceles
	EdgeNode edgeNodeAnother = findMaxDistance(pointSet, 
			maxDistance);
	// cout << edgeNodeAnother.Weight << endl;

	if (edgeNode.V1 == edgeNodeAnother.V1)
	{
		car.first = edgeNode.V1;
		car.second = edgeNode.V2;
		car.third = edgeNodeAnother.V2;
	}
	else if (edgeNode.V1 == edgeNodeAnother.V2)
	{
		car.first = edgeNode.V1;
		car.second = edgeNodeAnother.V1;
		car.third = edgeNode.V2;
	}
	else if (edgeNode.V2 == edgeNodeAnother.V1)
	{
		car.first = edgeNode.V2;
		car.second = edgeNode.V1;
		car.third = edgeNodeAnother.V2;
	}
	else
	{
		car.first = edgeNode.V2;
		car.second = edgeNode.V1;
		car.third = edgeNodeAnother.V2;
	}    
}

double getAbsoluteOrientation(vector<Point2f>& pointSet,
		Car& car)
{
	if (pointSet.size() == 0)
		return 0;
	determineTriangleVertex(pointSet, car);
//	cout << car.first << endl;
	Point vertex;
	vertex.x = int(car.first.x);
	vertex.y = int(car.first.y);
//	cout << vertex << endl;
    // Find the minimum area enclosing circle
    Point2f center;
    float radius = 0;
    minEnclosingCircle(pointSet, center, radius);
    Point centerInt;
    centerInt.x = int(center.x);
    centerInt.y = int(center.y);
  //  cout << center << endl;
   // cout << centerInt << endl;
    car.medianPoint = centerInt;
    // cout << "center: " << center << endl;
    return getSlope(centerInt, vertex);
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
			circle(cimage, currentPoint, cvRound(radius), 
				Scalar(255, 255, 255), 1, LINE_AA);
		}
		iter ++;
	}
	// cout << "refine: " << newPointSet.size() << endl;
 //    cout << "finish imwrite w.jpg" << endl;
 //  	imwrite("w.jpg", cimage);
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
		if (car.marker == (*iter).marker)
		{
			lastCar = *iter;
			return true;
		}

		iter ++;
	}
	return false;
}
