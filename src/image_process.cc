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

void rotatedRectROI(const Mat& src, Mat& cropped, 
		RotatedRect rect)
{
	// get angle and size from the bounding box
    float angle = rect.angle;
    Size rect_size = rect.size;
    // thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
    if (rect.angle < -45.) {
        angle += 90.0;
        swap(rect_size.width, rect_size.height);
    }
    // get the rotation matrix
    Mat M, rotated;
    M = getRotationMatrix2D(rect.center, angle, 1.0);
    // perform the affine transformation
    warpAffine(src, rotated, M, src.size(), INTER_CUBIC);
    // crop the resulting image
    getRectSubPix(rotated, rect_size, rect.center, 
            cropped);
}

void thresh_callback(const Mat &src_gray, 
	vector<Point2f> &pointSet)
{
	Mat thresh_output;
	thresh_output = src_gray.clone();
    // find contours
    vector<vector<Point>> contours;
    findContours( src_gray, contours, RETR_TREE, 
            CHAIN_APPROX_SIMPLE);
    cimage = Mat::zeros(thresh_output.size(), CV_8UC3); // global
    Mat barriers_image = Mat::zeros(thresh_output.size(), CV_8UC1);
    vector<RotatedRect> dup_rectangle_set;
    vector<vector<Point>> next_notice_contours;
    for (size_t t = 0; t < contours.size(); t++) 
    {
    	// drawContours(cimage, contours, static_cast<int>(t), 
	    //             Scalar(255, 0, 255), 2, 8);   
        // only select specific area
        RotatedRect rotatedRect = minAreaRect(contours[t]);
        Rect rect = rotatedRect.boundingRect();
        
        // geometry analysis
        float w = rotatedRect.size.width;

        float h = rotatedRect.size.height;
        float areaRect = w * h;
        float rate = w / h;

        double perimeter = arcLength(contours[t], true);
        // the minimum perimeter is 240
        if (perimeter > 230)
        {
        	// barriers
        	drawContours(barriers_image, contours, static_cast<int>(t), 
	                Scalar(255, 0, 255), 2, 8);  
      //   	namedWindow( "barriers_image", 1 );
    		// imshow("barriers_image", barriers_image);
        }
        else if (perimeter > 130)
        // if (perimeter < 130 | perimeter > 230 | rate < 0.85)
        // 	continue;
        {
        	Point2f currentPoint = rotatedRect.center;
	        // TODO
	        // remove the shadow of wheels
	        // remove duplicate center of rectangle
	        if (dup_rectangle_set.size() == 0)
	        {
	        	dup_rectangle_set.push_back(rotatedRect);
	        	continue;
	        }
	        // copy vector
	        vector<RotatedRect> double_dup_rectangle_set;
	        for(auto iter = dup_rectangle_set.begin(); 
	        		iter != dup_rectangle_set.end();)
	        {

	        	double_dup_rectangle_set.push_back(*iter);
	        	iter ++;
	        }
	        /*
	         * here, should be replaced with a special function
	         * bool PointNeighbourVector(Point2f currentPoint, 
	         *  vector<Point2f> double_dup_point_set)
	         *
	         */
	    	bool flag = true;
	        for(auto iter = double_dup_rectangle_set.begin(); 
	        		iter != double_dup_rectangle_set.end();)
	        {
	        	Point2f tempPoint = (*iter).center;	        	
	        	if (!neighbourPoint(tempPoint, currentPoint))
				{
					flag = true;
				}
				else
				{
					flag = false;
					break;
				}
				iter ++;
	        }
		    if (flag)
	        	dup_rectangle_set.push_back(rotatedRect);
	        double_dup_rectangle_set.clear();
        }
        else
        {
        	// next notice points
        	next_notice_contours.push_back(contours[t]);
        	// cout << "next: " << next_notice_contours.size() << endl;
        }
    }
    float radius = 4;
    int i = 0;
    for (size_t t = 0; t < next_notice_contours.size(); t++) 
    {
        double area = contourArea(next_notice_contours[t]);
        RotatedRect rotated_rect = minAreaRect(next_notice_contours[t]);
        // // geometry analysis
        if (area > 2 & area < 40) 
        {
        	for (auto iter = dup_rectangle_set.begin();
		    			iter != dup_rectangle_set.end();)
		    {
		    	// iter is RotatedRect class
		    	// display the position of big square
		    	if (IsPointInRotatedRect((*iter), 
		    				rotated_rect.center))
		        {
		        	// rotated_rect is small square
		        	pointSet.push_back(rotated_rect.center);
		        	circle(cimage, rotated_rect.center, cvRound(radius), 
						Scalar(255, 0, 0), 2, LINE_AA);
		        }
		    	iter ++;
		    }
    	}
    }
    namedWindow( "thresh_output", 0 );
    imshow("thresh_output", cimage);
    waitKey(5);
    // cout << "size(): " << pointSet.size() << endl;
}

float GetCross(Point2f &p1, Point2f &p2, Point2f &p)
{
	return (p2.x - p1.x) * (p.y - p1.y)
			- (p.x - p1.x) * (p2.y - p1.y);
}

bool IsPointInRotatedRect(RotatedRect &rotated_rect, Point2f &p)
{
	Point2f vtx[4];
	rotated_rect.points(vtx);
    // Draw the bounding box
    for( int i = 0; i < 4; i++ )
    {
    	line(cimage, vtx[i], vtx[(i+1)%4], 
        		Scalar(0, 255, 0), 1, LINE_AA);
    }
	return (GetCross(vtx[0], vtx[1], p) * 
			GetCross(vtx[2], vtx[3], p) >= 0 )
			&& (GetCross(vtx[1], vtx[2], p) * 
			GetCross(vtx[3], vtx[0], p) >= 0);
	
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
void classificationCar(vector<Point2f> *pointSet, 
		vector<Car> &car_set)
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
		else
		{
			point ++;
		}
	}
	// cout << "classificationCar: " << point_set.size() << endl;
	int marker = point_set.size() - 3;
	
	if (marker >= 0)
	{
		// search 
		for (auto iter = car_set.begin(); iter != car_set.end();)
		{
			if ((*iter).get_marker() == marker)
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
	vector<Point2f> pointSet = car.get_point_set();	
	if (pointSet.size() != 0)
	{
		double slope = getAbsoluteOrientation(pointSet, car); // stuff a triangle
		car.set_slope(slope);
	}	
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
	// search second max distance between points,
	// equals to another isosceles
	EdgeNode edgeNodeAnother = findMaxDistance(pointSet, 
			maxDistance);
	/*
	 * if the side edge is the longest, then
	 * the bottom edge is third of length.
	 *
	 */
	switch(SIDE)
	{
		case SIDE:
			if (edgeNode.V1 == edgeNodeAnother.V1 |
					edgeNode.V1 == edgeNodeAnother.V2)
			{
				car.set_first(edgeNode.V1);
				car.set_second(edgeNode.V2);
			}
			else
			{
				car.set_first(edgeNode.V2);
				car.set_second(edgeNode.V1);			
			}  
			car.set_third(edgeNodeAnother.V2);
			break;
		case BOTTOM:
			if (edgeNodeAnother.V1 == edgeNode.V1 |
					edgeNodeAnother.V1 == edgeNode.V2)
			{
				car.set_first(edgeNodeAnother.V2);
			}
			else
			{
				car.set_first(edgeNodeAnother.V1);
			}
			car.set_second(edgeNode.V2);
			car.set_third(edgeNode.V1);
			break;
	}
	
}

/*
 * 
 *
 */
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
    car.set_median_point(center);
    Point2f medianPoint = car.get_median_point();
    Point2f first = car.get_first();
    return getSlope(medianPoint, first);
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
    return 1;
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
		if (car.get_marker() == (*iter).get_marker())
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
        if (car.get_marker() == (*iter).get_marker())
        {
            iter = carStateSet.erase(iter);
            return;
        }
        iter ++;
    }
}
