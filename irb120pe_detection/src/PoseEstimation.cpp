#include <iostream>
#include <numbers>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include "rclcpp/rclcpp.hpp"


using namespace cv;
using namespace std;

float get_angle(Point2f O, Point2f P, int rows);
Point2f get_intersection_Point(Point2f O, Point2f P, Point2f R, Point2f S);
Point2f get_Transform2WR(Point2f O, int rows);



int main(int argc, char * argv[])
{

	
	rclcpp::init(argc, argv);

	Mat imgHSV, mask, outputImg;
	vector<int> params1;
	vector<vector<Point>> contours;

	/*Limits of the HSV mask*/
	Scalar lowLimit(0, 100, 100);
	Scalar upLimit(20, 255, 255);

	params1.push_back(IMWRITE_JPEG_QUALITY);
	params1.push_back(95);



	string filepath = "/src/ROI2.jpg";
	string packagepath = ament_index_cpp::get_package_share_directory("irb120pe_detection");
	string path = packagepath + filepath;

 	Mat inputImg = imread(path, -1);


	if (inputImg.empty())
	{
		//Error
		return 0;
	}
	//resize(inputImg, inputImg, Size(640, 640));

	inputImg.copyTo(outputImg);

	cvtColor(inputImg, imgHSV, COLOR_BGR2HSV);

	inRange(imgHSV, lowLimit, upLimit, mask);

	findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	vector<vector<Point>> polyContours(contours.size());
	int max = 0;
	int position = 0;
	for (int i = 0;i < contours.size();i++) {
		if (contourArea(contours[i]) > max) {
			max = contourArea(contours[i]);
			position = i;
		}
	}

	for (int i = 0;i < contours.size();i++) { //Check all the contours in the contours vector
		float perimeter = arcLength(contours[i], true); //Get the perimeter
		approxPolyDP(contours[i], polyContours[i], 0.1 * perimeter, true); //Use the perimeter to give a better approximation of a polygonal shape
	}

	bool check = true;
	float epsilon = 0.1;

	while (check) {

		if (polyContours[position].size() == 4) {
			check = false;
		}
		else if (polyContours[position].size() > 4) {
			epsilon = 10 * epsilon;
			polyContours.clear();
			for (int i = 0;i < contours.size();i++) { //Check all the contours in the contours vector
				float perimeter = arcLength(contours[i], true); //Get the perimeter
				approxPolyDP(contours[i], polyContours[i], epsilon * perimeter, true); //Use the perimeter to give a better approximation of a polygonal shape
			}
		}
		else {
			epsilon = 0.1 * epsilon;
			polyContours.clear();
			for (int i = 0;i < contours.size();i++) { //Check all the contours in the contours vector
				float perimeter = arcLength(contours[i], true); //Get the perimeter
				approxPolyDP(contours[i], polyContours[i], epsilon * perimeter, true); //Use the perimeter to give a better approximation of a polygonal shape
			}
		}
	}
	for (int i = 0;i < contours.size();i++) { //Check all the contours in the contours vector
		float perimeter = arcLength(contours[i], true); //Get the perimeter
		approxPolyDP(contours[i], polyContours[i], 0.1 * perimeter, true); //Use the perimeter to give a better approximation of a polygonal shape
	}

	if (polyContours[position].size() > 4) {
		polyContours.clear();
		for (int i = 0;i < contours.size();i++) { //Check all the contours in the contours vector
			float perimeter = arcLength(contours[i], true); //Get the perimeter
			approxPolyDP(contours[i], polyContours[i], 0.1 * perimeter, true); //Use the perimeter to give a better approximation of a polygonal shape
		}

	}

	//Get the 4 corners
	Point2f O(polyContours[position].at(0));
	Point2f P(polyContours[position].at(2));
	Point2f R(polyContours[position].at(1));
	Point2f S(polyContours[position].at(3));

	//Get the intersection Point
	Point2f intersection(get_intersection_Point(O, P, R, S));

	//Get the point with lowest y
	int pos = 0;

	int maxy = mask.rows;
	for (int i = 0;i < 4;i++) {
		if (polyContours[position].at(i).y < maxy) {
			pos = i;
			maxy = polyContours[position].at(i).y;
		}
	}


	//Define the point with the lowest y as the origin point
	Point2f origin(polyContours[position].at(pos));
	Point2f final;

	//Define the final point as the opposite corner of the origin point
	if (pos > 1) {
		int i = pos - 2;
		final=polyContours[position].at(i);
	}
	else {
		int i = pos + 2;
		final=polyContours[position].at(i);
	}

	//Get angle of orientation in the World System of references
	float angle = get_angle(get_Transform2WR(origin, mask.rows), get_Transform2WR(final, mask.rows),mask.rows);

	//Define x and y of the intersection point as float values to send with ROS2
	float x = intersection.x;
	float y = intersection.y;


	//Decide in function of the angle if the cube is in an horitzontal position or in a rotated position
	if ((angle >= 30 && angle <= 60) || (angle <= (-30) && angle >= (-60))) {
		cout << "Horizontal" << endl;
	}
	else {
		cout << "Rotated" << endl;
	}

	imshow("Image", outputImg);
	cout << "Centre: (" << x << ", " << y << ")" << endl;
	cout << "Rotation: " << angle << endl;
	

	rclcpp::shutdown();

	return 0;
}

float get_angle(Point2f O, Point2f P, int rows)
{
	float X = O.x - P.x;
	float Y = O.y - P.y;

	return atan2f(Y,X) * (180 / 3.14159265358979323846);
}

Point2f get_intersection_Point(Point2f O, Point2f P, Point2f R, Point2f S)
{
	float m1, b1, m2, b2;
	float x, y;
	m1 = (P.y - O.y) / (P.x - O.x);
	b1 = O.y - (m1 * O.x);
	m2 = (S.y - R.y) / (S.x - R.x);
	b2 = S.y - (m2 * S.x);

	x = (b2 - b1) / (m1 - m2);
	y = m2 * x + b2;

	return Point2f(int(x), int(y));
}

Point2f get_Transform2WR(Point2f O, int rows)
{
	int x, y;
	x = 1 + int(O.x);
	y = rows - int(O.y);

	return Point2f(x, y);
}