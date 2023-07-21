#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/core.hpp>
#include "opencv2/aruco.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include "rclcpp/rclcpp.hpp"

using namespace std;
using namespace cv;

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	string filepath = "/src/frame.jpg";
	string packagepath = ament_index_cpp::get_package_share_directory("irb120pe_detection");
	string path = packagepath + filepath;

 	Mat inputImg = imread(path, -1);

	
	cv::Mat perspTransMatrix, perspectiveImg, perspectivePoseImg;

	if(inputImg.empty()){
		cout<<"Error! Empty image"<<endl;
		return 0;
	}

	//Values of the calibration x and y in mm
	float w = 1050;
	float h = 750;
	
	//Detection of the Aruco Markers in the input image
	std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
	cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
	cv::aruco::detectMarkers(inputImg, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);




	//Parameters to save jpg images
	std::vector<int> params1;
	params1.push_back(IMWRITE_JPEG_QUALITY);
	params1.push_back(95);

	//Visualize detection of the Aruco markers
	Mat outputImage = inputImg.clone();
	aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

	//Calibration process

	Point2f src[4]; //Points of the corners from the inputimage
	Point2f dst[4] = { {0.0f,0.0f},{w,0.0f},{0.0f,h},{w,h} }; //Points calibrated with w and h
	
	int poly_corner[4][3];

	// Get the first corner & ID of the markers
	for (int i = 0; i < 4;i++) {
		poly_corner[i][0] = int(markerIds[i]);
		poly_corner[i][1] = int(markerCorners[i][0].x);
		poly_corner[i][2] = int(markerCorners[i][0].y);
		
	}
	// Arrange by ID 
	for (int i = 0;i < 3;i++) {
		for (int j = i + 1;j < 4;j++) {
			if (poly_corner[i][0] > poly_corner[j][0]) {
				for (int k = 0;k < 3;k++) {
					int aux = poly_corner[i][k];
					poly_corner[i][k] = poly_corner[j][k];
					poly_corner[j][k] = aux;
				}
			}
		}
	}

	// Get the source vector
	for (int i = 0;i < 4;i++) {
		src[i] = Point2f(poly_corner[i][1], poly_corner[i][2]);
	}

	// Get transform matrix
	perspTransMatrix = getPerspectiveTransform(src, dst);

	//Get image of the perspective
	warpPerspective(outputImage, perspectiveImg, perspTransMatrix, Point(w, h));
	
	string path2 = packagepath + "/src/perspective.jpg";

	cout<<"I have read the image"<<endl;

	imwrite(path2, perspectiveImg, params1);

	cout<<"I have saved the image"<<endl;
	imshow("Output Image", perspectiveImg);
	
	waitKey(0);

	rclcpp::shutdown();

	return 0;
}
