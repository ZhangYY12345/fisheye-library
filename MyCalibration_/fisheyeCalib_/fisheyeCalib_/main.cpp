#include "methods/parameterCalib_.h"

//
//  main.cpp
//  Calibration
//
//  Created by Ryohei Suda on 2014/03/30.
//  Copyright (c) 2014 Ryohei Suda. All rights reserved.
//

#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>

int main(int argc, const char * argv[])
{
	/****************************************
	***detect lines in calibration images
	****************************************/
	std::string fname = "D:\\studying\\stereo vision\\research code\\fisheye-stereo-calibrate\\fisheye-library\\MyCalibration_\\fisheyeCalib_\\fisheyeCalib_\\patterns.xml";
	LineDetection ld;
	//    ld.editAllEdges(ld.loadEdgeXML(fname));

	//    std::vector<std::vector<std::vector<cv::Point2i> > > edges = ld.loadEdgeXML(fname);
	//    ld.saveParameters();
	//    ld.editAllEdges(edges);
	ld.loadImageXML(fname);
	ld.saveParameters();

	ld.processAllImages();
	//
	std::string output = "linesDetected.xml";
	ld.writeXML(output);


	/****************************************
	***calibration with pre-detected lines
	****************************************/
	Calibration calib;
	std::string filename = "linesDetected.xml";
	calib.loadData(filename);
	int a_size = 3;
	IncidentVector::initA(a_size);
	//    std::vector<double> a;
	//    a.push_back(5e-3); a.push_back( 6e-4); a.push_back( 7e-5); a.push_back( 8e-6); a.push_back( 9e-7);
	//    a_size = 5;
	//    IncidentVector::setA(a);
	//    int x = atoi(argv[2]), y = atoi(argv[3]), f = atoi(argv[4]);
	//    double x = 953., y = 600., f = 401.;
	//    IncidentVector::setF(f);
	//    IncidentVector::setCenter(cv::Point2d(793,606));
	//    IncidentVector::setF0((int)f);

	std::cout << "Projection Model:\t" << IncidentVector::getProjectionName() << std::endl;
	std::cout << "Center:\t" << IncidentVector::getCenter() << std::endl;
	std::cout << "     f:\t" << IncidentVector::getF() << std::endl;
	for (int i = 0; i < IncidentVector::nparam - 3; ++i) {
		std::cout << "    a" << i << ":\t" << IncidentVector::getA().at(i) << std::endl;
	}

	std::cout << "Orthogonal pairs: " << calib.edges.size() << std::endl;
	long lines = 0;
	long points = 0;
	for (auto &pair : calib.edges) {
		lines += pair.edge[0].size() + pair.edge[1].size();
		for (auto &line : pair.edge[0]) {
			points += line.size();
		}
		for (auto &line : pair.edge[1]) {
			points += line.size();
		}
	}
	std::cout << "Lines: " << lines << std::endl;
	std::cout << "Points: " << points << std::endl;


	// Show an image of all edges
//    cv::Mat img = cv::Mat::zeros(IncidentVector::getImgSize().height, IncidentVector::getImgSize().width, CV_8UC3);
//    cv::Vec3b color[30] = {cv::Vec3b(255,255,255), cv::Vec3b(255,0,0), cv::Vec3b(255,255,0), cv::Vec3b(0,255,0), cv::Vec3b(0,0,255),
//        cv::Vec3b(255,0,255), cv::Vec3b(204,51,51), cv::Vec3b(204,204,51), cv::Vec3b(51,204,51), cv::Vec3b(51,204,204),
//        cv::Vec3b(51,51,204), cv::Vec3b(204,51,204), cv::Vec3b(204,204,204), cv::Vec3b(153,102,102), cv::Vec3b(153,153,102),
//        cv::Vec3b(102,153,102), cv::Vec3b(102,153,153), cv::Vec3b(102,102,153), cv::Vec3b(153,102,153), cv::Vec3b(153,153,153),
//        cv::Vec3b(51,51,204), cv::Vec3b(204,51,204), cv::Vec3b(204,204,204), cv::Vec3b(153,102,102), cv::Vec3b(153,153,102),
//        cv::Vec3b(102,153,102), cv::Vec3b(102,153,153), cv::Vec3b(102,102,153), cv::Vec3b(153,102,153), cv::Vec3b(153,153,153),
//    };
//    cv::namedWindow("lines", CV_WINDOW_NORMAL);
//    int j = 0;
//    for (auto &pair : calib.edges) {
//        for (int i = 0; i < 2; ++i) {
//            for (auto &line : pair.edge[i]) {
//                for (auto &point : line) {
//                    img.at<cv::Vec3b>(int(point->point.y), int(point->point.x)) = color[j%30];
//                }
//            }
//        }
//        cv::imshow("lines", img);
//        cv::waitKey();
//        ++j;
//    }
//        cv::imshow("edges", img);
//        cv::waitKey();
//        img = cv::Mat::zeros(IncidentVector::getImgSize().height, IncidentVector::getImgSize().width, CV_8UC1);
//    cv::imwrite("lines.png", img);

//    if (std::string(argv[7]) == std::string("divide")) {
//        calib.calibrate(true);
//    } else {
//        calib.calibrate(false);
//    }

//    IncidentVector::initA(0);
//    calib.calibrate(false);
//    IncidentVector::initA(1);
//    calib.calibrate(false);
	IncidentVector::initA(a_size);
	calib.calibrate(false);
	//    calib.calibrate(true);
	//    calib.calibrate2();

	std::string outname = "resCalib.xml";
	calib.save(outname);

	//    calib.calibrate(true);
	//    calib.save(std::string("d_")+outname);

	std::cout << "END" << std::endl;


	/****************************************
	***reprojection and get the rectified images
	****************************************/
	Reprojection reproj;
	double f_;

	std::string param = "resCalib.xml";
	//std::cout << "Type parameter file name > ";
	//std::cin >> param;
	reproj.loadPrameters(param);

	// Print parameters
	std::cout << "f: " << IncidentVector::getF() << "\nf0: " << IncidentVector::getF0() << std::endl;
	std::cout << "center: " << IncidentVector::getCenter() << std::endl;
	std::cout << "image size: " << IncidentVector::getImgSize() << std::endl;
	std::cout << "ai: ";
	std::vector<double> a_s = IncidentVector::getA();
	for (std::vector<double>::iterator it = a_s.begin(); it != a_s.end(); it++) {
		std::cout << *it << '\t';
	}
	std::cout << std::endl;

	reproj.theta2radius();
	//    reproj.saveRadius2Theta("Stereographic.dat");

	std::string srcname = "9_pattern2.jpg";
	//std::cout << "Type source image file name > ";
	//std::cin >> srcname;
	cv::Mat src = cv::imread(srcname);
	cv::Mat mapx;
	cv::Mat mapy;

	f_ = IncidentVector::getF();
	reproj.calcMaps(f_, mapx, mapy);

	cv::Mat dst;
	cv::remap(src, dst, mapx, mapy, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT); // Rectify
	cv::namedWindow("src", cv::WINDOW_GUI_NORMAL);
	cv::imshow("src", src);
	cv::moveWindow("src", 0, 0);
	cv::namedWindow("dst", cv::WINDOW_GUI_NORMAL);
	cv::imshow("dst", dst);
	cv::moveWindow("dst", 0, 0);
	//cv::waitKey();
	cv::imwrite("out.png", dst);

	return 0;
}

////capture images for single camera calibration
//int main()
//{
//	cv::VideoCapture cap(1);
//	int count = 0;
//	bool flag0 = false, flag1 = false, flag2 = false, flag3 = false;
//
//	if (cap.isOpened())
//	{
//		while (count < 12)
//		{
//			cv::Mat frame;
//			cap.read(frame);
//			cv::imshow("cap", frame);
//
//			char key = cv::waitKey(100);
//			switch(key)
//			{
//			case 'a':	//水平直线1
//				cv::imwrite(std::to_string(count) + "_pattern0.jpg", frame);
//				std::cout << "pattern0.jpg"<< std::endl;
//				flag0 = true;
//				break;
//			case 'd':	//水平直线2
//				cv::imwrite(std::to_string(count) + "_pattern1.jpg", frame);
//				std::cout << "pattern1.jpg" << std::endl;
//				flag1 = true;
//				break;
//			case 'w':	//垂直直线1
//				cv::imwrite(std::to_string(count) + "_pattern2.jpg", frame);
//				std::cout << "pattern2.jpg" << std::endl;
//				flag2 = true;
//				break;
//			case 's':	//垂直直线2
//				cv::imwrite(std::to_string(count) + "_pattern3.jpg", frame);
//				std::cout << "pattern3.jpg" << std::endl;
//				flag3 = true;
//				break;
//			}
//
//			if(flag0 && flag1 && flag2 && flag3)
//			{
//				count++;
//				flag0 = flag1 = flag2 = flag3 = false;
//			}
//		}
//	}
//}