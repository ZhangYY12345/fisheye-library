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
#include "methods/method_.h"
#include <valarray>
//#include <pthread.h>
int main(int argc, const char * argv[])
{
	//single fisheye camera calibration
	fisheyeCalibInfo calibInfoL, calibInfoR;
	calibInfoL.calibPatternFile = "patternsL.xml";
	calibInfoL.calibLineDetected = "linesDetectedL.xml";
	calibInfoL.calibFile = "resCalibL.xml";
	fisheyeCalib_(calibInfoL);

	calibInfoR.calibPatternFile = "patternsR.xml";
	calibInfoR.calibLineDetected = "linesDetectedR.xml";
	calibInfoR.calibFile = "resCalibR.xml";
	fisheyeCalib_(calibInfoR);

	//stereo calibration based on unditort images
	//std::string imgPath = "C:\\Users\\lenovo\\Web\\CaptureFiles\\2019-07-23";
	std::string imgPath = "D:\\studying\\stereo vision\\research code\\data\\2019-07-23";

	//load all the images in the folder
	cv::String filePath = imgPath + "\\*L.jpg";
	std::vector<cv::String> fileNames;
	cv::glob(filePath, fileNames, false);

	cv::Mat distImgL_, distImgR_;
	distImgL_ = cv::imread(fileNames[0]);
	distImgR_ = cv::imread(fileNames[0].substr(0, fileNames[0].length() - 5) + "R.jpg");

	/****************************************
	***reprojection and get the rectified images
	****************************************/

	calibInfo infoCalib;
	infoCalib.calibFileL = "resCalibL.xml";
	infoCalib.calibFileR = "resCalibR.xml";
	infoCalib.calibChessImgPathL = "D:\\studying\\stereo vision\\research code\\data\\2019-07-23\\left";
	infoCalib.calibChessImgPathR = "D:\\studying\\stereo vision\\research code\\data\\2019-07-23\\right";
	infoCalib.chessRowNum = 6;
	infoCalib.chessColNum = 9;
	infoCalib.stereoCalib = "unditortStereoCalib.xml";
	rectify_(infoCalib);

	cv::Mat mapxL, mapyL, mapxR, mapyR;
	cv::Mat lmapx, lmapy, rmapx, rmapy;
	cv::Size imgSize;

	//cv::FileStorage fn(infoCalib.stereoCalib, cv::FileStorage::READ);
	//fn["Fisheye_Undistort_Map_mapxL"] >> mapxL;
	//fn["Fisheye_Undistort_Map_mapyL"] >> mapyL;
	//fn["Fisheye_Undistort_Map_mapxR"] >> mapxR;
	//fn["Fisheye_Undistort_Map_mapyR"] >> mapyR;
	//fn["ImgSize"] >> imgSize;
	//fn["Stereo_Rectify_Map_mapxL"] >> lmapx;
	//fn["Stereo_Rectify_Map_mapyL"] >> lmapy;
	//fn["Stereo_Rectify_Map_mapxR"] >> rmapx;
	//fn["Stereo_Rectify_Map_mapyR"] >> rmapy;
	//fn.release();

	cv::FileStorage fn_1("mapxL.xml", cv::FileStorage::READ);
	fn_1["Fisheye_Undistort_Map_mapxL"] >> mapxL;
	fn_1.release();

	cv::FileStorage fn_2("mapyL.xml", cv::FileStorage::READ);
	fn_2["Fisheye_Undistort_Map_mapyL"] >> mapxL;
	fn_2.release();

	cv::FileStorage fn_3("mapxR.xml", cv::FileStorage::READ);
	fn_3["Fisheye_Undistort_Map_mapxR"] >> mapxL;
	fn_3.release();

	cv::FileStorage fn_4("mapyR.xml", cv::FileStorage::READ);
	fn_4["Fisheye_Undistort_Map_mapyR"] >> mapxL;
	fn_4.release();

	cv::FileStorage fn_5("lmapx.xml", cv::FileStorage::READ);
	fn_5["Stereo_Rectify_Map_mapxL"] >> lmapx;
	fn_5.release();

	cv::FileStorage fn_6("lmapy.xml", cv::FileStorage::READ);
	fn_6["Stereo_Rectify_Map_mapyL"] >> lmapy;
	fn_6.release();

	cv::FileStorage fn_7("rmapx.xml", cv::FileStorage::READ);
	fn_7["Stereo_Rectify_Map_mapxR"] >> rmapx;
	fn_7.release();

	cv::FileStorage fn_8("rmapy.xml", cv::FileStorage::READ);
	fn_8["Stereo_Rectify_Map_mapyR"] >> rmapy;
	fn_8.release();


	//cv::imwrite("Fisheye_Undistort_Map_mapxL.jpg", mapxL);
	//cv::imwrite("Fisheye_Undistort_Map_mapyL.jpg", mapyL);
	//cv::imwrite("Fisheye_Undistort_Map_mapxR.jpg", mapxR);
	//cv::imwrite("Fisheye_Undistort_Map_mapyR.jpg", mapyR);
	//cv::imwrite("Stereo_Rectify_Map_mapxL.jpg", lmapx);
	//cv::imwrite("Stereo_Rectify_Map_mapyL.jpg", lmapy);
	//cv::imwrite("Stereo_Rectify_Map_mapxR.jpg", rmapx);
	//cv::imwrite("Stereo_Rectify_Map_mapyR.jpg", rmapy);


	//mapxL=cv::imread("Fisheye_Undistort_Map_mapxL.jpg");
	//mapyL=cv::imread("Fisheye_Undistort_Map_mapyL.jpg");
	//mapxR=cv::imread("Fisheye_Undistort_Map_mapxR.jpg");
	//mapyR=cv::imread("Fisheye_Undistort_Map_mapyR.jpg");
	//lmapx=cv::imread("Stereo_Rectify_Map_mapxL.jpg");
	//lmapy=cv::imread("Stereo_Rectify_Map_mapyL.jpg");
	//rmapx=cv::imread("Stereo_Rectify_Map_mapxR.jpg");
	//rmapy=cv::imread("Stereo_Rectify_Map_mapyR.jpg");

	//mapxL.convertTo(mapxL, CV_32FC1);
	//mapyL.convertTo(mapyL, CV_32FC1);
	//mapxR.convertTo(mapxR, CV_32FC1);
	//mapyR.convertTo(mapyR, CV_32FC1);
	//lmapx.convertTo(lmapx, CV_32FC1);
	//lmapy.convertTo(lmapy, CV_32FC1);
	//rmapx.convertTo(rmapx, CV_32FC1);
	//rmapy.convertTo(rmapy, CV_32FC1);


	for (int i = 0; i < fileNames.size(); i++)
	{
		cv::Mat distImgL, distImgR;
		distImgL = cv::imread(fileNames[i]);
		distImgR = cv::imread(fileNames[i].substr(0, fileNames[i].length() - 5) + "R.jpg");

		cv::Mat undistImgL, undistImgR;
		if (distImgL.size() != imgSize)
		{
			resize(distImgL, distImgL, imgSize);
		}
		if (distImgR.size() != imgSize)
		{
			resize(distImgR, distImgR, imgSize);
		}
		remap(distImgL, undistImgL, mapxL, mapyL, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
		remap(distImgR, undistImgR, mapxR, mapyR, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

		cv::Mat rectiLeft, rectiRight;
		cv::remap(undistImgL, rectiLeft, lmapx, lmapy, cv::INTER_LINEAR);
		cv::remap(undistImgR, rectiRight, rmapx, rmapy, cv::INTER_LINEAR);

		cv::imwrite(fileNames[i].substr(0, fileNames[i].length() - 5) + "L_rectify.jpg", rectiLeft);
		cv::imwrite(fileNames[i].substr(0, fileNames[i].length() - 5) + "R_rectify.jpg", rectiRight);

		for (int ii = 0; ii < rectiLeft.rows; ii += 100)
		{
			cv::line(undistImgL, cv::Point(0, ii), cv::Point(rectiLeft.cols, ii), cv::Scalar(0, 255, 0));
			cv::line(undistImgR, cv::Point(0, ii), cv::Point(rectiLeft.cols, ii), cv::Scalar(0, 255, 0));

			cv::line(rectiLeft, cv::Point(0, ii), cv::Point(rectiLeft.cols, ii), cv::Scalar(0, 255, 0));
			cv::line(rectiRight, cv::Point(0, ii), cv::Point(rectiLeft.cols, ii), cv::Scalar(0, 255, 0));
		}


		cv::Mat rectification;
		merge4(undistImgL, undistImgR, rectiLeft, rectiRight, rectification);
		cv::imwrite(fileNames[i].substr(0, fileNames[i].length() - 5) + "_rectifyMerge.jpg", rectification);
	}
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