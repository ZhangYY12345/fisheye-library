#include "methods/parameterCalib_.h"

//
//  main.cpp
//  Calibration
//
//  Created by ZYY on 2019/07/28.
//  Copyright (c) 2019 ZYY. All rights reserved.
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
	calibInfoL.calibPatternFile = "./20191017-1/patternsL.xml"; //"patternsL.xml";
	calibInfoL.calibLineDetected = "./20191017-1/linesDetectedL.xml";
	calibInfoL.calibFile = "./20191017-1/resCalibL.xml";
	fisheyeCalib_(calibInfoL);

	calibInfoR.calibPatternFile = "./20191017-1/patternsR.xml";
	calibInfoR.calibLineDetected = "./20191017-1/linesDetectedR.xml";
	calibInfoR.calibFile = "./20191017-1/resCalibR.xml";
	fisheyeCalib_(calibInfoR);

	//stereo calibration based on unditort images
	//std::string imgPath = "C:\\Users\\lenovo\\Web\\CaptureFiles\\2019-07-23";
	std::string imgPath = "D:\\studying\\stereo vision\\research code\\data\\20190723-3";

	//load all the images in the folder
	cv::String filePath = imgPath + "\\*L.jpg";
	std::vector<cv::String> fileNames;
	cv::glob(filePath, fileNames, false);

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
	infoCalib.stereoCalib_undistort_mapxL = "undistort_mapxL.xml";
	infoCalib.stereoCalib_undistort_mapyL = "undistort_mapyL.xml";
	infoCalib.stereoCalib_undistort_mapxR = "undistort_mapxR.xml";
	infoCalib.stereoCalib_undistort_mapyR = "undistort_mapyR.xml";

	infoCalib.stereoCalib_rectify_mapxL = "rectify_mapxL.xml";
	infoCalib.stereoCalib_rectify_mapyL = "rectify_mapyL.xml";
	infoCalib.stereoCalib_rectify_mapxR = "rectify_mapxR.xml";
	infoCalib.stereoCalib_rectify_mapyR = "rectify_mapyR.xml";

	rectify_(infoCalib);


	cv::Mat mapxL, mapyL, mapxR, mapyR;
	cv::Mat lmapx, lmapy, rmapx, rmapy;
	cv::Size imgSize;

	cv::Mat K1, K2, D1, D2, matrixR, matrixT;
	cv::FileStorage fn(infoCalib.stereoCalib, cv::FileStorage::READ);
	fn["ImgSize"] >> imgSize;
	fn["StereoCalib_K1"] >> K1;
	fn["StereoCalib_D1"] >> D1;
	fn["StereoCalib_K2"] >> K2;
	fn["StereoCalib_D2"] >> D2;
	fn["StereoCalib_R"] >> matrixR;
	fn["StereoCalib_T"] >> matrixT;
	fn.release();

	cv::FileStorage fn_2(infoCalib.stereoCalib_undistort_mapyL, cv::FileStorage::READ);
	fn_2["Fisheye_Undistort_Map_mapyL"] >> mapyL;
	fn_2.release();

	cv::FileStorage fn_3(infoCalib.stereoCalib_undistort_mapxR, cv::FileStorage::READ);
	fn_3["Fisheye_Undistort_Map_mapxR"] >> mapxR;
	fn_3.release();

	cv::FileStorage fn_4(infoCalib.stereoCalib_undistort_mapyR, cv::FileStorage::READ);
	fn_4["Fisheye_Undistort_Map_mapyR"] >> mapyR;
	fn_4.release();

	cv::FileStorage fn_5(infoCalib.stereoCalib_rectify_mapxL, cv::FileStorage::READ);
	fn_5["Stereo_Rectify_Map_mapxL"] >> lmapx;
	fn_5.release();

	cv::FileStorage fn_6(infoCalib.stereoCalib_rectify_mapyL, cv::FileStorage::READ);
	fn_6["Stereo_Rectify_Map_mapyL"] >> lmapy;
	fn_6.release();

	cv::FileStorage fn_7(infoCalib.stereoCalib_rectify_mapxR, cv::FileStorage::READ);
	fn_7["Stereo_Rectify_Map_mapxR"] >> rmapx;
	fn_7.release();

	cv::FileStorage fn_8(infoCalib.stereoCalib_rectify_mapyR, cv::FileStorage::READ);
	fn_8["Stereo_Rectify_Map_mapyR"] >> rmapy;
	fn_8.release();

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