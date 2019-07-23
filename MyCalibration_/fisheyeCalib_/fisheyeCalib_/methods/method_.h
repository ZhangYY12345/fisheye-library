#pragma once
#include "parameterCalib_.h"

//reading xml files
//void* xmlMapReader(xmlMapRead* file);

//fisheye calibration
void fisheyeCalib_(fisheyeCalibInfo infoStereoCalib);

//rectification
void fisheyeUndistort_(std::string filePath_, cv::Size imgSize, cv::Mat mapX, 
	cv::Mat mapY, std::vector<cv::Mat>& imgUndistort);

bool ptsDetect_calib(std::vector<cv::Mat> imgsL, std::vector<cv::Mat> imgsR, 
	douVecPt2f& ptsL, douVecPt2f& ptsR, douVecPt3f& ptsReal, int corRowNum, int corColNum);

void merge4(const cv::Mat& tl, const cv::Mat& tr, const cv::Mat& bl, const cv::Mat& br, cv::Mat& merged);

void rectify_(calibInfo infoStereoCalib);
