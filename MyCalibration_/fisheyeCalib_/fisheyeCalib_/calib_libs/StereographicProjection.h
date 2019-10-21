//
//  StereographicProjection.h
//  Calibration
//
//  Created by Ryohei Suda on 2014/11/15.
//  Copyright (c) 2014年 RyoheiSuda. All rights reserved.
//

#ifndef __Calibration__StereographicProjection__
#define __Calibration__StereographicProjection__

#include "IncidentVector.h"
#include <opencv2/opencv.hpp>

class StereographicProjection : public IncidentVector
{
private:
    cv::Point3d calcDu();
    cv::Point3d calcDv();
	cv::Point3d calcDpx();
	cv::Point3d calcDpy();
    cv::Point3d calcDf();
    std::vector<cv::Point3d> calcDak();

	void calcDu(cv::Point3d& res);	
	void calcDv(cv::Point3d& res);
	void calcDpx(cv::Point3d& res);
	void calcDpy(cv::Point3d& res);
	void calcDf(cv::Point3d& res);
	void calcDak(std::vector<cv::Point3d>& res);

public:
    StereographicProjection(cv::Point2d p);
    double aoi(double r); // Calculate theta
};

#endif /* defined(__Calibration__StereographicProjection__) */
