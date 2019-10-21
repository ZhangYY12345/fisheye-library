//
//  OrthographicProjection.h
//  Calibration
//
//  Created by Ryohei Suda on 2015/01/25.
//  Copyright (c) 2015年 RyoheiSuda. All rights reserved.
//

#ifndef __Calibration__OrthographicProjection__
#define __Calibration__OrthographicProjection__

#include "IncidentVector.h"

class OrthographicProjection : public IncidentVector
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
    OrthographicProjection(cv::Point2d p);
    double aoi(double r); // Calculate theta
};
#endif /* defined(__Calibration__OrthographicProjection__) */
