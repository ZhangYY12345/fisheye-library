//
//  StereographicProjection.cpp
//  Calibration
//
//  Created by Ryohei Suda on 2014/11/15.
//  Copyright (c) 2014年 RyoheiSuda. All rights reserved.
//

#include "StereographicProjection.h"
// r = 2 f tan(0-/2)
StereographicProjection::StereographicProjection(cv::Point2d p): IncidentVector(p)
{
}

double StereographicProjection::aoi(double r)
{
    double t;
    t = r / f0;
    
    for(int i = 0; i < a.size(); ++i) {
        t += a[i] * pow(r/f0, 3+2*i);
    }
    t *= f0 / (2 * f);
    t = 2 * atan(t);
    
    return t;
}


cv::Point3d StereographicProjection::calcDu()
{
    if (r != 0) {
        cv::Point3d mu;
        mu.x = (-1/r + pow((point.x - center.x) * px_size.x, 2) / pow(r, 3)) * px_size.x;
        mu.y = (point.x-center.x) * (point.y-center.y) * pow(px_size.x, 2) * px_size.y / pow(r, 3);
        mu.z = 0;
        mu *= sin(theta);
        double tu = 1; // derivative of d(theta)/du
        for (int i = 0; i < a.size(); ++i) {
            tu += (2*i+3) * a[i] * pow(r/f0, 2*i+2);
        }
        tu *= - pow(cos(theta/2), 2) * (point.x-center.x) * px_size.x * px_size.x / (r * f);
        mu += part * tu;
        return mu;
        
    } else {
        return cv::Point3d(0, 0, 0);
    }
}

cv::Point3d StereographicProjection::calcDv()
{
    if (r != 0) {
        cv::Point3d mv;
        mv.x = (point.x-center.x) * (point.y-center.y) * px_size.y * px_size.y * px_size.x / pow(r, 3);
        mv.y = (-1/r + pow((point.y - center.y) * px_size.y, 2) / pow(r, 3)) * px_size.y;
        mv.z = 0;
        mv *= sin(theta);
        double tv = 1; // derivative of d(theta)/dv
        for (int i = 0; i < a.size(); ++i) {
            tv += (2*i+3) * a[i] * pow(r/f0, 2*i+2);
        }
        tv *= - pow(cos(theta/2), 2) * (point.y-center.y) * px_size.y * px_size.y / (r * f);
        mv += part * tv;
        return mv;
        
    } else {
        return cv::Point3d(0, 0, 0);
    }
}

cv::Point3d StereographicProjection::calcDpx()
{
	if (r != 0) {
		cv::Point3d mpx;
		mpx.x = (point.x - center.x) / r - pow(point.x - center.x, 3) * pow(px_size.x, 2) / pow(r, 3);
		mpx.y = - pow(point.x - center.x, 2) * (point.y - center.y) * px_size.y * px_size.x / pow(r, 3);
		mpx.z = 0;
		mpx *= sin(theta);
		double tpx = 1; // derivative of d(theta)/dv
		for (int i = 0; i < a.size(); ++i) {
			tpx += (2 * i + 3) * a[i] * pow(r / f0, 2 * i + 2);
		}
		tpx *= pow(cos(theta / 2), 2) * pow(point.x - center.x, 2) * px_size.x / (r * f);
		mpx += part * tpx;
		return mpx;

	}
	else {
		return cv::Point3d(0, 0, 0);
	}
}

cv::Point3d StereographicProjection::calcDpy()
{
	if (r != 0) {
		cv::Point3d mpy;
		mpy.x = -pow(point.y - center.y, 2) * (point.x - center.x) * px_size.y * px_size.x / pow(r, 3);
		mpy.y = (point.y - center.y) / r - pow(point.y - center.y, 3) * pow(px_size.y, 2) / pow(r, 3);
		mpy.z = 0;
		mpy *= sin(theta);
		double tpy = 1; // derivative of d(theta)/dv
		for (int i = 0; i < a.size(); ++i) {
			tpy += (2 * i + 3) * a[i] * pow(r / f0, 2 * i + 2);
		}
		tpy *= pow(cos(theta / 2), 2) * pow(point.y - center.y, 2) * px_size.y / (r * f);
		mpy += part * tpy;
		return mpy;

	}
	else {
		return cv::Point3d(0, 0, 0);
	}

}

cv::Point3d StereographicProjection::calcDf()
{
    cv::Point3d mf;
    
    if (r != 0) {
        mf = part * (-sin(theta) / f);
        return mf;
    } else {
        return cv::Point3d(0, 0, 0);
    }
}

std::vector<cv::Point3d> StereographicProjection::calcDak()
{
    std::vector<cv::Point3d> ms;
    if (r != 0) {
        cv::Point3d m;
        m = part * (cos(theta/2) * f0 / f);
        for(int i=0; i<a.size(); ++i) {
            ms.push_back(pow(r/f0, 2*i+3) * m);
        }
    } else {
        for (int i = 0; i < a.size(); ++i) {
            ms.push_back(cv::Point3d(0,0,0));
        }
    }
    return ms;
}
