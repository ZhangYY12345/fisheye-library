//
//  EquisolidAngleProjection.cpp
//  Calibration
//
//  Created by Ryohei Suda on 2014/11/14.
//  Copyright (c) 2014年 RyoheiSuda. All rights reserved.
//

#include "EquisolidAngleProjection.h"
//r = 2 f sin(O-/2)
EquisolidAngleProjection::EquisolidAngleProjection(cv::Point2d p): IncidentVector(p)
{
}

double EquisolidAngleProjection::aoi(double r)
{
    double t;
    t = r / f0;
    
    for(int i = 0; i < a.size(); ++i) {
        t += a[i] * pow(r/f0, 3+2*i);
    }
    t *= f0 / (2 * f);
    t = 2 * asin(t);
    
    return t;
}


cv::Point3d EquisolidAngleProjection::calcDu()
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
        tu *= -(point.x-center.x) * px_size.x * px_size.x / (r * f * cos(theta/2));
        mu += part * tu;
        return mu;
        
    } else {
        return cv::Point3d(0, 0, 0);
    }
}

cv::Point3d EquisolidAngleProjection::calcDv()
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
        tv *= -(point.y-center.y) * px_size.y * px_size.y / (r * f * cos(theta/2));
        mv += part * tv;
        return mv;
        
    } else {
        return cv::Point3d(0, 0, 0);
    }
}

cv::Point3d EquisolidAngleProjection::calcDpx()
{
	if (r != 0) {
		cv::Point3d mpx;
		mpx.x = (point.x - center.x) / r - pow(point.x - center.x, 3) * pow(px_size.x, 2) / pow(r, 3);
		mpx.y = -pow(point.x - center.x, 2) * (point.y - center.y) * px_size.y * px_size.x / pow(r, 3);
		mpx.z = 0;
		mpx *= sin(theta);
		double tpx = 1; // derivative of d(theta)/dv
		for (int i = 0; i < a.size(); ++i) {
			tpx += (2 * i + 3) * a[i] * pow(r / f0, 2 * i + 2);
		}
		tpx *= pow(point.x - center.x, 2) * px_size.x / (r * f * cos(theta / 2));
		mpx += part * tpx;
		return mpx;

	}
	else {
		return cv::Point3d(0, 0, 0);
	}
}

cv::Point3d EquisolidAngleProjection::calcDpy()
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
		tpy *= pow(point.y - center.y, 2) * px_size.y / (r * f * cos(theta / 2));
		mpy += part * tpy;
		return mpy;

	}
	else {
		return cv::Point3d(0, 0, 0);
	}
}

cv::Point3d EquisolidAngleProjection::calcDf()
{
    cv::Point3d mf;
    
    if (r != 0) {
        mf = part * (-2/f * tan(theta/2));
        return mf;
    } else {
        return cv::Point3d(0, 0, 0);
    }
}

std::vector<cv::Point3d> EquisolidAngleProjection::calcDak()
{
    std::vector<cv::Point3d> ms;
    if (r != 0) {
        cv::Point3d m;
        m = part * (f0 / (f * cos(theta/2)));
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
