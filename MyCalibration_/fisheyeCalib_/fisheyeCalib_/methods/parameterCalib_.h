#pragma once

#include "../methods_libs/IncidentVector.h"
#include "../methods_libs/EquidistanceProjection.h"
#include "../methods_libs/EquisolidAngleProjection.h"
#include "../methods_libs/OrthographicProjection.h"
#include "../methods_libs/StereographicProjection.h"
#include "../methods_libs/tinyxml2.h"

struct calib_data
{
	std::vector<std::vector<cv::Point2i> > detectedLines;
};