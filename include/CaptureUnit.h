#pragma once

#include "stdafx.h"
#include "Utils.h"
#include "TrackingViewer.hpp"

#ifdef _SL_JETSON_
const bool isJetson = true;
#else
const bool isJetson = false;
#endif

class CaptureUnit
{
	int id;
	sl::Camera zed;
	sl::InitParameters initParams;
	sl::ObjectDetectionParameters objectDetectionParams;
	sl::PositionalTrackingParameters positionalTrackingParams;
	sl::Resolution displayResolution;
	sl::Objects bodies;
	sl::Pose camPose;
	sl::ObjectDetectionRuntimeParameters ObjectDetectionRuntimeParams;
	cv::Mat image_left_ocv;
	sl::Mat image_left;
	sl::float2 img_scale;

public:

	CaptureUnit(sl::DeviceProperties deviceProps);	
	//CaptureUnit(const CaptureUnit& captureUnit);

	~CaptureUnit();

	inline sl::InitParameters getInitParameters() { return initParams; }
	inline sl::ObjectDetectionParameters getObjectDetectionParameters() { return objectDetectionParams; }
	inline sl::PositionalTrackingParameters getpositionalTrackingParameters() { return positionalTrackingParams; }

	sl::ERROR_CODE init();
	void configure();
	void process();
	void parseArgs(int argc, char **argv);
};

