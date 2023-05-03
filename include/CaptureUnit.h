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

	sl::Camera zed;
	sl::InitParameters initParams;
	sl::BodyTrackingParameters bodyTrackingParams;
	// sl::PositionalTrackingParameters positionalTrackingParams;
	sl::Resolution displayResolution;
	sl::Bodies bodies;
	// sl::Pose camPose;
	sl::BodyTrackingRuntimeParameters bodyTrackingRuntimeParams;
	cv::Mat currentImage;
	cv::Mat prevImage;
	cv::Mat imageDiff;
	sl::Mat image_left;
	sl::float2 img_scale;

	std::mutex processMtx;
	std::once_flag checkGoodFunctionFlag;

	int id;

public:

	CaptureUnit(sl::DeviceProperties deviceProps);	
	//CaptureUnit(const CaptureUnit& captureUnit);

	~CaptureUnit();

	inline sl::InitParameters getInitParameters() { return initParams; }
	inline sl::BodyTrackingParameters getBodyTrackingParameters() { return bodyTrackingParams; }
	// inline sl::PositionalTrackingParameters getpositionalTrackingParameters() { return positionalTrackingParams; }

	sl::ERROR_CODE init();
	void configure();
	//void initProcess();
	void process();
	void parseArgs(int argc, char **argv);
	bool checkGoodFunction();
};

