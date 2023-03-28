#pragma once

#include <sl/Camera.hpp>
#include "GLViewer.hpp"
#include "TrackingViewer.hpp"
#include "Utils.h"

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

	CaptureUnit(sl::DeviceProperties deviceProps)
	{
		initParams.camera_resolution = sl::RESOLUTION::HD720;

		// On Jetson the object detection combined with an heavy depth mode could reduce the frame rate too much
		initParams.depth_mode = isJetson ? sl::DEPTH_MODE::PERFORMANCE : sl::DEPTH_MODE::ULTRA;
		initParams.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
		initParams.sdk_verbose = 1;
		id = deviceProps.id;
		initParams.input.setFromCameraID(deviceProps.id);
		objectDetectionParams.enable_tracking = true; // track people across images flow
		objectDetectionParams.enable_body_fitting = false; // smooth skeletons moves
		objectDetectionParams.body_format = sl::BODY_FORMAT::POSE_18;
		objectDetectionParams.detection_model = isJetson ? sl::DETECTION_MODEL::HUMAN_BODY_FAST : sl::DETECTION_MODEL::HUMAN_BODY_ACCURATE;
	}
	
	CaptureUnit(const CaptureUnit& captureUnit)
	{
		this->id = captureUnit.id;
		this->zed = captureUnit.zed;
		this->initParams = captureUnit.initParams;
		this->objectDetectionParams = captureUnit.objectDetectionParams;
		this->positionalTrackingParams = captureUnit.positionalTrackingParams;
		this->displayResolution = captureUnit.displayResolution;
		this->bodies = captureUnit.bodies;
		this->camPose = captureUnit.camPose;
		this->ObjectDetectionRuntimeParams = captureUnit.ObjectDetectionRuntimeParams;
		this->image_left_ocv = captureUnit.image_left_ocv;
		this->image_left = captureUnit.image_left;
		this->img_scale = img_scale;
	}

	~CaptureUnit()
	{
		image_left.free();
		bodies.object_list.clear();

		// Disable modules
		zed.disableObjectDetection();
		zed.disablePositionalTracking();
		zed.close();
	}

	inline sl::InitParameters getInitParameters() { return initParams; }
	inline sl::ObjectDetectionParameters getObjectDetectionParameters() { return objectDetectionParams; }
	inline sl::PositionalTrackingParameters getpositionalTrackingParameters() { return positionalTrackingParams; }

	sl::ERROR_CODE init();
	void configure();
	void process();
	void parseArgs(int argc, char **argv);
};

