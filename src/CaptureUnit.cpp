#include "CaptureUnit.h"
#include <tchar.h>

bool is_playback = false;

CaptureUnit::CaptureUnit(sl::DeviceProperties deviceProps)
{
	initParams.camera_resolution = sl::RESOLUTION::HD720;

	// On Jetson the object detection combined with an heavy depth mode could reduce the frame rate too much
	initParams.depth_mode = isJetson ? sl::DEPTH_MODE::PERFORMANCE : sl::DEPTH_MODE::ULTRA;
	initParams.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
	initParams.sdk_verbose = 1;
	id = deviceProps.id;
	//initParams.sdk_gpu_id = id % 2;
	initParams.input.setFromCameraID(deviceProps.id);
	objectDetectionParams.enable_tracking = false; // track people across images flow
	objectDetectionParams.enable_body_fitting = false; // smooth skeletons moves
	objectDetectionParams.body_format = sl::BODY_FORMAT::POSE_18;
	objectDetectionParams.detection_model = isJetson ? sl::DETECTION_MODEL::HUMAN_BODY_FAST : sl::DETECTION_MODEL::HUMAN_BODY_ACCURATE;
}

//CaptureUnit::CaptureUnit(const CaptureUnit& captureUnit)
//{
//	this->id = captureUnit.id;
//	this->zed = captureUnit.zed;
//	this->initParams = captureUnit.initParams;
//	this->objectDetectionParams = captureUnit.objectDetectionParams;
//	this->positionalTrackingParams = captureUnit.positionalTrackingParams;
//	this->displayResolution = captureUnit.displayResolution;
//	this->bodies = captureUnit.bodies;
//	this->camPose = captureUnit.camPose;
//	this->ObjectDetectionRuntimeParams = captureUnit.ObjectDetectionRuntimeParams;
//	this->image_left_ocv = captureUnit.image_left_ocv;
//	this->image_left = captureUnit.image_left;
//	this->img_scale = img_scale;
//}

CaptureUnit::~CaptureUnit()
{
	image_left.free();
	bodies.object_list.clear();

	// Disable modules
	zed.disableObjectDetection();
	zed.disablePositionalTracking();
	zed.close();
}

void CaptureUnit::parseArgs(int argc, char ** argv)
{
	if (argc > 1 && std::string(argv[1]).find(".svo") != std::string::npos) {
		// SVO input mode
		initParams.input.setFromSVOFile(argv[1]);
		is_playback = true;
		std::cout << "[Sample] Using SVO File input: " << argv[1] << std::endl;
	}
	else if (argc > 1 && std::string(argv[1]).find(".svo") == std::string::npos) {
		std::string arg = std::string(argv[1]);
		unsigned int a, b, c, d, port;
		if (sscanf(arg.c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5) {
			// Stream input mode - IP + port
			std::string ip_adress = std::to_string(a) + "." + std::to_string(b) + "." + std::to_string(c) + "." + std::to_string(d);
			initParams.input.setFromStream(sl::String(ip_adress.c_str()), port);
			std::cout << "[Sample] Using Stream input, IP : " << ip_adress << ", port : " << port << std::endl;
		}
		else if (sscanf(arg.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
			// Stream input mode - IP only
			initParams.input.setFromStream(sl::String(argv[1]));
			std::cout << "[Sample] Using Stream input, IP : " << argv[1] << std::endl;
		}
		else if (arg.find("HD2K") != std::string::npos) {
			initParams.camera_resolution = sl::RESOLUTION::HD2K;
			std::cout << "[Sample] Using Camera in resolution HD2K" << std::endl;
		}
		else if (arg.find("HD1080") != std::string::npos) {
			initParams.camera_resolution = sl::RESOLUTION::HD1080;
			std::cout << "[Sample] Using Camera in resolution HD1080" << std::endl;
		}
		else if (arg.find("HD720") != std::string::npos) {
			initParams.camera_resolution = sl::RESOLUTION::HD720;
			std::cout << "[Sample] Using Camera in resolution HD720" << std::endl;
		}
		else if (arg.find("VGA") != std::string::npos) {
			initParams.camera_resolution = sl::RESOLUTION::VGA;
			std::cout << "[Sample] Using Camera in resolution VGA" << std::endl;
		}
	}
}

sl::ERROR_CODE CaptureUnit::init()
{
	std::cout << "Opening camera: " << id <<std::endl;
	auto returned_state = zed.open(initParams);
	if (returned_state != sl::ERROR_CODE::SUCCESS) {
		utils::print("Open Camera", returned_state, "\nExit program.");
		zed.close();
		return returned_state;
	}
	std::cout << "Opening camera: " << id << " successfull" << std::endl;

	returned_state = zed.enablePositionalTracking(positionalTrackingParams);
	if (returned_state != sl::ERROR_CODE::SUCCESS) {
		utils::print("enable Positional Tracking", returned_state, "\nExit program.");
		zed.close();
		return returned_state;
	}

	returned_state = zed.enableObjectDetection(objectDetectionParams);
	if (returned_state != sl::ERROR_CODE::SUCCESS) {
		utils::print("enable Object Detection", returned_state, "\nExit program.");
		zed.close();
		return returned_state;
	}

	return sl::ERROR_CODE::SUCCESS;
}

void CaptureUnit::configure()
{
	sl::CameraConfiguration config = zed.getCameraInformation().camera_configuration;
	displayResolution = sl::Resolution(std::min((int)config.resolution.width, 1280), std::min((int)config.resolution.height, 720));
	
	image_left_ocv = cv::Mat(displayResolution.height, displayResolution.width, CV_8UC4, 1);
	image_left = sl::Mat(displayResolution, sl::MAT_TYPE::U8_C4, image_left_ocv.data, image_left_ocv.step);
	img_scale = sl::float2(displayResolution.width / (float)config.resolution.width, displayResolution.height / (float)config.resolution.height);
	
	camPose.pose_data.setIdentity();
	ObjectDetectionRuntimeParams.detection_confidence_threshold = 40;
}

void CaptureUnit::initProcess()
{
	auto handle = GetCurrentThread();
	
	if (!SetThreadPriority(handle, THREAD_PRIORITY_HIGHEST))
	{
		_tprintf(TEXT("Failed to end background mode (%d)\n"), GetLastError());
	}
	
	_tprintf(TEXT("Current thread (id: %d) priority is 0x%x\n"), GetThreadId(handle), GetThreadPriority(handle));
	
	process();
}

void CaptureUnit::process()
{
	char key = ' ';
	std::cout << " Initiating process while loop for camera: " << id << std::endl;

	while (key != 'q') {
		
		std::lock_guard<std::mutex> guard(processMtx);

		// Grab images
		if (zed.grab() == sl::ERROR_CODE::SUCCESS) {

			// Retrieve Detected Human Bodies
			zed.retrieveObjects(bodies, ObjectDetectionRuntimeParams);

			//OCV View
			zed.retrieveImage(image_left, sl::VIEW::LEFT, sl::MEM::CPU, displayResolution);

			zed.getPosition(camPose, sl::REFERENCE_FRAME::WORLD);

			std::string window_name = "ZED | 2D View " + std::to_string(id);

			int fps = static_cast<int>(zed.getCurrentFPS());
			auto timestamp = utils::time_in_HH_MM_SS_MMM();
			render_2D(image_left_ocv, img_scale, bodies.object_list, objectDetectionParams.enable_tracking, objectDetectionParams.body_format);
			cv::putText(image_left_ocv, "FPS: " + std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(0, 0, 0));
			cv::putText(image_left_ocv, "Timestamp: " + timestamp, cv::Point(10, 60), cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(0, 0, 0));
			cv::imshow(window_name, image_left_ocv);
			key = cv::waitKey(10);
		}
	}
}
