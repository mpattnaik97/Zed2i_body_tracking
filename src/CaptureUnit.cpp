#include "CaptureUnit.h"

bool is_playback = false;

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
	auto returned_state = zed.open(initParams);
	if (returned_state != sl::ERROR_CODE::SUCCESS) {
		utils::print("Open Camera", returned_state, "\nExit program.");
		zed.close();
		return returned_state;
	}

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

void CaptureUnit::process()
{
	char key = ' ';
	while (key != 'q') {

		// Grab images
		if (zed.grab() == sl::ERROR_CODE::SUCCESS) {

			// Retrieve Detected Human Bodies
			zed.retrieveObjects(bodies, ObjectDetectionRuntimeParams);

			//OCV View
			zed.retrieveImage(image_left, sl::VIEW::LEFT, sl::MEM::CPU, displayResolution);

			zed.getPosition(camPose, sl::REFERENCE_FRAME::WORLD);

			std::string window_name = "ZED | 2D View " + std::to_string(id);

			int fps = zed.getCurrentFPS();
			auto timestamp = utils::time_in_HH_MM_SS_MMM();
			render_2D(image_left_ocv, img_scale, bodies.object_list, objectDetectionParams.enable_tracking, objectDetectionParams.body_format);
			cv::putText(image_left_ocv, "FPS: " + std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(0, 0, 0));
			cv::putText(image_left_ocv, "Timestamp: " + timestamp, cv::Point(10, 60), cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(0, 0, 0));
			cv::imshow(window_name, image_left_ocv);
			key = cv::waitKey(10);
		}
	}
}