#pragma once

#include "CaptureUnit.h"
#include "TrackingViewer.hpp"
//#include <pthread.h>

int main(int argc, char **argv) {

	std::vector<sl::DeviceProperties> deviceList = sl::Camera::getDeviceList();
	int nDevices = deviceList.size();
    
	if (nDevices <= 0)
	{
		std::cout << "No Zed camera detected";
		return EXIT_FAILURE;
	}

	// Create ZED objects
	std::vector<std::shared_ptr<CaptureUnit>> captureUnits;
	captureUnits.reserve(nDevices);

	std::vector<std::thread> threads(nDevices);

	for (int z = 0; z < nDevices; z++) {
		std::cout << "ID : " << deviceList[z].id << " ,model : " << deviceList[z].camera_model << " , S/N : " 
			<< deviceList[z].serial_number << " , state : " << deviceList[z].camera_state << std::endl;
		captureUnits.push_back(std::make_shared<CaptureUnit>(deviceList[z]));
	}

	// Open the camera
	int i = 0;
	for (auto& captureUnit : captureUnits)
	{
		auto returned_state = captureUnit->init();
		if (returned_state != sl::ERROR_CODE::SUCCESS)
			return EXIT_FAILURE;

		captureUnit->parseArgs(argc, argv);
		captureUnit->configure();
		threads[i++] = std::thread(&CaptureUnit::process, captureUnit);
	}

	for (auto& th : threads)
	{
		th.join();
	}


    return EXIT_SUCCESS;
}

