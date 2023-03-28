///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2022, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/*****************************************************************************************
 ** This sample demonstrates how to detect human bodies and retrieves their 3D position **
 **         with the ZED SDK and display the result in an OpenGL window.                **
 *****************************************************************************************/

// ZED includes
#include <sl/Camera.hpp>
#include "CaptureUnit.h"
#include "Utils.h"

// Sample includes
#include "GLViewer.hpp"
#include "TrackingViewer.hpp"

int main(int argc, char **argv) {

	std::vector<sl::DeviceProperties> deviceList = sl::Camera::getDeviceList();
	int nDevices = deviceList.size();
    
	// Create ZED objects
	std::vector<CaptureUnit> captureUnits;
	captureUnits.reserve(nDevices);

	std::vector<std::thread> threads(nDevices);

	for (int z = 0; z < nDevices; z++) {
		std::cout << "ID : " << deviceList[z].id << " ,model : " << deviceList[z].camera_model << " , S/N : " 
			<< deviceList[z].serial_number << " , state : " << deviceList[z].camera_state << std::endl;
		captureUnits.emplace_back(deviceList[z]);
	}

	// Open the camera
	int i = 0;
	for (auto& captureUnit : captureUnits)
	{
		auto returned_state = captureUnit.init();
		if (returned_state != sl::ERROR_CODE::SUCCESS)
			return EXIT_FAILURE;

		captureUnit.parseArgs(argc, argv);
		captureUnit.configure();
		threads[i++] = std::thread(&CaptureUnit::process, &captureUnit);
	}
	for (auto& th : threads)
	{
		th.join();
	}
    return EXIT_SUCCESS;
}

