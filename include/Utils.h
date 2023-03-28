#pragma once

#include <sl/Camera.hpp>
#include "GLViewer.hpp"
#include "TrackingViewer.hpp"

namespace utils
{
	void print(std::string msg_prefix, sl::ERROR_CODE err_code, std::string msg_suffix);
	std::string time_in_HH_MM_SS_MMM();
};

