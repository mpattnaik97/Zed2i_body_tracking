#pragma once

#include "stdafx.h"

namespace utils
{
	void print(std::string msg_prefix, sl::ERROR_CODE err_code, std::string msg_suffix);
	std::string time_in_HH_MM_SS_MMM();
};

