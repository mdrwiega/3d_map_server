/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <iostream>

#define LOG_INFO_STREAM(msg) std::cout << msg;
#define LOG_ERROR_STREAM(msg) std::cerr << msg;
