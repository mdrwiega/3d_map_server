/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <memory>

#include <Eigen/Dense>
#include <octomap/octomap.h>

#include "utils/octree_utils.h"

namespace octomap_tools {

OcTreePtr sumOctrees(const OcTree& tree1, const OcTree& tree2);

}
