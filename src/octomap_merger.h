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
#include "utils/OctreeUtils.hh"

namespace octomap_tools {

OcTreePtr sumOctrees(OcTree& tree1, OcTree& tree2);

}
