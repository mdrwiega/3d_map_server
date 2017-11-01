/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/
#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/transforms.h>

#include <Eigen/Dense>
#include <octomap/octomap.h>

#include "utils/OctreeUtils.hh"
#include "utils/PointcloudUtils.h"

namespace octomap_tools {

void extractIntersectingAndDownsamplePointClouds(
    const PointCloud& cloud1, const PointCloud& cloud2,
    float voxelSize, const Point& margin,
    PointCloud& cloud1reduced, PointCloud& cloud2reduced);

}
