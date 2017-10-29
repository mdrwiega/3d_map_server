/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/
#pragma once

#include "OctomapMerger.hh"

#include <iomanip>
#include <stdexcept>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/transforms.h>

#include "md_utils/math/cuboid.hh"
#include "md_utils/math/trilinear_interpolation.hh"
#include "md_utils/math/transformations.hh"

#include <Eigen/Dense>
#include <octomap/octomap.h>
#include "utils/OctreeUtils.hh"

namespace octomap_tools {

OcTreePtr transformOctree(const OcTree& tree,
                          const Eigen::Matrix4f& transformation);


float calculateNewNodeOccupancy(const octomap::point3d& src_point,
                                const octomap::point3d& src_approx_point,
                                const OcTree& tree_in,
                                const octomap::OcTreeNode* src_node);

void extractIntersectingOctrees(
    const OcTree& tree1, const OcTree& tree2,
    const Point& margin,
    OcTree& out_tree1, OcTree& out_tree2);

}
