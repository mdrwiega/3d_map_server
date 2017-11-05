/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/
#pragma once

#include <iomanip>
#include <stdexcept>
#include <algorithm>

#include "md_utils/math/cuboid.h"
#include "md_utils/math/trilinear_interpolation.h"
#include "md_utils/math/transformations.h"

#include <Eigen/Dense>
#include <octomap/octomap.h>

#include "utils/octree_utils.h"

namespace octomap_tools {

OcTreePtr transformOctree(const OcTree& tree,
                          const Eigen::Matrix4f& transformation);

float calculateNewNodeOccupancy(const octomap::point3d& src_point,
                                const octomap::point3d& src_approx_point,
                                const OcTree& tree_in,
                                const octomap::OcTreeNode* src_node);

void getMinMaxOctree(const OcTree& tree, Point& min, Point& max);

[[deprecated]]
void filterOutLeafsNotInRange(
    const OcTree& tree_in, const Point& min, const Point& max, OcTree& tree_out);

OcTree cutOctree(const OcTree& tree_in,
                 const Eigen::Vector3f& min,
                 const Eigen::Vector3f& max);

void extractIntersectingOctrees(
    const OcTree& tree1, const OcTree& tree2,
    const Point& margin,
    OcTree& out_tree1, OcTree& out_tree2);

}
