/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/
#pragma once

#include <Eigen/Dense>
#include <octomap/octomap.h>

#include <octomap_tools/types.h>

namespace octomap_tools {

/**
 * Transforms octree
 */
OcTreePtr FastOcTreeTransform(const OcTree& tree_in, const Eigen::Matrix4f& transf);

/**
 * More accurate transformation method but also less efficient
 */
OcTreePtr OcTreeTransform(const OcTree& tree_in, const Eigen::Matrix4f& transformation);

/**
 * Crop the octree based on specified min and max vectors
 */
OcTreePtr CropOcTree(const OcTree& tree_in, const Eigen::Vector3f& min, const Eigen::Vector3f& max);

OcTreePtr FastSumOctrees(const OcTree& tree1, const OcTree& tree2);

OcTreePtr SumOctrees(const OcTree& tree1, const OcTree& tree2);

void FilterLeafsNotInRange(const OcTree& tree_in, const Point& min, const Point& max, OcTree& tree_out);

void ExtractIntersectingOctrees(
    const OcTree& tree1, const OcTree& tree2, const Point& margin, OcTree& out_tree1, OcTree& out_tree2);

void ExtractIntersectingPointClouds(
    const PointCloud& cloud1, const PointCloud& cloud2, float voxelSize, const Point& margin,
    PointCloud& cloud1reduced, PointCloud& cloud2reduced);

float calculateNewNodeOccupancy(
    const octomap::point3d& src_point, const octomap::point3d& src_approx_point,
    const OcTree& tree_in, const octomap::OcTreeNode* src_node);

} // namespace octomap_tools
