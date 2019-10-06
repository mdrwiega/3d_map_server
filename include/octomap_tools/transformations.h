/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/
#pragma once

#include <Eigen/Dense>
#include <octomap/octomap.h>

#include <octomap_tools/utils.h>

namespace octomap_tools {

using Vector3f = Eigen::Vector3f;

/**
 * Transforms octree based on passed transformation
 */
OcTreePtr transformOctree(const OcTree& tree,
                          const Eigen::Matrix4f& transformation);

// Get min and max of tree in O(log n) where n is number of tree nodes
void getMinMaxOctree(const OcTree& tree, Vector3f& min, Vector3f& max);

/**
 * Crop the octree based on specified min and max vectors
 */
OcTree cutOctree(const OcTree& tree_in, const Vector3f& min, const Vector3f& max);

OcTreePtr CropOcTree(const OcTree& tree_in, const Vector3f& min, const Vector3f& max);

OcTreePtr sumOctrees(const OcTree& tree1, const OcTree& tree2);

void extractIntersectingOctrees(const OcTree& tree1, const OcTree& tree2,
                                const Point& margin,
                                OcTree& out_tree1, OcTree& out_tree2);

void filterOutLeafsNotInRange(
    const OcTree& tree_in, const Point& min, const Point& max, OcTree& tree_out);

void extractIntersectingAndDownsamplePointClouds(
    const PointCloud& cloud1, const PointCloud& cloud2,
    float voxelSize, const Point& margin,
    PointCloud& cloud1reduced, PointCloud& cloud2reduced);

}
