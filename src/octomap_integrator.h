/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <Eigen/Dense>
#include <octomap/octomap.h>
#include <pcl/point_types.h>

#include "utils/octree_utils.h"

namespace octomap_tools {

using Point = pcl::PointXYZ;

struct OctreeIntegrationConf
{
  unsigned max_iter;          // Max number of iterations (ICP)
  float    max_nn_dist;       // Max correspondence distance (NN)
  float    fitness_eps;       // Euclidean fitness epsilon (ICP)
  Point    intersec_margin;   // Margin in intersecting region extraction (m)
  float    transf_eps;        // Transformation epsilon (ICP)
  float    voxel_size;        // Size of voxel after downsampling
};

/**
 * @brief Integrates two octomaps into one.
 *
 * The output octomap is in the frame of tree2.
 */
OcTreePtr integrateOctomaps(
    const OcTree& tree1, const OcTree& tree2,
    const OctreeIntegrationConf& conf,
    const Eigen::Matrix4f& T_init, Eigen::Matrix4f& T_fin,
    float& error);

OcTreePtr integrateOctomapsPcl(
    const OcTree& tree1, const OcTree& tree2,
    const OctreeIntegrationConf& conf,
    const Eigen::Matrix4f& T_init, Eigen::Matrix4f& T_fin,
    float& error);

Eigen::Matrix4f estimateTransBetweenOctomaps(
    const OcTree& tree1, const OcTree& tree2,
    const OctreeIntegrationConf& conf);

Eigen::Matrix4f estimateTransBetweenOctomapsPcl(
    const OcTree& tree1, const OcTree& tree2,
    const OctreeIntegrationConf& conf);

Eigen::Matrix4f estimateTransBetweenPointclouds(
    const PointCloud& cloud1, const PointCloud& cloud2,
    const OctreeIntegrationConf& params);

}
