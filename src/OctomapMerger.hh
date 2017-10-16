/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <memory>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <octomap/octomap.h>
#include "utils/OctreeUtils.hh"

namespace octomap_tools {

class OctomapMerger
{
 public:
  OctomapMerger() = default;
  ~OctomapMerger() = default;

  struct EstimationParams
  {
    unsigned maxIter;               // Max number of iterations (ICP)
    float    maxCorrespondenceDist; // Max correspondence distance (ICP)
    float    fitnessEps;            // Euclidean fitness epsilon (ICP)
    float    transfEps;             // Transformation epsilon (ICP)
    float    voxelSize;             // Size of voxel after downsampling
    Point    intersecMargin;        // Margin in intersecting region extraction
  };

  void extractIntersectingAndDownsamplePointClouds(
      const PointCloud& cloud1, const PointCloud& cloud2,
      float voxelSize, const Point& margin,
      PointCloud& cloud1reduced, PointCloud& cloud2reduced);

  Eigen::Matrix4f computeTransBetweenPointclouds(
      const PointCloud& cloud1, const PointCloud& cloud2,
      const EstimationParams& params);

  OcTreePtr sumOctrees(OcTree& tree1, OcTree& tree2);

  OcTreePtr transformOctree(const OcTree& tree,
                            const Eigen::Matrix4f& transformation);

  float calculateNewNodeOccupancy(const octomap::point3d& src_point,
                                  const octomap::point3d& src_approx_point,
                                  const OcTree& tree_in,
                                  const octomap::OcTreeNode* src_node);
};

}
