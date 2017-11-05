#pragma once

#include "octree_nearest_neighbours.h"
#include "pointcloud_transformations.h"

#include <Eigen/Dense>
#include "utils/octree_utils.h"

namespace octomap_tools {

using NearestNeighboursFcn = std::function<void(
    const Eigen::Matrix3Xf&, Eigen::Matrix3Xf&, Eigen::Matrix3Xf&)>;

/**
 * @brief Calculates transformation with ICP method between two input point sets
 *
 * @param new_points - set of new input points (source point clouds)
   @param ref_points - set of reference input points.
 * @param R - rotation matrix, 3x3
 * @param T - translation vector 3x1
 *
 * @return err - cumulative sum of distance points
 */
float icp(const Eigen::Matrix3Xf& new_points,
          const Eigen::Matrix3Xf& ref_points,
          Eigen::Matrix3f& R,
          Eigen::Vector3f& T,
          unsigned max_iter = 50,
          float tolerance = 0.001,
          NearestNeighboursFcn nnCalculator = nearestNeighboursKdTree);

float icp(const OcTree& src_tree, const OcTree& dst_points,
          Eigen::Matrix3f& R, Eigen::Vector3f& T,
          unsigned max_iter, float tolerance);

Eigen::Matrix4f icp(const OcTree& src_tree, const OcTree& dst_points,
                    unsigned max_iter, float tolerance);

}

