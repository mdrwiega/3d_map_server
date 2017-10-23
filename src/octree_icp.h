#include "utils/OctreeUtils.hh"

#include <Eigen/Dense>

namespace octomap_tools {

using NearestNeighboursFcn = std::function<void(
    const Eigen::Matrix3Xf&, Eigen::Matrix3Xf&, Eigen::Matrix3Xf&)>;

void nearestNeighboursKdTree(const Eigen::Matrix3Xf& dst_points,
                             Eigen::Matrix3Xf& src_points,
                             Eigen::Matrix3Xf& nearest_neighbours);

void nearestNeighboursOcTree(const Eigen::Matrix3Xf& dst_points,
                             Eigen::Matrix3Xf& src_points,
                             Eigen::Matrix3Xf& nearest_neighbours);

void nearestNeighboursOnOcTree(const OcTree& tree_dst,
                             Eigen::Matrix3Xf& src_points,
                             Eigen::Matrix3Xf& nearest_neighbours);
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
}

