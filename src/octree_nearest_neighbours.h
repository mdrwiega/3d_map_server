#pragma once

#include "utils/OctreeUtils.hh"

namespace octomap_tools {

  static constexpr unsigned tree_max_value = 32768;
  static constexpr unsigned tree_depth_max = 16;

void getNeighborsWithinRadius(const OcTree& tree,
                              const Point& point,
                              const double radiusSquared,
                              const OcTreeNode* node,
                              const OcTreeKey& key,
                              unsigned tree_depth,
                              std::vector<Point>& points,
                              std::vector<float>& distances);

void radiusSearch(const OcTree& tree,
                  const Point &point,
                  const double radius,
                  std::vector<Point> &points,
                  std::vector<float> &distances);

void searchNearestNeighbour(const OcTree& tree,
                            const Point& query,
                            double max_distance,
                            Point& point,
                            float& distance);

void nearestNeighboursKdTree(const Eigen::Matrix3Xf& dst_points,
                             Eigen::Matrix3Xf& src_points,
                             Eigen::Matrix3Xf& nearest_neighbours);

void nearestNeighboursOcTree(const Eigen::Matrix3Xf& dst_points,
                             Eigen::Matrix3Xf& src_points,
                             Eigen::Matrix3Xf& nearest_neighbours);

void nearestNeighboursOnOcTree(const OcTree& tree_dst,
                             Eigen::Matrix3Xf& src_points,
                             Eigen::Matrix3Xf& nearest_neighbours);

}
