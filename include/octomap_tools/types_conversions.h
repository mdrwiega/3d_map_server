/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <chrono>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <octomap/octomap.h>
#include <Eigen/Dense>

#include <octomap_tools/utils.h>

namespace octomap_tools {

constexpr double kPi  = 3.14159265358979323846;


using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

inline Point ToPcl(const Eigen::Vector3f& v) {
  return Point(v(0), v(1), v(2));
}

inline Point ToPcl(const octomap::point3d& p) {
  return Point(p.x(), p.y(), p.z());
}

inline Eigen::Vector3f ToEigen(const Point& p) {
  return { p.x, p.y, p.z };
}

inline Eigen::Vector3f ToEigen(const octomap::point3d& p) {
  return { p.x(), p.y(), p.z() };
}

inline octomap::point3d ToOctomap(const Eigen::Vector3f& p) {
  return octomap::point3d(p[0], p[1], p[2]);
}

inline float ToRadians(float deg) {
  return deg * kPi / 180.0;
}

template <typename T> T ToRad(T deg) {
  return deg * kPi / 180.0;
}

inline Eigen::Matrix3Xf OctreeToPoints(const octomap::OcTree& tree)
{
  Eigen::Matrix3Xf m(3, tree.getNumLeafNodes());
  int k = 0;
  // Traverse all leafs in the tree
  for (auto i = tree.begin_leafs(); i != tree.end_leafs(); ++i)
  {
    m.col(k++) = Eigen::Vector3f(i.getX(), i.getY(), i.getZ());
  }
  return m;
}

[[deprecated("Conversion from Pointcloud to octree is lossy.")]]
inline octomap::OcTree PointCloudToOctree(const PointCloud& cloud, double tree_resolution) {
  auto start = std::chrono::high_resolution_clock::now();
  octomap::OcTree tree(tree_resolution);
  for (const auto& i : cloud) {
    auto point = octomap::point3d{i.x, i.y, i.z};
    tree.setNodeValue(point, 1.0, true);
  }
  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
  std::cout << "Pointcloud (" << cloud.size() << " points) converted to octree in: " << diff.count() << " ms." << std::endl;
  return tree;
}

inline PointCloud OcTreeToPointCloud(const octomap::OcTree& input_tree) {
  auto start = std::chrono::high_resolution_clock::now();

  octomap::OcTree tree = input_tree;
  expandOccupiedRecursive(tree, tree.getRoot(), 0);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (auto i = tree.begin_leafs(); i != tree.end_leafs(); ++i) {
    if (tree.isNodeOccupied(*i))
      cloud.push_back(pcl::PointXYZ(i.getX(), i.getY(), i.getZ()));
  }

  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
  std::cout << "Octree (" << cloud.size() << " occ nodes) converted to pointcloud in: " << diff.count() << " ms." << std::endl;
  return cloud;
}

}
