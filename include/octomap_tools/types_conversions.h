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

inline octomap::OcTree PointsToOctree(const Eigen::Matrix3Xf& points, double tree_resolution) {
  octomap::OcTree tree(tree_resolution);
  for (auto i = 0; i < points.cols(); ++i)
  {
    auto point = octomap::point3d{points(0,i), points(1,i), points(2,i)};
    tree.setNodeValue(point, 1.0, true);
  }

  return tree;
}

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

inline PointCloud OctreeToPointCloud3(const octomap::OcTree& input_tree) {
  auto start = std::chrono::high_resolution_clock::now();
  octomap::OcTree tree = input_tree;
  tree.expand();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (auto i = tree.begin_leafs(); i != tree.end_leafs(); ++i) {
    if (tree.isNodeOccupied(*i))
      cloud.push_back(pcl::PointXYZ(i.getX(), i.getY(), i.getZ()));
  }
  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
  std::cout << "Octree (" << cloud.size() << " points) converted to pointcloud in: " << diff.count() << " ms." << std::endl;
  return cloud;
}

inline PointCloud OctreeToPointCloud2(const octomap::OcTree& input_tree) {
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
  std::cout << "2 Octree (" << cloud.size() << " points) converted to pointcloud in: " << diff.count() << " ms." << std::endl;
  return cloud;
}

inline PointCloud OctreeToPointCloud(const octomap::OcTree& input_tree) {
  auto start = std::chrono::high_resolution_clock::now();
  octomap::OcTree tree = input_tree;
  const auto max_depth = tree.getTreeDepth();

  // Expand occupied nodes
  std::vector<octomap::OcTreeNode*> collapsed_occ_nodes;
  do {
    collapsed_occ_nodes.clear();
    for (auto it = tree.begin(); it != tree.end(); ++it) {
      if(tree.isNodeOccupied(*it) && it.getDepth() < max_depth) {
        collapsed_occ_nodes.push_back(&(*it));
      }
    }
    for (auto it = collapsed_occ_nodes.begin(); it != collapsed_occ_nodes.end(); ++it)
    {
      tree.expandNode(*it);
    }
  } while(collapsed_occ_nodes.size() > 0);

  //
  PointCloud cloud;
  for (auto it = tree.begin(); it != tree.end(); ++it)
  {
    if(tree.isNodeOccupied(*it))
    {
      auto p = it.getCoordinate();
      cloud.push_back({p.x(), p.y(), p.z()});
    }
  }
  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
  std::cout << "Full octree (" << cloud.size() << " points) converted to pointcloud in: " << diff.count() << " ms." << std::endl;
  return cloud;
}

}
