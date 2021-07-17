#pragma once

#include <chrono>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <octomap/octomap.h>
#include <Eigen/Dense>

#include <common/utils.h>

namespace octomap_tools {

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

template <typename T>
constexpr T ToDeg(T angle) {
  return angle * 180.0 / M_PI;
}

template <typename T> constexpr T ToRad(T angle) {
  return angle * M_PI / 180.0;
}

inline Eigen::Matrix3Xf OctreeToPoints(const octomap::OcTree& input_tree) {
  octomap::OcTree tree = input_tree;
  ExpandOccupiedNodesRecursive(tree, tree.getRoot(), 0);

  Eigen::Matrix3Xf m(3, tree.getNumLeafNodes());
  // Traverse all leafs in the tree
  int k = 0;
  for (auto i = tree.begin_leafs(); i != tree.end_leafs(); ++i) {
    m.col(k++) = Eigen::Vector3f(i.getX(), i.getY(), i.getZ());
  }
  return m;
}

inline PointCloudPtr OcTreeToPointCloud(const octomap::OcTree& input_tree) {
  auto start = std::chrono::high_resolution_clock::now();
  std::cout << "Size of octree : " << input_tree.size() << "\n";
  octomap::OcTree tree = input_tree;
  ExpandOccupiedNodesRecursive(tree, tree.getRoot(), 0);
  PointCloudPtr cloud(new PointCloud);
  for (auto i = tree.begin_leafs(); i != tree.end_leafs(); ++i) {
    if (tree.isNodeOccupied(*i))
      cloud->push_back(pcl::PointXYZ(i.getX(), i.getY(), i.getZ()));
  }

  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
  std::cout << "OcTree (" << cloud->size() << " nodes) converted to PointCloud in "
            << diff.count() << " ms." << std::endl;
  return cloud;
}

[[deprecated("Conversion from Pointcloud to octree is lossy.")]]
inline octomap::OcTree PointCloudToOctree(const PointCloud& cloud, double tree_resolution) {
  auto start = std::chrono::high_resolution_clock::now();
  octomap::OcTree tree(tree_resolution);
  for (const auto& i : cloud) {
    auto point = octomap::point3d{i.x, i.y, i.z};
    tree.updateNode(point, true, true);
  }
  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
  std::cout << "Pointcloud (" << cloud.size() << " points) converted to octree in: "
            << diff.count() << " ms." << std::endl;
  return tree;
}

} // namespace octomap_tools
