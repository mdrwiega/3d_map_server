/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <octomap/octomap.h>
#include <Eigen/Dense>
//#include <opencv/cv.hpp>

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


//inline cv::Point ToCv(const Eigen::Vector3f& p)
//{
//  return cv::Point(p(0), p(1));
//}

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

inline octomap::OcTree PointsToOctree(
    const Eigen::Matrix3Xf& points, double tree_resolution)
{
  octomap::OcTree tree(tree_resolution);
  for (auto i = 0; i < points.cols(); ++i)
  {
    auto point = octomap::point3d{points(0,i), points(1,i), points(2,i)};
    tree.setNodeValue(point, 1.0, true);
  }

  return tree;
}

inline octomap::OcTree PointCloudToOctree(
    const PointCloud& cloud, double tree_resolution)
{
  octomap::OcTree tree(tree_resolution);
  for (auto i : cloud)
  {
    auto point = octomap::point3d{i.x, i.y, i.z};
    tree.setNodeValue(point, 1.0, true);
  }
  return tree;
}

inline pcl::PointCloud<pcl::PointXYZ> octreeToPointCloud(const octomap::OcTree& tree) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (auto i = tree.begin_leafs(); i != tree.end_leafs(); ++i) {
    if (tree.isNodeOccupied(*i))
      cloud.push_back(pcl::PointXYZ(i.getX(), i.getY(), i.getZ()));
  }
  return cloud;
}

inline PointCloud convertOctreeToPointcloud(octomap::OcTree& tree)
{
  auto maxDepth = tree.getTreeDepth();
  //  LOG_INF() << "tree depth is " << maxDepth << std::endl;

  // Expansion of occupied nodes
  std::vector<octomap::OcTreeNode*> collapsedOccNodes;
  do {
    collapsedOccNodes.clear();
    for (auto it = tree.begin(); it != tree.end(); ++it)
    {
      if(tree.isNodeOccupied(*it) && it.getDepth() < maxDepth)
      {
        collapsedOccNodes.push_back(&(*it));
      }
    }
    for (auto it = collapsedOccNodes.begin(); it != collapsedOccNodes.end(); ++it)
    {
      tree.expandNode(*it);
    }
    //    LOG_INF() << "expanded " << collapsedOccNodes.size() << " nodes" << std::endl;
  } while(collapsedOccNodes.size() > 0);

  PointCloud cloud;
  for (auto it = tree.begin(); it != tree.end(); ++it)
  {
    if(tree.isNodeOccupied(*it))
    {
      auto p = it.getCoordinate();
      cloud.push_back({p.x(), p.y(), p.z()});
    }
  }
  return cloud;
}

}
