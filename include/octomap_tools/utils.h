/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <iomanip>
#include <stdexcept>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include <octomap/octomap.h>
#include <Eigen/Dense>

namespace octomap_tools {

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;
using PointCloudPtr = PointCloud::Ptr;

using OcTree = octomap::OcTree;
using OcTreePtr = std::unique_ptr<octomap::OcTree>;
using OcTreeNode = octomap::OcTreeNode;
using OcTreeKey = octomap::OcTreeKey;

PointCloud createUniformPointCloud(Point min, Point max, Point step);

int getNodeDepth(const OcTree& tree, const octomap::point3d& point,
                 const OcTreeNode& node);

int getLeafDepth(const OcTree& tree, const OcTreeNode& node);

inline void PrintOcTreeInfo(const OcTree& tree, std::string name) {
  double xMin, xMax, yMin, yMax, zMin, zMax;
  tree.getMetricMin(xMin, yMin, zMin);
  tree.getMetricMax(xMax, yMax, zMax);
  std::cout << "OcTree: " << name << "\n"
            << "Size: " << tree.size() << "  Leafs: " << tree.getNumLeafNodes() << "  Resolution: " << tree.getResolution() << "\n"
            << "Limits: x(" << xMin << ", " << xMax << ")  "
            << "y(" << yMin << ", " << yMax << ")  z(" << zMin << ", " << zMax << ")\n";
}

inline void printOcTree(const OcTree& tree, std::string name) {
  PrintOcTreeInfo(tree, name);
  std::cout << "Leafs:\n";
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    std::cout << it.getCoordinate() << "  LogOdds: " << it->getLogOdds()
                                    << "  Depth: " << getLeafDepth(tree, *it) << "\n";
  }
  std::cout << std::endl;
}

void filterOutPointsNotInRange(const PointCloud& cloudIn,
                               const Point& min, const Point& max,
                               PointCloud& cloudOut);

void expandNodeOnlyEmptyChilds(OcTreeNode* node, OcTree& tree);

void printPointsAndDistances(std::string title,
                             std::vector<Point>& points,
                             std::vector<float>& distances);

int getKeyDepth(const OcTree& tree, const octomap::point3d& point,
                const octomap::OcTreeKey& key);

bool contains(OcTree& tree, const Point& query, float sqRadius,
              const octomap::OcTreeKey& o);

int getClosestChild(const Point& q, const Point& p);

float squaredNorm(const Point& p);

float squaredDistance(const Point& p, const Point& q);

double getVoxelSquaredSideLen(const OcTree& tree,
                              unsigned tree_depth_arg);

double getVoxelSquaredDiameter(const OcTree& tree,
                               unsigned tree_depth_arg);

inline void downsamplePointCloud(const PointCloud::ConstPtr& cloudIn,
                          PointCloud& cloudOut, double voxelSize)
{
  pcl::VoxelGrid<Point> grid;
  grid.setLeafSize(voxelSize, voxelSize, voxelSize);
  grid.setInputCloud(cloudIn);
  grid.filter(cloudOut);
}

/**
 * Splits pointcloud into two parts
 *
 * @param[in] plane - plane equation in form: a*x + b*y + c*y + d = 0
 *                    where elements of vector are factors: a,b,c,d
 * @param[in] in - input pointcloud
 * @param[out] out1 - output pointcloud which contains points bigger than plane
 * @param[out] out2 - output pointcloud which contains points smaller than plane
 */
inline void splitPointcloud(const Eigen::Vector4f& plane,
                     const PointCloud& in, PointCloud& out1, PointCloud& out2)
{
  out1.clear();
  out2.clear();

  for (const auto& p : in.points)
  {
    if ((plane[0] * p.x + plane[1] * p.y + plane[2] * p.z + plane[3]) > 0)
      out1.push_back(p);
    else
      out2.push_back(p);
  }
}

inline std::string PointCloudInfoToString(const PointCloud& cloud, const std::string cloudName)
{
  std::stringstream ss;
  Point min, max;
  pcl::getMinMax3D(cloud, min, max);

   ss << "\nPointcloud: " << cloudName
      << "\nSize: " << cloud.size()
      << std::setprecision(3)
      << "\nLimits: x(" << min.x << ", " << max.x << ")  "
      << "y(" << min.y << ", " << max.y << ")  "
      << "z(" << min.z << ", " << max.z << ")\n";
  return ss.str();
}

inline void ExpandOccupiedNodesRecursive(octomap::OcTree& tree,
                                         octomap::OcTreeNode* node,
                                         unsigned depth) {
  if (depth >= tree.getTreeDepth()) {
    return;
  }
  // Do not expand if node is not occupied
  if (!tree.isNodeOccupied(node)) {
    return;
  }
  // Expand node if it doesn't have children
  if (!tree.nodeHasChildren(node)) {
    tree.expandNode(node);
  }

  // Recursively expand children
  for (unsigned int i=0; i<8; i++) {
    if (tree.nodeChildExists(node, i)) {
      ExpandOccupiedNodesRecursive(tree, tree.getNodeChild(node, i), depth+1);
    }
  }
}

}
