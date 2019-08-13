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

void writeOcTreeToFile(const OcTree& tree, const std::string& fileName);


void writePointCloudAsOctreeToFile(PointCloud::Ptr& cloud,
                                   const std::string& fileName, float resolution = 0.05);

PointCloud createUniformPointCloud(Point min, Point max, Point step);

std::unique_ptr<OcTree> readOctreeFromFile(const std::string fileName);

int getNodeDepth(const OcTree& tree, const octomap::point3d& point,
                 const OcTreeNode& node);

int getLeafDepth(const OcTree& tree, const OcTreeNode& node);

void printOcTreeInfo(const OcTree& tree, std::string name);


inline void printOcTreeInfo(const OcTree& tree, std::string name) {
  double xMin, xMax, yMin, yMax, zMin, zMax;
  tree.getMetricMin(xMin, yMin, zMin);
  tree.getMetricMax(xMax, yMax, zMax);
  std::cout << "OcTree: " << name << "\n"
            << "Size: " << tree.size() << "  Leafs: " << tree.getNumLeafNodes() << "  Resolution: " << tree.getResolution() << "\n"
            << "Limits: x(" << xMin << ", " << xMax << ")  "
            << "y(" << yMin << ", " << yMax << ")  z(" << zMin << ", " << zMax << ")\n";
}

inline void printOcTree(const OcTree& tree, std::string name) {
  printOcTreeInfo(tree, name);
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

inline PointCloudPtr readPointCloudFromFile(const std::string fileName)
{
  PointCloudPtr cloud (new PointCloud);

  if (pcl::io::loadPCDFile<Point> (fileName, *cloud) == -1)
  {
//    LOG_ERR() << "\nCouldn't read file " << fileName;
    return nullptr;
  }
//  LOG_INF() << "Loaded " << cloud->width * cloud->height
//      << " data points from " << fileName << std::endl;
  return cloud;
}

inline void savePointCloudToFile(const std::string fileName, const PointCloud& cloudIn, bool binary)
{
  if (pcl::io::savePCDFile<Point> (fileName, cloudIn, binary) == -1)
  {
//    LOG_ERR() << "\nCouldn't save to file " << fileName;
    return;
  }
//  LOG_INF() << "Saved " << cloudIn.width * cloudIn.height
//      << " data points to " << fileName << std::endl;
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

inline std::string pointcloudInfoToString(const PointCloud& cloud, const std::string cloudName)
{
  std::stringstream ss;
  Point min, max;
  pcl::getMinMax3D(cloud, min, max);

   ss << "\nPointcloud: " << cloudName
      << "\nSize: " << cloud.width * cloud.height
      << std::setprecision(3)
      << "\nLimits: x(" << min.x << ", " << max.x << ")  "
      << "y(" << min.y << ", " << max.y << ")  "
      << "z(" << min.z << ", " << max.z << ")\n";
  return ss.str();
}

inline void printPointcloudInfo(const PointCloud& cloud, const std::string cloudName)
{
//  LOG_INF() << pointcloudInfoToString(cloud, cloudName);
}

inline void visualizePointCloud(const PointCloud::Ptr cloud)
{
//  pcl::visualization::CloudViewer viewer ("Cloud Viewer");
//  viewer.showCloud (cloud);
//  while (!viewer.wasStopped ()) { }
}

inline PointCloud createCrossShapePointCloud(
    float length, float width, float height, float res,
    float offsetX = 0, float offsetY = 0, float offsetZ = 0)
{
  auto cloud1 = createUniformPointCloud(
      Point{-length + offsetX, -width + offsetY, -height + offsetZ},
      Point{length + offsetX, width + offsetY, height + offsetZ},
      Point{res, res, res});

  auto cloud2 = createUniformPointCloud(
      Point{-width + offsetX, -length + offsetY, -height + offsetZ},
      Point{width + offsetX, length + offsetY, height + offsetZ},
      Point{res, res, res});

  return cloud1 + cloud2;

}

}
