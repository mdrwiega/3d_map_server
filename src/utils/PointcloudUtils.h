/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <iostream>
#include <iomanip>
#include <limits>
#include <stdexcept>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

#include "utils/Logger.hh"

#include <Eigen/Dense>

namespace octomap_tools {

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;
using PointCloudPtr = PointCloud::Ptr;

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
    LOG_ERR() << "\nCouldn't read file " << fileName;
    return nullptr;
  }
  LOG_INF() << "Loaded " << cloud->width * cloud->height
      << " data points from " << fileName << std::endl;
  return cloud;
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

/**
 * Checks if 3D points in matrix (each point in row) are collinear
 */
inline bool pointsAreCollinear(const Eigen::Matrix3f& points)
{
  Eigen::Vector3f ab = points.row(1) - points.row(0);
  Eigen::Vector3f ac = points.row(2) - points.row(0);
  return ab.cross(ac) == Eigen::Vector3f{0,0,0};
}

/**
 * Calculates plane from three points
 *
 * @param[in] A - matrix with points coordinates in rows
 */
inline Eigen::Vector4f calculatePlaneFromThreePoints(const Eigen::Matrix3f& A)
{
  if (pointsAreCollinear(A))
  {
    LOG_ERR() << "\nPoints are collinear. Please choose other points.\n";
    throw std::runtime_error("Points are collinear");
  }

  Eigen::Vector3f x = A.colPivHouseholderQr().solve(Eigen::Vector3f{-1,-1,-1});
  LOG_DBG() << "\nThe plane:\n"
            << x[0] << " x + " << x[1] << " y + " << x[2] << " z + 1 = 0\n";

  return {x[0], x[1], x[2], 1};
}

inline void printPointcloudInfo(const PointCloud& cloud, const std::string cloudName)
{
  Point min, max;
  pcl::getMinMax3D(cloud, min, max);

  LOG_INF() << "\nPointcloud: " << cloudName
      << "\n---------------------------------------"
      << "\nSize: " << cloud.width * cloud.height
      << std::setprecision(3)
      << "\nLimits x: (" << min.x << ", " << max.x << ")"
      << "\nLimits y: (" << min.y << ", " << max.y << ")"
      << "\nLimits z: (" << min.z << ", " << max.z << ")"
      << "\n";
}

inline void visualizePointCloud(const PointCloud::Ptr cloud)
{
  pcl::visualization::CloudViewer viewer ("Cloud Viewer");
  viewer.showCloud (cloud);
  while (!viewer.wasStopped ()) { }
}

}
