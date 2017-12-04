/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "pointcloud_transformations.h"

#include "utils/logger.h"

namespace octomap_tools {

void extractIntersectingAndDownsamplePointClouds(
    const PointCloud& cloud1, const PointCloud& cloud2,
    float voxelSize, const Point& margin,
    PointCloud& cloud1reduced, PointCloud& cloud2reduced)
{
  PointCloud::Ptr cloud1filtered(new PointCloud);
  PointCloud::Ptr cloud2filtered(new PointCloud);

  Point minCloud1, maxCloud1;
  pcl::getMinMax3D(cloud1, minCloud1, maxCloud1);
  filterOutPointsNotInRange(cloud2, minCloud1, maxCloud1, *cloud2filtered);

  Point minCloud2filtered, maxCloud2filtered;
  pcl::getMinMax3D(*cloud2filtered, minCloud2filtered, maxCloud2filtered);

  auto addPoints = [](const Point& i, const Point& j){
    return Point{i.x+j.x, i.y+j.y, i.z+j.z}; };
  auto subPoints = [](const Point& i, const Point& j){
    return Point{i.x-j.x, i.y-j.y, i.z-j.z}; };

  minCloud2filtered = subPoints(minCloud2filtered, margin);
  maxCloud2filtered = addPoints(maxCloud2filtered, margin);

  filterOutPointsNotInRange(cloud1, minCloud2filtered, maxCloud2filtered, *cloud1filtered);

  downsamplePointCloud(cloud1filtered, cloud1reduced, voxelSize);
  downsamplePointCloud(cloud2filtered, cloud2reduced, voxelSize);

  Point minCloud2, maxCloud2;
  pcl::getMinMax3D(cloud2, minCloud2, maxCloud2);
  Point min_red1, max_red1;
  pcl::getMinMax3D(cloud1reduced, min_red1, max_red1);
  Point min_red2, max_red2;
  pcl::getMinMax3D(cloud1reduced, min_red2, max_red2);

  LOG_DBG() << "Extract intersecting\n"
      << "Input cloud 1 size: " << cloud1.size()
      << " min: " << minCloud1 << " max: " << maxCloud1 << std::endl
      << "Input cloud 2 size: " << cloud2.size()
      << " min: " << minCloud2 << " max: " << maxCloud2 << std::endl
      << "Filtered cloud 1 size: " << cloud1filtered->size() << std::endl
      << "Filtered cloud 2 size: " << cloud2filtered->size() << std::endl
      << "Downsampled cloud 1 size: " << cloud1reduced.size()
      << " min: " << min_red1 << " max: " << max_red1 << std::endl
      << "Downsampled cloud 2 size: " << cloud2reduced.size()
      << " min: " << min_red2 << " max: " << max_red2 << std::endl;
}

}
