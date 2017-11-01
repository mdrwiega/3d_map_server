/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "pointcloud_transformations.h"

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
}

}
