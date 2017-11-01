/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "utils/OctreeUtils.hh"

#include <gtest/gtest.h>

#include "pointcloud_transformations.h"

using namespace octomap_tools;
using namespace octomap;

TEST(PointcloudTransformationsTest, ExtractIntersectingPointClouds_CommonPartExist)
{
  // Cloud with ranges from -10 to 10
  auto cloud1 = createUniformPointCloud(
      Point{-10,-10,-10}, Point{10,10,10}, Point{1,1,1});

  // Cloud with ranges from 0 to 20
  auto cloud2 = createUniformPointCloud(
      Point{0,0,0}, Point{20,20,20}, Point{1,1,1});

  PointCloud::Ptr cloud1filtered(new PointCloud);
  PointCloud::Ptr cloud2filtered(new PointCloud);

  Point margin {1,1,1};
  extractIntersectingAndDownsamplePointClouds(
      cloud1, cloud2, 1.0, margin, *cloud1filtered, *cloud2filtered);

  // Common part is from 0 to 10 and that points should be in each pointcloud
  const Point minRange{0, 0, 0};
  const Point maxRange{10, 10, 10};

  EXPECT_GT(cloud1filtered->size(), 0U);
  EXPECT_GT(cloud2filtered->size(), 0U);

  for (const auto& p : *cloud1filtered)
  {
    EXPECT_TRUE(p.x >= minRange.x && p.x <= maxRange.x);
    EXPECT_TRUE(p.y >= minRange.y && p.y <= maxRange.y);
    EXPECT_TRUE(p.z >= minRange.z && p.z <= maxRange.z);
  }
  for (const auto& p : *cloud2filtered)
  {
    EXPECT_TRUE(p.x >= minRange.x && p.x <= maxRange.x);
    EXPECT_TRUE(p.y >= minRange.y && p.y <= maxRange.y);
    EXPECT_TRUE(p.z >= minRange.z && p.z <= maxRange.z);
  }
}

TEST(PointcloudTransformationsTest, ExtractIntersectingPointClouds_CommonPartNotExist)
{
  // Cloud with ranges from -10 to 10
  auto cloud1 = createUniformPointCloud(
      Point{-10,-10,-10}, Point{10,10,10}, Point{1,1,1});

  // Cloud with ranges from 20 to 30
  auto cloud2 = createUniformPointCloud(
      Point{20,20,20}, Point{30,30,30}, Point{1,1,1});

  PointCloud::Ptr cloud1filtered(new PointCloud);
  PointCloud::Ptr cloud2filtered(new PointCloud);

  Point margin {1,1,1};
  extractIntersectingAndDownsamplePointClouds(
      cloud1, cloud2, 1.0, margin, *cloud1filtered, *cloud2filtered);

  EXPECT_EQ(cloud1filtered->size(), 0U);
  EXPECT_EQ(cloud2filtered->size(), 0U);
}

