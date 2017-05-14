/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "OctomapMerger.hh"

#include <iostream>
#include <stdexcept>
#include <gtest/gtest.h>

using namespace octomap_tools;

class OctomapMergerTest : public ::testing::Test
{
public:
    OctomapMergerTest() = default;
    ~OctomapMergerTest() = default;

    PointCloud createUniformPointCloud(Point min, Point max, Point step)
    {
        if (min.x > max.x || min.y > max.y || min.z > max.z)
            throw std::runtime_error("Incorrect ranges");

        PointCloud cloud;

        for (float i = min.x; i < max.x; i += step.x)
            for (float j = min.y; j < max.y; j += step.y)
                for (float k = min.z; k < max.z; k += step.z)
                    cloud.push_back(Point{(float)i, (float)j, (float)k});

        return cloud;
    }

    OctomapMerger merger;
};

TEST_F(OctomapMergerTest, FilterOutPointsNotInRange)
{
    auto cloudInit = createUniformPointCloud(
            Point{-10,-10,-10}, Point{10,10,10}, Point{1,1,1});

    const Point minRange{-5, -5, -5};
    const Point maxRange{5, 5, 5};
    PointCloud cloudFiltered;
    merger.filterOutPointsNotInRange(cloudInit, minRange, maxRange, cloudFiltered);

    EXPECT_GT(cloudFiltered.size(), 0U);

    for (const auto& p : cloudFiltered)
    {
        EXPECT_TRUE(p.x >= minRange.x && p.x <= maxRange.x);
        EXPECT_TRUE(p.y >= minRange.y && p.y <= maxRange.y);
        EXPECT_TRUE(p.z >= minRange.z && p.z <= maxRange.z);
    }
}

TEST_F(OctomapMergerTest, ExtractIntersectingPointClouds_CommonPartExist)
{
    // Cloud with ranges from -10 to 10
    auto cloud1 = createUniformPointCloud(
            Point{-10,-10,-10}, Point{10,10,10}, Point{1,1,1});

    // Cloud with ranges from 0 to 20
    auto cloud2 = createUniformPointCloud(
            Point{0,0,0}, Point{20,20,20}, Point{1,1,1});

    PointCloud::Ptr cloud1filtered(new PointCloud);
    PointCloud::Ptr cloud2filtered(new PointCloud);

    merger.extractIntersectingPointClouds(cloud1, cloud2, 1.0, *cloud1filtered, *cloud2filtered);

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

TEST_F(OctomapMergerTest, ExtractIntersectingPointClouds_CommonPartNotExist)
{
    // Cloud with ranges from -10 to 10
    auto cloud1 = createUniformPointCloud(
            Point{-10,-10,-10}, Point{10,10,10}, Point{1,1,1});

    // Cloud with ranges from 20 to 30
    auto cloud2 = createUniformPointCloud(
            Point{20,20,20}, Point{30,30,30}, Point{1,1,1});

    PointCloud::Ptr cloud1filtered(new PointCloud);
    PointCloud::Ptr cloud2filtered(new PointCloud);

    merger.extractIntersectingPointClouds(cloud1, cloud2, 1.0, *cloud1filtered, *cloud2filtered);

    EXPECT_EQ(cloud1filtered->size(), 0U);
    EXPECT_EQ(cloud2filtered->size(), 0U);
}


