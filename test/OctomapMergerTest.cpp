/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "OctomapMerger.hh"
#include "utils/OctreeUtils.hh"
#include "utils/Logger.hh"
#include "../table_printer/src/TablePrinter.hh"

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <gtest/gtest.h>
#include <chrono>
#include <limits>
#include <cmath>

#include <pcl/registration/transforms.h>

using namespace octomap_tools;
using namespace std::chrono;

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

    PointCloud createCrossShapePointCloud(float length, float width, float height, float res)
    {
        auto cloud1 = createUniformPointCloud(
                Point{-length, -width, -height},
                Point{length, width, height},
                Point{res, res, res});

        auto cloud2 = createUniformPointCloud(
                Point{-width, -length, -height},
                Point{width, length, height},
                Point{res, res, res});

        return cloud1 + cloud2;

    }

    using EstimationParams = OctomapMerger::EstimationParams;
    using EstimationsConfs = std::list<EstimationParams>;

    struct EstimationQuality
    {
        Eigen::Vector3f position;
        Eigen::Vector3f rpy;
        float positionMSE;
        float orientationMSE;
        float overallMSE;
    };

    struct EstimationResult
    {
        EstimationParams parameters;
        Eigen::Matrix4f transform;
        std::chrono::milliseconds time;
        EstimationQuality quality;
    };

    using EstimationsResults = std::vector<EstimationResult>;
    template <typename T> struct Param { T min, max, multi; };

    EstimationsConfs prepareParamsSet(
            const Param<unsigned>& maxIter, const Param<float>& maxCorrDist,
            const Param<float>& fitnessEps, const Param<float>& transEps,
            const Param<float>& voxelSize, const Param<Point>& margin)
    {
        auto addPoints = [](const Point& i, const Point& j){
            return Point{i.x+j.x, i.y+j.y, i.z+j.z};
        };
        auto comparePoints = [](const Point& i, const Point& j){
            return i.x <= j.x && i.y <= j.y && i.z <= j.z;
        };

        EstimationsConfs params;

        for (auto i = maxIter.min; i <= maxIter.max; i *= maxIter.multi)
            for (auto j = maxCorrDist.min; j <= maxCorrDist.max; j *= maxCorrDist.multi)
                for (auto k = fitnessEps.min; k <= fitnessEps.max; k *= fitnessEps.multi)
                    for (auto l = transEps.min; l <= transEps.max; l *= transEps.multi)
                        for (auto m = voxelSize.min; m <= voxelSize.max; m *= voxelSize.multi)
                            for (auto n = margin.min; comparePoints(n, margin.max); n = addPoints(n, margin.multi))
                                params.push_back(EstimationParams{i, j, k, l, m, n});
        return params;
    }

    void preprocessTestResults(EstimationsResults& results)
    {
        auto& base = results[0];

        for (size_t i = 0; i < results.size(); i++)
        {
            auto& q = results[i].quality;
            auto t = results[i].transform.col(3);
            auto rpy = results[i].transform.block<3,3>(0,0).eulerAngles(0,1,2);

            q.position.x() = t.x();
            q.position.y() = t.y();
            q.position.z() = t.z();
            q.rpy.x() = rpy.x();
            q.rpy.y() = rpy.y();
            q.rpy.z() = rpy.z();

            if (i == 0)
            {
                q.positionMSE = std::numeric_limits<float>::min();
                q.orientationMSE = std::numeric_limits<float>::min();
                q.overallMSE = std::numeric_limits<float>::min();
            }
            else
            {
                auto& bq = base.quality;
                q.positionMSE = sqrt(pow(bq.position.x() - q.position.x(), 2)
                                     + pow(bq.position.y() - q.position.y(), 2)
                                     + pow(bq.position.z() - q.position.z(), 2));

                q.orientationMSE = sqrt(pow(bq.rpy.x() - q.rpy.x(), 2)
                                     + pow(bq.rpy.y() - q.rpy.y(), 2)
                                     + pow(bq.rpy.z() - q.rpy.z(), 2));

                q.overallMSE = q.positionMSE + q.orientationMSE;
            }
        }

        // Sort results by overall MSE
        std::sort(results.begin(), results.end(), [](auto& a, auto& b) {
            return a.quality.overallMSE < b.quality.overallMSE;
        });
    }

    void printTestResults(const EstimationsResults& results, std::string title)
    {
        table_printer::TablePrinter tp(&std::cout);

        const std::string columns[] = {
                "x", "y", "z", "roll", "pitch", "yaw", "time[ms]", "overallMSE",
                "maxIter", "maxCorrDist", "fitnessEps", "transEps", "voxelSize", "margin"
        };

        for (const auto& i : columns)
        {
            auto colWidth = std::max(static_cast<int>(i.size()+2), 9);
            tp.AddColumn(i, colWidth);
        }

        tp.PrintTitle("Test results: " + title);
        tp.PrintHeader();

        for (auto res : results)
        {
            auto p = res.parameters;
            auto t = res.quality.position;
            auto rpy = res.quality.rpy;

            tp << t.x() << t.y() << t.z() << rpy.x() << rpy.y() << rpy.z()
               << static_cast<float>(res.time.count())
               << res.quality.overallMSE
               << p.maxIter << p.maxCorrespondenceDist << p.fitnessEps << p.transfEps
               << p.voxelSize << p.intersecMargin.x;
        }

        tp.PrintFooter();
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

    Point margin {1,1,1};
    merger.extractIntersectingAndDownsamplePointClouds(
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

    Point margin {1,1,1};
    merger.extractIntersectingAndDownsamplePointClouds(
            cloud1, cloud2, 1.0, margin, *cloud1filtered, *cloud2filtered);

    EXPECT_EQ(cloud1filtered->size(), 0U);
    EXPECT_EQ(cloud2filtered->size(), 0U);
}

TEST_F(OctomapMergerTest, GetTransformationBetweenPointclouds_TheSameClouds_OnlyTransformed)
{
    auto sourceCloud = createCrossShapePointCloud(0.6, 0.2, 0.1, 0.02);

    constexpr float x = 0.1, y = 0.1, z = -0.1;
    constexpr float roll = 5.0 * M_PI / 180, pitch = 5.0 * M_PI / 180, yaw = 10.0 * M_PI / 180;
    auto tfInitial = createTransformationMatrix(x, y, z, roll, pitch, yaw);

    PointCloud targetCloud;
    pcl::transformPointCloud(sourceCloud, targetCloud, tfInitial);

    // Parameters ranges (min, max, multiplicator)
    Param<unsigned> maxIter     {1000, 1000, 2};
    Param<float>    maxCorrDist {1.0, 1.0, 2};
    Param<float>    fitnessEps  {1e-5, 1e-5, 10};
    Param<float>    transEps    {1e-12, 1e-12, 1e3};
    Param<float>    voxelSize   {0.02, 0.08, 4};
    Param<Point>    margin {Point{.5,.5,.5}, Point{1.0,1.0,1.0}, Point{.5,.5,.5}};

    auto params = prepareParamsSet(maxIter, maxCorrDist, fitnessEps, transEps, voxelSize, margin);
    EstimationsResults results {{{}, tfInitial, static_cast<milliseconds>(0)}};

    for (const auto& p : params)
    {
        auto start = high_resolution_clock::now();
        auto tfFinal = merger.computeTransBetweenPointclouds(sourceCloud, targetCloud, p);
        auto diff = high_resolution_clock::now() - start;
        results.push_back({p, tfFinal, duration_cast<milliseconds>(diff)});

        auto rpy = tfFinal.block<3,3>(0, 0).eulerAngles(0,1,2);
        constexpr float maxAbsError = 0.08;

        EXPECT_NEAR(x, tfFinal(0,3), maxAbsError);
        EXPECT_NEAR(y, tfFinal(1,3), maxAbsError);
        EXPECT_NEAR(z, tfFinal(2,3), maxAbsError);
        EXPECT_NEAR(roll,  rpy(0), maxAbsError);
        EXPECT_NEAR(pitch, rpy(1), maxAbsError);
        EXPECT_NEAR(yaw,   rpy(2), maxAbsError);
    }

    preprocessTestResults(results);
    printTestResults(results, "Cross pointcloud - test1");
}

TEST_F(OctomapMergerTest, GetTransformationBetweenPointclouds_CloudsDiffers)
{
    auto sourceCloud = createCrossShapePointCloud(0.6, 0.2, 0.1, 0.02);

    constexpr float x = 0.1, y = 0.1, z = -0.1;
    constexpr float roll = 5.0 * M_PI / 180, pitch = 5.0 * M_PI / 180, yaw = 10.0 * M_PI / 180;
    auto tfInitial = createTransformationMatrix(x, y, z, roll, pitch, yaw);

    PointCloud targetCloud;
    pcl::transformPointCloud(sourceCloud, targetCloud, tfInitial);

    // Parameters ranges (min, max, multiplicator)
    Param<unsigned> maxIter     {1000, 1000, 2};
    Param<float>    maxCorrDist {1.0, 1.0, 2};
    Param<float>    fitnessEps  {1e-5, 1e-5, 10};
    Param<float>    transEps    {1e-12, 1e-12, 1e3};
    Param<float>    voxelSize   {0.02, 0.08, 4};
    Param<Point>    margin {Point{.5,.5,.5}, Point{1.0,1.0,1.0}, Point{.5,.5,.5}};

    auto params = prepareParamsSet(maxIter, maxCorrDist, fitnessEps, transEps, voxelSize, margin);
    EstimationsResults results {{{}, tfInitial, static_cast<milliseconds>(0)}};

    for (const auto& p : params)
    {
        auto start = high_resolution_clock::now();
        auto tfFinal = merger.computeTransBetweenPointclouds(sourceCloud, targetCloud, p);
        auto diff = high_resolution_clock::now() - start;
        results.push_back({p, tfFinal, duration_cast<milliseconds>(diff)});

        auto rpy = tfFinal.block<3,3>(0, 0).eulerAngles(0,1,2);
        constexpr float maxAbsError = 0.08;

        EXPECT_NEAR(x, tfFinal(0,3), maxAbsError);
        EXPECT_NEAR(y, tfFinal(1,3), maxAbsError);
        EXPECT_NEAR(z, tfFinal(2,3), maxAbsError);
        EXPECT_NEAR(roll,  rpy(0), maxAbsError);
        EXPECT_NEAR(pitch, rpy(1), maxAbsError);
        EXPECT_NEAR(yaw,   rpy(2), maxAbsError);
    }

    preprocessTestResults(results);
    printTestResults(results, "Cross pointcloud - test1");
}
