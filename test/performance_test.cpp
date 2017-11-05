/******************************************************************************
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <chrono>
#include <limits>
#include <cmath>

#include "utils/table_printer.h"

#include "test_utils.h"
#include "octomap_integrator.h"
#include "md_utils/math/transformations.h"
#include "utils/octree_utils.h"
#include "octree_transformations.h"
#include <pcl/registration/transforms.h>
#include "utils/pointcloud_utils.h"

using namespace Eigen;
using namespace octomap_tools;
using namespace octomap;
using namespace std::chrono;
using namespace md;

#define SHOW_IMAGES 0
#define RUN_OCTOVIS 0

const std::string ds_path = "datasets/";
const std::string tmp_path = "build/tmp/";

class OctomapIntegratorTest : public ::testing::Test
{
 public:
  using EstimationsConfs = std::list<OctreeIntegrationConf>;

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
    OctreeIntegrationConf parameters;
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
                params.push_back(OctreeIntegrationConf{i, j, k, n, l, m});
    return params;
  }

  void preprocessTestResults(EstimationsResults& results)
  {
    auto& base = results[0];

    for (size_t i = 0; i < results.size(); i++)
    {
      auto& q = results[i].quality;
      q.position = results[i].transform.col(3).block<3,1>(0,0);
      q.rpy = results[i].transform.block<3,3>(0,0).eulerAngles(0,1,2);

      if (i == 0)
      {
        q.positionMSE = std::numeric_limits<float>::min();
        q.orientationMSE = std::numeric_limits<float>::min();
        q.overallMSE = std::numeric_limits<float>::min();
      }
      else
      {
        auto& bq = base.quality;
        q.positionMSE = (bq.position - q.position).norm();
        q.orientationMSE = (bq.rpy - q.rpy).norm();
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
    md::TablePrinter tp(&std::cout);
    const std::string columns[] = {
        "x", "y", "z", "roll", "pitch", "yaw", "time[ms]", "overallMSE",
        "maxIter", "maxCorrDist", "fitnessEps", "margin", "transf_eps", "voxel_size"
    };

    for (const auto& i : columns)
    {
      auto colWidth = std::max(static_cast<int>(i.size()+2), 9);
      tp.addColumn(i, colWidth);
    }

    tp.printTitle("Test results: " + title);
    tp.printHeader();

    for (auto res : results)
    {
      auto p = res.parameters;
      auto t = res.quality.position;
      auto rpy = res.quality.rpy;

      tp << t.x() << t.y() << t.z() << rpy.x() << rpy.y() << rpy.z()
                       << static_cast<float>(res.time.count())
                       << res.quality.overallMSE
                       << p.max_iter << p.max_nn_dist << p.fitness_eps
                       << p.intersec_margin.x << p.transf_eps << p.voxel_size;
    }

    tp.printFooter();
  }
};


TEST_F(OctomapIntegratorTest, GetTransformationBetweenPointclouds_TheSameClouds_OnlyTransformed)
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
  Param<float>    voxelSize   {0.08, 0.08, 4};
  Param<Point>    margin {Point{.5,.5,.5}, Point{.5,.5,.5}, Point{.5,.5,.5}};

  auto params = prepareParamsSet(maxIter, maxCorrDist, fitnessEps, transEps, voxelSize, margin);
  EstimationsResults results {{{}, tfInitial, static_cast<milliseconds>(0)}};

  for (const auto& p : params)
  {
    auto start = high_resolution_clock::now();
    auto tfFinal = estimateTransBetweenPointclouds(sourceCloud, targetCloud, p);
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

TEST_F(OctomapIntegratorTest, GetTransformationBetweenPointclouds_CloudsDiffers)
{
  auto sourceCloud = createCrossShapePointCloud(0.6, 0.2, 0.1, 0.02);

  constexpr float x = 0.1, y = 0.1, z = -0.1;
  constexpr float roll = 5.0 * M_PI / 180, pitch = 5.0 * M_PI / 180, yaw = 10.0 * M_PI / 180;
  auto tfInitial = createTransformationMatrix(x, y, z, roll, pitch, yaw);

  auto sourceCloud2 = sourceCloud + createCrossShapePointCloud(0.6, 0.2, 0.1, 0.02, 0.2, -0.1);

  PointCloud targetCloud;
  pcl::transformPointCloud(sourceCloud2, targetCloud, tfInitial);

  // Parameters ranges (min, max, multiplicator)
  Param<unsigned> maxIter     {1000, 1000, 2};
  Param<float>    maxCorrDist {1.0, 1.0, 2};
  Param<float>    fitnessEps  {1e-5, 1e-5, 10};
  Param<float>    transEps    {1e-12, 1e-12, 1e3};
  Param<float>    voxelSize   {0.02, 0.02, 2};
  Param<Point>    margin {Point{.5,.5,.5}, Point{.5,.5,.5}, Point{.5,.5,.5}};

  auto params = prepareParamsSet(maxIter, maxCorrDist, fitnessEps, transEps, voxelSize, margin);
  EstimationsResults results {{{}, tfInitial, static_cast<milliseconds>(0)}};

  for (const auto& p : params)
  {
    auto start = high_resolution_clock::now();
    auto tfFinal = estimateTransBetweenPointclouds(sourceCloud, targetCloud, p);
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
  printTestResults(results, "Different pointcloud - test2");
}

