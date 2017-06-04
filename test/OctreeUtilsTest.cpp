/******************************************************************************
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "utils/OctreeUtils.hh"
#include "utils/Logger.hh"

#include <gtest/gtest.h>
#include <cmath>

#include "md_utils/math/transformations.hh"

using namespace octomap_tools;
using namespace Eigen;
using namespace md_utils;

#define EXPECT_MATRIX_EQ(m1, m2)           \
    for (int i = 0; i < m1.rows(); ++i)    \
    for (int j = 0; j < m1.cols(); ++j)    \
    EXPECT_NEAR(m1(i,j), m2(i,j), 1e-6);

TEST(OcTreeUtilsTest, getLeafDepth)
{
  constexpr float res = 0.1;

  OcTree tree1(res);
  tree1.createRoot();
  auto n = tree1.createNodeChild(tree1.getRoot(), 0);
  n = tree1.createNodeChild(n, 0);
  n = tree1.createNodeChild(n, 0);
  n->setValue(0.6);

  EXPECT_EQ(3, getLeafDepth(tree1, *n));
}

TEST(OcTreeUtilsTest, FilterOutPointsNotInRange)
{
  auto cloudInit = createUniformPointCloud(
      Point{-10,-10,-10}, Point{10,10,10}, Point{1,1,1});

  const Point minRange{-5, -5, -5};
  const Point maxRange{5, 5, 5};
  PointCloud cloudFiltered;
  filterOutPointsNotInRange(cloudInit, minRange, maxRange, cloudFiltered);

  EXPECT_GT(cloudFiltered.size(), 0U);

  for (const auto& p : cloudFiltered)
  {
    EXPECT_TRUE(p.x >= minRange.x && p.x <= maxRange.x);
    EXPECT_TRUE(p.y >= minRange.y && p.y <= maxRange.y);
    EXPECT_TRUE(p.z >= minRange.z && p.z <= maxRange.z);
  }
}

