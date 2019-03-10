/******************************************************************************
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "utils/octree_utils.h"

#include <gtest/gtest.h>

#include "md_utils/math/transformations.h"

#include "test_utils.h"

using namespace octomap_tools;
using namespace Eigen;
using namespace md;


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

