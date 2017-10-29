/******************************************************************************
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "utils/OctreeUtils.hh"

#include <gtest/gtest.h>

#include <pcl/registration/transforms.h>
#include "md_utils/math/transformations.hh"
#include "octree_transformations.h"

using namespace octomap_tools;
using namespace octomap;
using namespace md_utils;

#define EXPECT_POINT3D_EQ(n1, n2) \
    EXPECT_NEAR(n1.x(), n2.x(), 1e-5); \
    EXPECT_NEAR(n1.y(), n2.y(), 1e-5); \
    EXPECT_NEAR(n1.z(), n2.z(), 1e-5);

TEST(OctreeTransformationsTest, transformOcTree_TranslationOnly)
{
  constexpr float res = 0.1;
  OcTree tree(res);
  tree.setNodeValue(0.05, 0.05, 0.05, 0.6);
  tree.setNodeValue(0.25, 0.05, 0.05, 0.5);
  tree.setNodeValue(0.25, 0.05, 0.45, 0.4);

  float dx = 0.2, dy = 0.0, dz = -0.0;
  float roll = 0 * M_PI / 180, pitch = 0 * M_PI / 180, yaw = 0 * M_PI / 180;
  auto transform = createTransformationMatrix(dx, dy, dz, roll, pitch, yaw);

  auto transformedTree = transformOctree(tree, transform);

  printOcTree(tree, "Tree");
  printOcTree(*transformedTree, "Transformed tree");

  auto node = transformedTree->begin();
  ASSERT_NE(node, transformedTree->end());
  EXPECT_POINT3D_EQ(node.getCoordinate(), point3d(.05 + dx, .05 + dy, .05 + dz));
  EXPECT_NEAR(node->getValue(), 0.6, 1e-5);
  ++node;
  ASSERT_NE(node, transformedTree->end());
  EXPECT_POINT3D_EQ(node.getCoordinate(), point3d(.25 + dx, .05 + dy, .05 + dz));
  EXPECT_NEAR(node->getValue(), 0.5, 1e-5);
  ++node;
  ASSERT_NE(node, transformedTree->end());
  EXPECT_POINT3D_EQ(node.getCoordinate(), point3d(.25 + dx, .05 + dy, .45 + dz));
  EXPECT_NEAR(node->getValue(), 0.4, 1e-5);
}

TEST(OctreeTransformationsTest, transformOcTree_TranslationAndRotation)
{
  constexpr float res = 0.1;
  OcTree tree(res);
  tree.setNodeValue(0.05, 0.05, 0.05, 0.6);
  tree.setNodeValue(0.25, 0.05, 0.05, 0.5);
  tree.setNodeValue(0.25, 0.05, 0.45, 0.4);

  float dx = 0.1, dy = 0.0, dz = 0.0;
  float roll = 0 * M_PI / 180, pitch = 0 * M_PI / 180, yaw = 180 * M_PI / 180;
  auto transform = createTransformationMatrix(dx, dy, dz, roll, pitch, yaw);

  auto transformedTree = transformOctree(tree, transform);

  printOcTree(tree, "Tree");
  printOcTree(*transformedTree, "Transformed tree");

  auto node = transformedTree->begin();
  ASSERT_NE(node, transformedTree->end());
  EXPECT_POINT3D_EQ(node.getCoordinate(), point3d(-.25 + dx, -.05 + dy, .05 + dz));
  EXPECT_NEAR(node->getValue(), 0.5, 1e-5);
  ++node;
  ASSERT_NE(node, transformedTree->end());
  EXPECT_POINT3D_EQ(node.getCoordinate(), point3d(-.25 + dx, -.05 + dy, .45 + dz));
  EXPECT_NEAR(node->getValue(), 0.4, 1e-5);
  ++node;
  ASSERT_NE(node, transformedTree->end());
  EXPECT_POINT3D_EQ(node.getCoordinate(), point3d(-.05 + dx, -.05 + dy, .05 + dz));
  EXPECT_NEAR(node->getValue(), 0.6, 1e-5);
}
