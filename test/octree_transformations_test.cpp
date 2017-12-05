/******************************************************************************
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <gtest/gtest.h>

#include <pcl/registration/transforms.h>
#include "md_utils/math/transformations.h"
#include "octree_transformations.h"
#include "utils/octree_utils.h"
#include "utils/types_conversions.h"

#include "test_utils.h"

using namespace octomap_tools;
using namespace octomap;
using namespace md;
using namespace Eigen;

#define EXPECT_POINT3D_EQ(n1, n2) \
    EXPECT_NEAR(n1.x(), n2.x(), 1e-5); \
    EXPECT_NEAR(n1.y(), n2.y(), 1e-5); \
    EXPECT_NEAR(n1.z(), n2.z(), 1e-5);

#define EXPECT_VECTOR3F_NEAR(n1, n2, val) \
    EXPECT_NEAR(n1[0], n2[0], val); \
    EXPECT_NEAR(n1[1], n2[1], val); \
    EXPECT_NEAR(n1[2], n2[2], val)

void checkIfTransformedTreeBoundsAreCorrect(
    const OcTree& tree, const OcTree& transf_tree, const Eigen::Matrix4f& transf) {
  Vector3f min1, max1;
  getMinMaxOctree(tree, min1, max1);
  Vector3f min2, max2;
  getMinMaxOctree(transf_tree, min2, max2);

  auto R = transf.block<3,3>(0,0);
  auto T = transf.block<3,1>(0,3);

  Vector3f min1t = R * min1 + T;
  Vector3f max1t = R * max1 + T;

  EXPECT_VECTOR3F_NEAR(min1t, min2, 0.1);
  EXPECT_VECTOR3F_NEAR(max1t, max2, 0.1);
}

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

TEST(OctreeTransformationsTest, transformOcTreeFromDataset_ManyCases_CheckBounds)
{
  auto tree_in = unpackAndGetOctomap("fr_079");
  Vector3f o_min, o_max;
  getMinMaxOctree(*tree_in, o_min, o_max);
  Vector3f tree_min = o_min + Vector3f{25, 8, 0.5};
  Vector3f tree_max = o_max - Vector3f{5, 4, 2.2};
  auto tree = cutOctree(*tree_in, tree_min, tree_max);

  std::vector<std::vector<float>> transformations = {
      {0,   0,   0,   0,   0,   0  },
      {0.5, 0,   0,   0,   0,   0  },
      {0,   0.5, 0,   0,   0,   0  },
      {0,   0,   0.5, 0,   0,   0  },
      {0,   0,   0,   0.5, 0,   0  },
      {0,   0,   0,   0,   0.5, 0  },
      {0,   0,   0,   0,   0,   0.5},
      {0.5, 0.5, 0.5, 0,   0,   0  },
      {0,   0,   0,   0.5, 0.5, 0.5},
      {0.5, 0.5, 0.5, 0.5, 0.5, 0.5},
  };

  for (const auto& t : transformations)
  {
    std::cout << "--- Checking xyz rpy: " << t[0] << " " << t[1] << " " << t[2] << " "
              << t[3] << " " << t[4] << " " << t[5] << std::endl;
    auto transf = createTransformationMatrix(t[0], t[1], t[2],
                                             ToRad(t[3]), ToRad(t[4]), ToRad(t[5]));
    auto transf_tree = transformOctree(tree, transf);
    checkIfTransformedTreeBoundsAreCorrect(tree, *transf_tree, transf);
  }
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

TEST(OctreeTransformationsTest, ExtractIntersectingOctrees_CommonPartExist)
{
  // Cloud with ranges from -10 to 10
  auto cloud1 = createUniformPointCloud(
      Point{-10,-10,-10}, Point{10,10,10}, Point{1,1,1});

  // Cloud with ranges from 0 to 20
  auto cloud2 = createUniformPointCloud(
      Point{0,0,0}, Point{20,20,20}, Point{1,1,1});

  auto tree1 = PointCloudToOctree(cloud1, 0.5);
  auto tree2 = PointCloudToOctree(cloud2, 0.5);
  OcTree filtered_tree1(1), filtered_tree2(1);
  Point margin {1,1,1};
  extractIntersectingOctrees(tree1, tree2, margin, filtered_tree1, filtered_tree2);

  // Common part is from 0 to 10 and that points should be in each pointcloud
  const Point minRange{0, 0, 0};
  const Point maxRange{10, 10, 10};

  EXPECT_GT(filtered_tree1.getNumLeafNodes(), 0U);
  EXPECT_GT(filtered_tree2.getNumLeafNodes(), 0U);

  for (auto i = filtered_tree1.begin_leafs(); i != filtered_tree1.end_leafs(); ++i)
  {
    auto p = i.getCoordinate();
    EXPECT_TRUE(p.x() >= minRange.x && p.x() <= maxRange.x);
    EXPECT_TRUE(p.y() >= minRange.y && p.y() <= maxRange.y);
    EXPECT_TRUE(p.z() >= minRange.z && p.z() <= maxRange.z);
  }

  for (auto i = filtered_tree2.begin_leafs(); i != filtered_tree2.end_leafs(); ++i)
  {
    auto p = i.getCoordinate();
    EXPECT_TRUE(p.x() >= minRange.x && p.x() <= maxRange.x);
    EXPECT_TRUE(p.y() >= minRange.y && p.y() <= maxRange.y);
    EXPECT_TRUE(p.z() >= minRange.z && p.z() <= maxRange.z);
  }
}

TEST(OctreeTransformationsTest, ExtractIntersectingOctrees_CommonPartNotExist)
{
  // Cloud with ranges from -10 to 10
  auto cloud1 = createUniformPointCloud(
      Point{-10,-10,-10}, Point{10,10,10}, Point{1,1,1});

  // Cloud with ranges from 20 to 30
  auto cloud2 = createUniformPointCloud(
      Point{20,20,20}, Point{30,30,30}, Point{1,1,1});

  auto tree1 = PointCloudToOctree(cloud1, 0.5);
  auto tree2 = PointCloudToOctree(cloud2, 0.5);
  OcTree filtered_tree1(1), filtered_tree2(1);
  Point margin {1,1,1};
  extractIntersectingOctrees(tree1, tree2, margin, filtered_tree1, filtered_tree2);

  EXPECT_EQ(filtered_tree1.getNumLeafNodes(), 0U);
  EXPECT_EQ(filtered_tree2.getNumLeafNodes(), 0U);
}
