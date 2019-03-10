/******************************************************************************
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <gtest/gtest.h>

#include <chrono>

#include <pcl/registration/transforms.h>
#include "md_utils/math/transformations.h"
#include <octomap_tools/transformations.h>
#include <octomap_tools/types_conversions.h>
#include <octomap_tools/utils.h>

#include "test_utils.h"

using namespace octomap;
using namespace md;
using namespace Eigen;
using namespace std::chrono;

namespace octomap_tools {

class OctreeTransformationsTest : public ::testing::Test {
 public:
  using NodesList = std::vector<std::pair<point3d, float>>;

  void transform(float resolution, NodesList tree_leafs, const Eigen::Matrix4f& T) {
    OcTree tree(resolution);
    for (auto i : tree_leafs) {
      tree.setNodeValue(i.first, i.second);
    }

    auto start = high_resolution_clock::now();
    auto transformed_tree = transformOctree(tree, T);
    auto diff = duration_cast<milliseconds>(high_resolution_clock::now() - start);
    std::cout << "Transformation time: " << diff.count() << " ms." << std::endl;

    EXPECT_EQ(tree.getNumLeafNodes(), transformed_tree->getNumLeafNodes());
    EXPECT_EQ(tree.getResolution(), transformed_tree->getResolution());

    for (auto i = transformed_tree->begin_leafs(); i != transformed_tree->end_leafs(); ++i) {
      const auto& point = i.getCoordinate();
      Eigen::Vector4f p = T.inverse() * Eigen::Vector4f{point.x(), point.y(), point.z(), 0};
      const auto& v = i->getValue();

      // Find near points in initial set
      auto it = std::find_if(tree_leafs.begin(), tree_leafs.end(), [p, v, resolution](const auto& val) {
        return (near_abs(v, val.second, 1e-1f) &&
            near_abs(p.x(), val.first.x(), resolution / 2) &&
            near_abs(p.y(), val.first.y(), resolution / 2) &&
            near_abs(p.z(), val.first.z(), resolution / 2));
      });
      EXPECT_TRUE(it != tree_leafs.end());
    }

    if (::testing::Test::HasFailure()) {
      printOcTree(tree, "Tree");
      printOcTree(*transformed_tree, "Transformed tree");
    }
  }
};

TEST_F(OctreeTransformationsTest, Translation) {
  const float res = 0.1;
  const auto T = createTransformationMatrix(0, 0, 0, 0, 0, 3.14);
  const std::vector<std::pair<point3d, float>> tree_leafs = {
      { point3d{0.05, 0.05, 0.05}, 0.6},
      { point3d{0.25, 0.15, 0.05}, 0.5},
      { point3d{-0.15, 0.05, 0.05}, 0.4}
  };

  transform(res, tree_leafs, T);

}

//TEST(OctreeTransformationsTest, transformOcTreeFromDataset_ManyCases_CheckBounds)
//{
//  auto tree_in = unpackAndGetOctomap("fr_079");
//  Vector3f o_min, o_max;
//  getMinMaxOctree(*tree_in, o_min, o_max);
//  Vector3f tree_min = o_min + Vector3f{25, 8, 0.5};
//  Vector3f tree_max = o_max - Vector3f{5, 4, 2.2};
//  auto tree = cutOctree(*tree_in, tree_min, tree_max);
//
//  std::vector<std::vector<float>> transformations = {
////      {0,   0,   0,   0,   0,   0  },
////      {0.5, 0,   0,   0,   0,   0  },
////      {0,   0.5, 0,   0,   0,   0  },
////      {0,   0,   0.5, 0,   0,   0  },
////      {0,   0,   0,   0.5, 0,   0  },
//      {0,   0,   0,   0,   5.0, 0  },
////      {0,   0,   0,   0,   0,   0.5},
////      {0.5, 0.5, 0.5, 0,   0,   0  },
////      {0,   0,   0,   0.5, 0.5, 0.5},
////      {0.5, 0.5, 0.5, 0.5, 0.5, 0.5},
//  };
//
//  for (const auto& t : transformations)
//  {
//    std::cout << "--- Checking xyz rpy: " << t[0] << " " << t[1] << " " << t[2] << " "
//              << t[3] << " " << t[4] << " " << t[5] << std::endl;
//    auto transf = createTransformationMatrix(t[0], t[1], t[2],
//                                             ToRad(t[3]), ToRad(t[4]), ToRad(t[5]));
//    auto transf_tree = transformOctree(tree, transf);
//    checkIfTransformedTreeBoundsAreCorrect(tree, *transf_tree, transf);
//  }
//}
//
//TEST(OctreeTransformationsTest, transformOcTree_TranslationAndRotation)
//{
//  constexpr float res = 0.1;
//  OcTree tree(res);
//  tree.setNodeValue(0.05, 0.05, 0.05, 0.6);
//  tree.setNodeValue(0.25, 0.05, 0.05, 0.5);
//  tree.setNodeValue(0.25, 0.05, 0.45, 0.4);
//
//  float dx = 0.1, dy = -0.0, dz = 0.0;
//  float roll = 0 * M_PI / 180, pitch = 0 * M_PI / 180, yaw = 180 * M_PI / 180;
//  auto transform = createTransformationMatrix(dx, dy, dz, roll, pitch, yaw);
//
//  auto transformedTree = transformOctree(tree, transform);
//
//  //  printOcTree(tree, "Tree");
//  //  printOcTree(*transformedTree, "Transformed tree");
//
//  auto node = transformedTree->begin();
//  ASSERT_NE(node, transformedTree->end());
//  EXPECT_POINT3D_EQ(node.getCoordinate(), point3d(-.25 + dx, -.05 + dy, .05 + dz));
//  EXPECT_NEAR(node->getValue(), 0.5, 1e-5);
//  ++node;
//  ASSERT_NE(node, transformedTree->end());
//  EXPECT_POINT3D_EQ(node.getCoordinate(), point3d(-.25 + dx, -.05 + dy, .45 + dz));
//  EXPECT_NEAR(node->getValue(), 0.4, 1e-5);
//  ++node;
//  ASSERT_NE(node, transformedTree->end());
//  EXPECT_POINT3D_EQ(node.getCoordinate(), point3d(-.05 + dx, -.05 + dy, .05 + dz));
//  EXPECT_NEAR(node->getValue(), 0.6, 1e-5);
//}

//TEST(OctreeTransformationsTest, ExtractIntersectingOctrees_CommonPartExist)
//{
//  // Cloud with ranges from -10 to 10
//  auto cloud1 = createUniformPointCloud(
//      Point{-10,-10,-10}, Point{10,10,10}, Point{1,1,1});
//
//  // Cloud with ranges from 0 to 20
//  auto cloud2 = createUniformPointCloud(
//      Point{0,0,0}, Point{20,20,20}, Point{1,1,1});
//
//  auto tree1 = PointCloudToOctree(cloud1, 0.5);
//  auto tree2 = PointCloudToOctree(cloud2, 0.5);
//  OcTree filtered_tree1(1), filtered_tree2(1);
//  Point margin {1,1,1};
//  extractIntersectingOctrees(tree1, tree2, margin, filtered_tree1, filtered_tree2);
//
//  // Common part is from 0 to 10 and that points should be in each pointcloud
//  const Point minRange{0, 0, 0};
//  const Point maxRange{10, 10, 10};
//
//  EXPECT_GT(filtered_tree1.getNumLeafNodes(), 0U);
//  EXPECT_GT(filtered_tree2.getNumLeafNodes(), 0U);
//
//  for (auto i = filtered_tree1.begin_leafs(); i != filtered_tree1.end_leafs(); ++i)
//  {
//    auto p = i.getCoordinate();
//    EXPECT_TRUE(p.x() >= minRange.x && p.x() <= maxRange.x);
//    EXPECT_TRUE(p.y() >= minRange.y && p.y() <= maxRange.y);
//    EXPECT_TRUE(p.z() >= minRange.z && p.z() <= maxRange.z);
//  }
//
//  for (auto i = filtered_tree2.begin_leafs(); i != filtered_tree2.end_leafs(); ++i)
//  {
//    auto p = i.getCoordinate();
//    EXPECT_TRUE(p.x() >= minRange.x && p.x() <= maxRange.x);
//    EXPECT_TRUE(p.y() >= minRange.y && p.y() <= maxRange.y);
//    EXPECT_TRUE(p.z() >= minRange.z && p.z() <= maxRange.z);
//  }
//}
//
//TEST(OctreeTransformationsTest, ExtractIntersectingOctrees_CommonPartNotExist)
//{
//  // Cloud with ranges from -10 to 10
//  auto cloud1 = createUniformPointCloud(
//      Point{-10,-10,-10}, Point{10,10,10}, Point{1,1,1});
//
//  // Cloud with ranges from 20 to 30
//  auto cloud2 = createUniformPointCloud(
//      Point{20,20,20}, Point{30,30,30}, Point{1,1,1});
//
//  auto tree1 = PointCloudToOctree(cloud1, 0.5);
//  auto tree2 = PointCloudToOctree(cloud2, 0.5);
//  OcTree filtered_tree1(1), filtered_tree2(1);
//  Point margin {1,1,1};
//  extractIntersectingOctrees(tree1, tree2, margin, filtered_tree1, filtered_tree2);
//
//  EXPECT_EQ(filtered_tree1.getNumLeafNodes(), 0U);
//  EXPECT_EQ(filtered_tree2.getNumLeafNodes(), 0U);
//}

}
