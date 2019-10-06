/******************************************************************************
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <gtest/gtest.h>

#include <chrono>

#include <pcl/registration/transforms.h>
#include "md_utils/math/transformations.h"
#include <octomap_tools/transformations.h>
#include <octomap_tools/utils.h>
#include "../include/octomap_tools/conversions.h"

#include "test_utils.h"

using namespace octomap;
using namespace md;
using namespace Eigen;
using namespace std::chrono;

namespace octomap_tools {

class OctreeTransformationsTest : public ::testing::Test {
 public:
  using NodesList = std::vector<std::pair<point3d, float>>;

  void transformAndValidate(float resolution, const NodesList& tree_leafs, const Eigen::Matrix4f& T) {
    OcTree tree(resolution);
    for (auto i : tree_leafs) {
      tree.setNodeValue(i.first, i.second);
    }

    auto start = high_resolution_clock::now();
    auto transformed_tree = transformOctree(tree, T);
    auto diff = duration_cast<milliseconds>(high_resolution_clock::now() - start);
    std::cout << "Transformation time: " << diff.count() << " ms." << std::endl;

    validate(tree, transformed_tree, tree_leafs, T);
  }

  void transform(float resolution, const NodesList& tree_leafs, const Eigen::Matrix4f& T) {
    OcTree tree(resolution);
    for (auto i : tree_leafs) {
      tree.setNodeValue(i.first, i.second);
    }

    auto start = high_resolution_clock::now();
    auto transformed_tree = transformOctree(tree, T);
    auto diff = duration_cast<milliseconds>(high_resolution_clock::now() - start);
    std::cout << "Transformation time: " << diff.count() << " ms." << std::endl;
  }

  void validate(const OcTree& tree, const OcTreePtr& transformed_tree,
                const NodesList& tree_leafs, const Eigen::Matrix4f& T) {

    ASSERT_EQ(tree.getNumLeafNodes(), transformed_tree->getNumLeafNodes());
    ASSERT_EQ(tree.getResolution(), transformed_tree->getResolution());

    double res = tree.getResolution();
    const float logodds_abs_err = 100;

    for (auto i = transformed_tree->begin_leafs(); i != transformed_tree->end_leafs(); ++i) {
      const auto& point = i.getCoordinate();
      Eigen::Vector4f p = T.inverse() * Eigen::Vector4f{point.x(), point.y(), point.z(), 0};
      const auto& v = i->getValue();

      // Find near points in initial set
      auto it = std::find_if(tree_leafs.begin(), tree_leafs.end(),
                             [p, v, res, logodds_abs_err](const auto& val) {
        return (near_abs<float>(v, val.second, logodds_abs_err) &&
            near_abs<float>(p.x(), val.first.x(), res / 2) &&
            near_abs<float>(p.y(), val.first.y(), res / 2) &&
            near_abs<float>(p.z(), val.first.z(), res / 2));
      });
      EXPECT_TRUE(it != tree_leafs.end());

      if (::testing::Test::HasFailure() && visualize) {
        std::cout << "Transformed point: (" << point.x() << ", " << point.y() << ", " << point.z() << ") -> "
            << "orig point: (" << p.x() << ", " << p.y() << ", " << p.z() << ")\n";
      }
    }

    if (::testing::Test::HasFailure() && visualize) {
      printOcTree(tree, "Tree");
      printOcTree(*transformed_tree, "Transformed tree");
    }
  }

  NodesList createListOfLeafs(const OcTree& tree) {
    NodesList list;
    for (auto i = tree.begin_leafs(); i != tree.end_leafs(); ++i) {
      auto p = i.getCoordinate();
      list.push_back(std::pair<point3d, float>(p, i->getValue()));
    }
    return list;
  }

  bool visualize = true;
};

TEST_F(OctreeTransformationsTest, Translation) {
  const float res = 0.1;
  const auto T = createTransformationMatrix(0, 0, 0, 0, 0, 3.14);
  const std::vector<std::pair<point3d, float>> tree_leafs = {
      { point3d{0.05, 0.05, 0.05}, 0.6},
      { point3d{0.25, 0.15, 0.05}, 0.5},
      { point3d{-0.15, 0.05, 0.05}, 0.4}
  };

  transformAndValidate(res, tree_leafs, T);
}

TEST_F(OctreeTransformationsTest, Rotation) {
  const float res = 0.1;
  const auto T = createTransformationMatrix(0.0, 0, 0, ToRad(0), ToRad(180), ToRad(180));
  const std::vector<std::pair<point3d, float>> tree_leafs = {
      { point3d{0.05, 0.05, 0.05}, 0.6},
      { point3d{0.25, 0.05, 0.05}, 0.5},
      { point3d{0.25, 0.05, 0.45}, 0.4}
  };

  transformAndValidate(res, tree_leafs, T);
}

TEST_F(OctreeTransformationsTest, ThreeNodes_MultiCase) {
  const float res = 0.1;
  const std::vector<std::pair<point3d, float>> tree_leafs = {
      { point3d{0.05, 0.05, 0.05}, 0.6},
      { point3d{0.25, 0.05, 0.05}, 0.5},
      { point3d{0.25, 0.05, 0.45}, 0.4}
  };

  std::vector<std::vector<float>> transformations = {
      {0,   0,   0,   0,   0,   0  },
      {0.5, 0,   0,   0,   0,   0  },
      {0,   0.5, 0,   0,   0,   0  },
      {0,   0,   0.5, 0,   0,   0  },
      {0,   0,   0,   0.5, 0,   0  },
      {0,   0,   0,   0,   0,5, 0  },
      {0,   0,   0,   0,   0,   0.5},
      {0.5, 0.5, 0.5, 0,   0,   0  },
      {0,   0,   0,   0.5, 0.5, 0.5},
      {0.5, 0.5, 0.5, 0.5, 0.5, 0.5},
  };

  for (const auto& t : transformations) {
    std::cout << "--- Checking xyz rpy: " << t[0] << " " << t[1] << " " << t[2] << " "
              << t[3] << " " << t[4] << " " << t[5] << std::endl;
    auto T = createTransformationMatrix(t[0], t[1], t[2], ToRad(t[3]), ToRad(t[4]), ToRad(t[5]));
    transformAndValidate(res, tree_leafs, T);
  }
}

TEST_F(OctreeTransformationsTest, RealOcTreeTest)
{
  visualize = false;
  auto tree_in = unpackAndGetOctomap("fr_079");
  Vector3f o_min, o_max;
  Vector3f tree_min = {12, 2, 0.0};
  Vector3f tree_max = {14, 3, 2.0};
  auto tree = cutOctree(*tree_in, tree_min, tree_max);

  std::vector<std::vector<float>> transformations = {
      {0,   0,   0,   0,   5.0, 0  },
  };

  for (const auto& t : transformations) {
    std::cout << "--- Checking xyz rpy: " << t[0] << " " << t[1] << " " << t[2] << " "
              << t[3] << " " << t[4] << " " << t[5] << std::endl;
    auto T = createTransformationMatrix(t[0], t[1], t[2], ToRad(t[3]), ToRad(t[4]), ToRad(t[5]));
    auto tree_leafs = createListOfLeafs(tree);
    transformAndValidate(tree.getResolution(), tree_leafs, T);
  }
}

TEST_F(OctreeTransformationsTest, Performance)
{
  visualize = false;
  const float res = 0.1;
  const auto T = createTransformationMatrix(0.1, 0, 0, ToRad(1), ToRad(1.5), ToRad(180));

  Vector3f min = {-100, -100, -100};
  Vector3f max = {100, 1000, 1000};

  for (unsigned i = 1; i <= 100000; i *= 10) {
    NodesList tree_leafs;
    unsigned cnt = 1;
    for (float z = min.z() + res / 2; z < max.z() - res / 2; z += res) {
      for (float y = min.y() + res / 2; y < max.y() - res / 2; y += res) {
        for (float x = min.x() + res / 2; x < max.x() - res / 2; x += res) {
          if (cnt++ > i )
            break;
          tree_leafs.push_back(std::make_pair(point3d(x, y, z), 0.5));
        }
      }
    }
    std::cout << "\nSize: " << i << "\n";
    transform(res, tree_leafs, T);
  }
}

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
