/******************************************************************************
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include <octomap_tools/transformations.h>
#include <octomap_tools/utils.h>
#include <octomap_tools/math.h>
#include <octomap_tools/conversions.h>
#include <pcl/visualization/pcl_visualizer.h>


#include "test_utils.h"

using namespace octomap;
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
    auto transformed_tree = FastOcTreeTransform(tree, T);
    auto diff = duration_cast<milliseconds>(high_resolution_clock::now() - start);
    std::cout << "Transformation time: " << diff.count() << " ms." << std::endl;

    validate(tree, transformed_tree, tree_leafs, T);
  }

  void validate(const OcTree& tree, const OcTreePtr& transformed_tree,
                const NodesList& tree_leafs, const Eigen::Matrix4f& T) {

    ASSERT_EQ(tree.getNumLeafNodes(), transformed_tree->getNumLeafNodes());
    ASSERT_EQ(tree.getResolution(), transformed_tree->getResolution());

    double res = tree.getResolution();
    const float logodds_abs_err = 100;

    for (auto i = transformed_tree->begin_leafs(); i != transformed_tree->end_leafs(); ++i) {
      const auto& point = i.getCoordinate();
      Eigen::Vector4f p = inverseTransform(T) * Eigen::Vector4f{point.x(), point.y(), point.z(), 1};
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

  NodesList CreateListOfLeafs(const OcTree& tree) {
    NodesList list;
    for (auto i = tree.begin_leafs(); i != tree.end_leafs(); ++i) {
      auto p = i.getCoordinate();
      list.push_back(std::pair<point3d, float>(p, i->getValue()));
    }
    return list;
  }

  void PrepareOcTree(
      std::string octomap_packed_file,
      Vector3f octomap_min = {0,0,0}, Vector3f octomap_max = {0,0,0}) {
    auto orig_tree = unpackAndGetOctomap(octomap_packed_file);
    PrintOcTreeInfo(*orig_tree, "orig_tree");

    if (octomap_min != Eigen::Vector3f{0,0,0} && octomap_max != Eigen::Vector3f{0,0,0}) {
      cropped_tree_ = CropOcTree(*orig_tree, octomap_min, octomap_max);
      PrintOcTreeInfo(*cropped_tree_, "cropped_tree");
    } else {
      cropped_tree_ = orig_tree;
    }
  }

  void VisualizeOctreesAsPointClouds(OcTreePtr& tree, OcTreePtr& transformed_tree) {
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addCoordinateSystem(1.0);

    // Model --> BLUE
    auto cloud  = OcTreeToPointCloud(*tree);
    pcl::visualization::PointCloudColorHandlerCustom<Point> model_color(cloud, 0, 0, 255);
    viewer.addPointCloud(cloud, model_color, "cropped_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cropped_cloud");

    // Model after transformation --> GREEN
    auto transformed_cloud  = OcTreeToPointCloud(*transformed_tree);
    pcl::visualization::PointCloudColorHandlerCustom<Point> transformed_model_color_handler(transformed_cloud, 0, 255, 0);
    viewer.addPointCloud(transformed_cloud, transformed_model_color_handler, "transformed_model");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_model");

    while (!viewer.wasStopped()) {
      viewer.spinOnce(100);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  OcTreePtr cropped_tree_;
  bool visualize = true;
};

TEST_F(OctreeTransformationsTest, Translation) {
  const float res = 0.1;
  const auto T = createTransformationMatrix(0.2, 0, 0, 0, 0, 0);
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

TEST_F(OctreeTransformationsTest, Transform_ThreeNodes_MultiCase) {
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

TEST_F(OctreeTransformationsTest, Transform_RealOcTreeTest)
{
  visualize = false;
  auto tree_in = unpackAndGetOctomap("fr_079");
  Vector3f o_min, o_max;
  Vector3f tree_min = {12, 2, 0.0};
  Vector3f tree_max = {14, 3, 2.0};
  auto tree = CropOcTree(*tree_in, tree_min, tree_max);

  std::vector<std::vector<float>> transformations = {
      {1.0,   0,   0,   0.1,   0, 0  },
  };

  for (const auto& t : transformations) {
    std::cout << "--- Checking xyz rpy: " << t[0] << " " << t[1] << " " << t[2] << " "
              << t[3] << " " << t[4] << " " << t[5] << std::endl;
    auto T = createTransformationMatrix(t[0], t[1], t[2], ToRad(t[3]), ToRad(t[4]), ToRad(t[5]));
    auto tree_leafs = CreateListOfLeafs(*tree);
    transformAndValidate(tree->getResolution(), tree_leafs, T);
  }
}


TEST_F(OctreeTransformationsTest, FastOctreeTransform_Performance) {
  const float res = 0.1;
  const auto T = createTransformationMatrix(0.1, 0, 0, ToRad(1), ToRad(1.5), ToRad(180));

  Vector3f min = {-1000, -1000, -100};
  Vector3f max = {1000, 1000, 1000};

  std::cout << "tree_size;time_s\n";

  for (unsigned i = 1; i <= 50000000; i *= 2) {

    // Create a tree
    OcTree tree(res);
    unsigned cnt = 1;
    for (float z = min.z() + res / 2; z < max.z() - res / 2; z += res) {
      for (float y = min.y() + res / 2; y < max.y() - res / 2; y += res) {
        for (float x = min.x() + res / 2; x < max.x() - res / 2; x += res) {
          if (cnt++ > i)
            break;
          tree.setNodeValue(point3d(x, y, z), 0.5);
        }
      }
    }

    auto start = high_resolution_clock::now();
    FastOcTreeTransform(tree, T);
    auto diff = duration_cast<microseconds>(high_resolution_clock::now() - start);
    std::cout << tree.size() << ";" << std::fixed << std::setprecision(6) << (diff.count() / 1000000.0) << "\n";
  }
}

TEST_F(OctreeTransformationsTest, OctreeTransform_Performance) {
  const float res = 0.1;
  const auto T = createTransformationMatrix(0.1, 0, 0, ToRad(1), ToRad(1.5), ToRad(180));

  Vector3f min = {-1000, -1000, -100};
  Vector3f max = {1000, 1000, 1000};

  std::cout << "Tree size | time[s]\n";

  for (unsigned i = 1; i <= 100000; i *= 1.5) {

    // Create a tree
    OcTree tree(res);
    unsigned cnt = 1;
    for (float z = min.z() + res / 2; z < max.z() - res / 2; z += res) {
      for (float y = min.y() + res / 2; y < max.y() - res / 2; y += res) {
        for (float x = min.x() + res / 2; x < max.x() - res / 2; x += res) {
          if (cnt++ > i)
            break;
          tree.setNodeValue(point3d(x, y, z), 0.5);
        }
      }
    }

    auto start = high_resolution_clock::now();
    OcTreeTransform(tree, T);
    auto diff = duration_cast<microseconds>(high_resolution_clock::now() - start);
    std::cout << tree.size() << "; " << std::fixed << std::setprecision(6) << (diff.count() / 1000000.0) << "\n";
  }
}

TEST_F(OctreeTransformationsTest, FastMethod_fr) {
  std::string octomap_name = "fr_079";
  auto cloud_min = Vector3f(-5, -5, 0.0);
  auto cloud_max = Vector3f(5, 5, 2.0);
  PrepareOcTree(octomap_name, cloud_min, cloud_max);

  auto T = createTransformationMatrix(20.1, 0, 0.0, ToRad(0), ToRad(0), ToRad(90));

  auto start = std::chrono::high_resolution_clock::now();
  auto transformed_tree = FastOcTreeTransform(*cropped_tree_, T);
  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);

  std::cout << "\nOcTree transformed.\n";
  std::cout << OcTreeInfoToString(*cropped_tree_, "input tree");
  std::cout << OcTreeInfoToString(*transformed_tree, "transformed tree");
  std::cout << "Transformed in: " << diff.count() << " ms." << std::endl;

  start = std::chrono::high_resolution_clock::now();
  auto transformed_tree2 = OcTreeTransform(*cropped_tree_, T);
  diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);

  std::cout << "\nOcTree transformed.\n";
  std::cout << OcTreeInfoToString(*cropped_tree_, "input tree");
  std::cout << OcTreeInfoToString(*transformed_tree2, "transformed tree");
  std::cout << "Transformed in: " << diff.count() << " ms." << std::endl;

  VisualizeOctreesAsPointClouds(transformed_tree2, transformed_tree);

  EXPECT_GT(getNumberOfOccupiedNodes(*transformed_tree), 42000);
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
