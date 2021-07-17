#include <gtest/gtest.h>
#include <iostream>

#include <pcl/point_cloud.h>

#include <common/transformations.h>
#include <common/utils.h>
#include <common/math.h>
#include <icp.h>
#include <common/conversions.h>

#include "test_utils.h"

using namespace octomap_tools;
using namespace octomap;

class IcpMatchingTest : public ::testing::Test {
 public:
  IcpMatchingTest() {
    Configure();
  }

  void Configure() {
    cfg_.max_iter = 500;
    cfg_.max_nn_dist = 0.5;
    cfg_.fitness_score_dist = 0.5;
    cfg_.fitness_eps = 0.0005;
    cfg_.transf_eps = 0.0001;
    cfg_.scene_inflation_dist = 2.5;
    cfg_.visualize = true;
    cfg_.crop_scene = false;
  }

  ICP::Config cfg_;
};

TEST_F(IcpMatchingTest, Test_fr) {
  auto original_tree = unpackAndGetOctomap("fr_079");
  PrintOcTreeInfo(*original_tree, "original_tree");

  auto scene = CropOcTree(*original_tree, Vector3f(-8, -8, 0.0), Vector3f(8, 8, 2.0));
  auto init_model = CropOcTree(*original_tree, Vector3f(-5, -5, 0.0), Vector3f(5, 5, 2.0));

  // Transform model
  auto T = createTransformationMatrix(0.5, 0.0, 0.0, ToRad(0), ToRad(0), ToRad(10.0));
  auto model = FastOcTreeTransform(*init_model, T);

  auto scene_cloud = OcTreeToPointCloud(*scene);
  auto model_cloud = OcTreeToPointCloud(*model);

  ICP matcher(scene_cloud, model_cloud, cfg_);
  auto result = matcher.Align();

  PrintMatchingResult(T, result.transformation.inverse(), result.fitness_score1);
}

