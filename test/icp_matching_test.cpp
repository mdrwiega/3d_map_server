#include <gtest/gtest.h>
#include <iostream>

#include <pcl/point_cloud.h>

#include <octomap_tools/transformations.h>
#include <octomap_tools/utils.h>
#include <octomap_tools/math.h>
#include <octomap_tools/feature_cloud.h>
#include <octomap_tools/icp.h>
#include <octomap_tools/conversions.h>

#include "test_utils.h"

using namespace octomap_tools;
using namespace octomap;

class IcpMatchingTest : public ::testing::Test
{
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

  void PrepareOcTree(std::string octomap_packed_file,
                     Vector3f octomap_min = {0,0,0}, Vector3f octomap_max = {0,0,0}) {
    orig_tree_ = unpackAndGetOctomap(octomap_packed_file);
    PrintOcTreeInfo(*orig_tree_, "orig_tree");

    if (octomap_min != Vector3f{0,0,0} && octomap_max != Vector3f{0,0,0}) {
      cropped_tree_ = CropOcTree(*orig_tree_, octomap_min, octomap_max);
      PrintOcTreeInfo(*cropped_tree_, "cropped_tree");
    } else {
      cropped_tree_ = orig_tree_;
    }
  }

  OcTreePtr orig_tree_;
  OcTreePtr cropped_tree_;
  ICP::Config cfg_;
  Eigen::Matrix4f result_transf_;
};

TEST_F(IcpMatchingTest, Test_fr)
{
  std::string octomap_name = "fr_079";
  auto map_min = Vector3f(-5, -5, 0.0);
  auto map_max = Vector3f(5, 5, 2.0);
  PrepareOcTree(octomap_name, map_min, map_max);

  auto T = createTransformationMatrix(0.5, 0.0, 0.0, ToRad(0), ToRad(0), ToRad(10.0));
  auto tree_model = FastOcTreeTransform(*cropped_tree_, T);

  auto scene_cloud = OcTreeToPointCloud(*cropped_tree_);
  auto model_cloud = OcTreeToPointCloud(*tree_model);

  ICP matcher(scene_cloud, model_cloud, cfg_);
  auto result = matcher.Align();

  std::cout << "\nReal transformation between maps:\n" << transformationMatrixToString(T);
  auto rpy_real = ToRad(rotMatrixToRPY(T.block<3,3>(0,0)));
  std::cout << "RPY: (" << rpy_real[0] << ", " << rpy_real[1] << ", " << rpy_real[2] << ")\n";

  result_transf_ = result.transformation;
  std::cout << "\n\nEstimated transformation between maps:\n" << transformationMatrixToString(result_transf_);
  auto rpy_est = ToRad(rotMatrixToRPY(result_transf_.block<3,3>(0,0)));
  std::cout << "RPY: (" << rpy_est[0] << ", " << rpy_est[1] << ", " << rpy_est[2] << ")\n";
}

