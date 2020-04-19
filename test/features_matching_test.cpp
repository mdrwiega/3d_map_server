
#include <gtest/gtest.h>
#include <iostream>

#include <pcl/point_cloud.h>

#include <octomap_tools/transformations.h>
#include <octomap_tools/utils.h>
#include <octomap_tools/math.h>
#include <octomap_tools/feature_cloud.h>
#include <octomap_tools/features_matching.h>
#include <octomap_tools/conversions.h>

#include "test_utils.h"

using namespace octomap_tools;
using namespace octomap;

class FeaturesMatchingTest : public ::testing::Test
{
 public:
  FeaturesMatchingTest() {
    Configure();
  }

  void Configure() {
    cfg_.feature_cloud.normal_radius = 15.0;
    cfg_.feature_cloud.downsampling_radius = 0.15;
    cfg_.feature_cloud.descriptors_radius = 1.5;
    cfg_.feature_cloud.keypoints_method = FeatureCloud::KeypointsExtractionMethod::Iss3d;
    cfg_.feature_cloud.debug = true;

    cfg_.nr_iterations = 1000;
    cfg_.min_sample_distance = 0.2;
    cfg_.max_correspondence_distance = 100.0;
    cfg_.fitness_score_dist = 0.5;
    cfg_.cell_size_x = 3;
    cfg_.cell_size_y = 3;
    cfg_.model_size_thresh_ = 400;
    cfg_.keypoints_thresh_ = 150;

    cfg_.method = FeaturesMatching::AlignmentMethod::GeometryConsistencyClustering;
    // cfg_.method = FeaturesMatching::AlignmentMethod::SampleConsensus;
    cfg_.visualize = true;
    cfg_.debug = true;
  }

  void PrepareOcTree(
      std::string octomap_packed_file,
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
  FeaturesMatching::Config cfg_;
  FeatureCloudPtr scene_;
  FeatureCloudPtr model_;
  Eigen::Matrix4f result_transf_;
};

TEST_F(FeaturesMatchingTest, Test_fr)
{
  std::string octomap_name = "fr_079";
  auto map_min = Vector3f(-5, -5, 0.0);
  auto map_max = Vector3f(5, 5, 2.0);
  PrepareOcTree(octomap_name, map_min, map_max);

  auto T = createTransformationMatrix(15.0, 0.5, 0.0, ToRad(0), ToRad(0), ToRad(15));
  auto tree_model = FastOcTreeTransform(*cropped_tree_, T);

  auto scene_cloud = OcTreeToPointCloud(*cropped_tree_);
  auto model_cloud = OcTreeToPointCloud(*tree_model);

  // Prepare scene
  scene_ = std::make_shared<FeatureCloud>(scene_cloud, cfg_.feature_cloud);
  std::cout << "\nScene size: " << scene_->GetPointCloud()->size() << std::endl;
  scene_->ComputeDescriptors();

  // Prepare model
  model_ = std::make_shared<FeatureCloud>(model_cloud, cfg_.feature_cloud);
  std::cout << "\nModel size: " << model_->GetPointCloud()->size() << std::endl;
  model_->ComputeDescriptors();

  FeaturesMatching matcher(cfg_, scene_cloud, scene_cloud);
  FeaturesMatching::Result result = matcher.Align(0, cfg_, model_, scene_);

  std::cout << "\nReal transformation between maps:\n" << transformationMatrixToString(T);
  auto rpy_real = ToRad(rotMatrixToRPY(T.block<3,3>(0,0)));
  std::cout << "RPY: (" << rpy_real[0] << ", " << rpy_real[1] << ", " << rpy_real[2] << ")\n";

  result_transf_ = result.transformation;
  std::cout << "\n\nEstimated transformation between maps:\n" << transformationMatrixToString(result_transf_);
  auto rpy_est = ToRad(rotMatrixToRPY(result_transf_.block<3,3>(0,0)));
  std::cout << "RPY: (" << rpy_est[0] << ", " << rpy_est[1] << ", " << rpy_est[2] << ")\n";
}

