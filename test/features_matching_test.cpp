
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

class FeatureMatchingTest : public ::testing::Test {
 public:
  FeatureMatchingTest() {
    Configure();
  }

  void Configure() {
    cfg_.feature_cloud.downsampling_radius = 0.2; // Only if uniform sampling
    cfg_.feature_cloud.descriptors_radius = 1.0;
    cfg_.feature_cloud.keypoints_method = FeatureCloud::KeypointsExtractionMethod::Iss3d;
    cfg_.feature_cloud.debug = true;
    cfg_.feature_cloud.iss_num_of_threads = 6;
    cfg_.feature_cloud.iss_model_resolution = 0.02;
    cfg_.feature_cloud.iss_min_neighbours = 6;

    // Sample Consensus
    cfg_.nr_iterations = 1000;
    cfg_.min_sample_distance = 0.3;
    cfg_.max_correspondence_distance = 0.1;
    cfg_.fitness_score_dist = 0.5;

    cfg_.cell_size_x = 3;
    cfg_.cell_size_y = 3;
    cfg_.model_size_thresh_ = 400;
    cfg_.keypoints_thresh_ = 150;

    // cfg_.method = FeaturesMatching::AlignmentMethod::GeometryConsistencyClustering;
    cfg_.method = FeaturesMatching::AlignmentMethod::SampleConsensus;
    // cfg_.method = FeaturesMatching::AlignmentMethod::NewMethod;
    cfg_.visualize = true;
    cfg_.debug = true;
  }

  FeaturesMatching::Config cfg_;
};

TEST_F(FeatureMatchingTest, Test_fr) {
  auto original_tree = unpackAndGetOctomap("fr_079");
  PrintOcTreeInfo(*original_tree, "original_tree");

  auto scene = CropOcTree(*original_tree, Vector3f(-8, -8, 0.0), Vector3f(8, 8, 2.0));
  auto init_model = CropOcTree(*original_tree, Vector3f(-2, -2, 0.0), Vector3f(2, 2, 2.0));

  // Transform model
  auto T = createTransformationMatrix(20.0, 1.0, 0.0, ToRad(0), ToRad(0), ToRad(25.0));
  auto model = FastOcTreeTransform(*init_model, T);

  auto scene_cloud = OcTreeToPointCloud(*scene);
  auto model_cloud = OcTreeToPointCloud(*model);

  // Prepare scene
  auto fc_scene = std::make_shared<FeatureCloud>(scene_cloud, cfg_.feature_cloud);
  std::cout << "\nScene size: " << fc_scene->GetPointCloud()->size() << std::endl;
  fc_scene->ComputeDescriptors();

  // Prepare model
  auto fc_model = std::make_shared<FeatureCloud>(model_cloud, cfg_.feature_cloud);
  std::cout << "\nModel size: " << fc_model->GetPointCloud()->size() << std::endl;
  fc_model->ComputeDescriptors();

  FeaturesMatching matcher(cfg_, scene_cloud, scene_cloud);
  FeaturesMatching::Result result = matcher.Align(0, cfg_, fc_model, fc_scene);

  PrintMatchingResult(T, result.transformation.inverse(), result.fitness_score);

  if (result.fitness_score < 0.1) {
    std::cout << "\nOK!! \n";
  }
}

