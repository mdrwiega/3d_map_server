
#include <gtest/gtest.h>
#include <iostream>

#include <pcl/point_cloud.h>

#include <transformations.h>
#include <utils.h>
#include <math.h>
#include <features_matching.h>
#include <conversions.h>

#include "test_utils.h"

using namespace octomap_tools;
using namespace octomap;

class FeatureMatchingTest : public ::testing::Test {
 public:
  FeatureMatchingTest() {
    Configure();
    pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
  }

  void Configure() {
    cfg.feature_cloud.normal_radius = 10.0;
    cfg.feature_cloud.downsampling_radius = 0.15; // Only if uniform sampling
    cfg.feature_cloud.descriptors_radius = 1.0;
    cfg.feature_cloud.keypoints_method = FeatureCloud::KeypointsExtractionMethod::Iss3d;
    cfg.feature_cloud.debug = true;

    // ISS 3D
    cfg.feature_cloud.iss_salient_radius = 0.12;
    cfg.feature_cloud.iss_non_max_radius = 0.08;
    cfg.feature_cloud.iss_threshold21 = 0.975;
    cfg.feature_cloud.iss_threshold32 = 0.975;
    cfg.feature_cloud.iss_min_neighbours = 6;
    cfg.feature_cloud.iss_num_of_threads = 2;

    // KDTS
    cfg.kdts.desc_dist_thresh = 0.25;

    // Sample Consensus
    cfg.sac.min_sample_distance = 0.2;
    cfg.sac.max_correspondence_distance = 100.0;
    cfg.sac.nr_iterations = 500;
    cfg.sac.fitness_score_dist = 1.0;
    cfg.sac.samples_num = 5;
    cfg.sac.nn_for_each_sample_num = 10;
    cfg.sac.modified_version = true;
    cfg.sac.mod_feature_max_dist = 0.7;
    cfg.sac.mod_feature_max_dist_diff = 0.1;

    cfg.cell_size_x = 3;
    cfg.cell_size_y = 3;
    cfg.model_size_thresh_ = 400;
    cfg.keypoints_thresh_ = 150;
    cfg.divide_model = true;

    cfg.method = FeaturesMatching::AlignmentMethodType::SampleConsensus;

    cfg.show_visualizer = false;
  }

  FeaturesMatching::Config cfg;
};

TEST_F(FeatureMatchingTest, Test_fr) {
  auto original_tree = unpackAndGetOctomap("fr_079");
  PrintOcTreeInfo(*original_tree, "original_tree");

  auto scene = CropOcTree(*original_tree, Vector3f(-10, -10, 0.0), Vector3f(2, 10, 2.0));
  auto init_model = CropOcTree(*original_tree, Vector3f(-2, -10, 0.0), Vector3f(2.5, 10, 2.0));

  // Transform model
  auto T = createTransformationMatrix(12, 6, 0.5, ToRad(5.0), ToRad(5.0), ToRad(60.0));
  auto model = FastOcTreeTransform(*init_model, T);

  auto scene_cloud = OcTreeToPointCloud(*scene);
  auto model_cloud = OcTreeToPointCloud(*model);

  // Prepare scene
  auto fc_scene = std::make_shared<FeatureCloud>(scene_cloud, cfg.feature_cloud);
  std::cout << "\nScene size: " << fc_scene->GetPointCloud()->size() << std::endl;
  fc_scene->ComputeDescriptors();

  // Prepare model
  auto fc_model = std::make_shared<FeatureCloud>(model_cloud, cfg.feature_cloud);
  std::cout << "\nModel size: " << fc_model->GetPointCloud()->size() << std::endl;
  fc_model->ComputeDescriptors();

  // Standard SAC
  FeaturesMatching matcher(cfg, scene_cloud, model_cloud);
  FeaturesMatching::Result result = matcher.Align(0, cfg, fc_model, fc_scene);

  // Modified SAC
  cfg.sac.modified_version = false;
  FeaturesMatching matcher2(cfg, scene_cloud, model_cloud);
  FeaturesMatching::Result result1 = matcher2.Align(0, cfg, fc_model, fc_scene);

  PrintMatchingResult(T, result.transformation, result.fitness_score);

  PrintMatchingResult(T, result1.transformation, result1.fitness_score);


  if (result.fitness_score < 0.1) {
    std::cout << "\nOK!! \n";
  }
}

