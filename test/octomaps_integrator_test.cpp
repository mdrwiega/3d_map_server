/******************************************************************************
 * Copyright (c) 2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <iostream>
#include <fstream>

#include <octomap_tools/transformations.h>
#include <octomap_tools/utils.h>
#include <octomap_tools/math.h>
#include "test_utils.h"

#include "../include/octomap_tools/conversions.h"
#include "../include/octomap_tools/maps_integrator.h"

using namespace Eigen;
using namespace octomap_tools;
using namespace octomap;
using namespace std::chrono;

class MapsIntegratorTest : public ::testing::Test
{
 public:
  MapsIntegratorTest() {
    date_and_time_ = getCurrentDateAndTime();
    configure();
  }

  ~MapsIntegratorTest() {
    std::ofstream file(date_and_time_ + "_cfg_results2.txt", std::ios_base::app);
    file << OcTreeInfoToString(*orig_tree_, "orig cloud");
    file << OcTreeInfoToString(*cropped_tree_, "cropped_cloud");
    file << OcTreeInfoToString(*tree_l_, "cloud_l");
    file << OcTreeInfoToString(*tree_r_, "cloud_r");

    file << "\nReal transformation between maps:\n" << transformationMatrixToString(transformation_);
    auto rpy_real = ToRad(rotMatrixToRPY(transformation_.block<3,3>(0,0)));
    file << "Real RPY: (" << rpy_real[0] << ", " << rpy_real[1] << ", " << rpy_real[2] << ")\n";

    file << "\n\nEstimated transformation between maps:\n" << transformationMatrixToString(result_transf_);
    auto rpy_est =ToRad(rotMatrixToRPY(result_transf_.block<3,3>(0,0)));
    file << "Real RPY: (" << rpy_est[0] << ", " << rpy_est[1] << ", " << rpy_est[2] << ")\n";

    file << "\n\nError: " << (transformation_ * result_transf_  - Eigen::Matrix4f::Identity()).norm() << "\n";
    file << "x_common: " << x_common_ << "\n";
    Eigen::Vector3f min, max;
    getMinMaxOctree(*cropped_tree_, min, max);
    file << "Common area takes: " << std::setprecision(2)
         << x_common_ / (max[0] - min[0]) * 100.0 << " \% of full map\n";
  }

  void PrepareSceneAndModelWithXDivision(
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

    Eigen::Vector3f cropped_min, cropped_max;
    getMinMaxOctree(*cropped_tree_, cropped_min, cropped_max);
    Vector3f center = (cropped_max + cropped_min) / 2;
    Vector3f tree_l_max = {center(0) + x_common_ / 2, cropped_max(1), cropped_max(2)};
    Vector3f tree_r_min = {center(0) - x_common_ / 2, cropped_min(1), cropped_min(2)};

    tree_l_ = CropOcTree(*orig_tree_, cropped_min, tree_l_max);
    auto tree_r_tmp = CropOcTree(*orig_tree_, tree_r_min, cropped_max);
    tree_r_ = FastOcTreeTransform(*tree_r_tmp, transformation_);
    PrintOcTreeInfo(*tree_l_, "tree_l");
    PrintOcTreeInfo(*tree_r_, "tree_r");
  }

  void configure() {
    // Feature matching config_uration
    config_.fitness_score_thresh = 0.002;
    config_.show_visualization_ = false;
    config_.show_keypoints_ = false;
    config_.integrate_octomaps = true;
    config_.show_integrated_octomaps = false;
    config_.show_two_pointclouds = false;
    config_.icp.max_iter = 500;
    config_.icp.max_nn_dist = 0.5;
    config_.icp.fitness_score_dist = 0.5;
    config_.icp.fitness_eps = 0.0005;
    config_.icp.transf_eps = 0.0001;
    config_.icp.scene_inflation_dist = 2.5;
    config_.icp.visualize = false;
    config_.icp.files_path_and_pattern = date_and_time_ + "_";
    config_.files_path_and_pattern = date_and_time_ + "_";

    config_.template_alignment.nr_iterations = 1000;
    config_.template_alignment.min_sample_distance = 0.2;
    config_.template_alignment.max_correspondence_distance = 100.0;
    config_.template_alignment.fitness_score_dist = 0.5;
    config_.template_alignment.feature_cloud.normal_radius = 20.0;
    config_.template_alignment.feature_cloud.downsampling_radius = 0.15;
    config_.template_alignment.feature_cloud.descriptors_radius = 1.5;
    config_.template_alignment.feature_cloud.keypoints_method = FeatureCloud::KeypointsDetectMethod::Uniform;
    config_.template_alignment.cell_size_x = 3;
    config_.template_alignment.cell_size_y = 3;
    config_.template_alignment.model_size_thresh_ = 400;
    config_.template_alignment.keypoints_thresh_ = 40;
  }

  OcTreePtr orig_tree_;
  OcTreePtr cropped_tree_;
  OcTreePtr tree_l_;
  OcTreePtr tree_r_;

  MapsIntegrator::Config config_;
  Eigen::Matrix4f transformation_;
  std::string date_and_time_;
  Eigen::Matrix4f result_transf_;
  float x_common_;
};

TEST_F(MapsIntegratorTest, Test_fr)
{
  std::string octomap_name = "fr_079";
  auto cloud_min = Vector3f(-10, -10, 0.0);
  auto cloud_max = Vector3f(10, 10, 2.0);
  transformation_ = createTransformationMatrix(12, 6, 0.5, ToRad(5), ToRad(5), ToRad(60));
  x_common_ = 4.5;
  PrepareSceneAndModelWithXDivision(octomap_name, cloud_min, cloud_max);

  config_.show_visualization_ = true;
  config_.show_two_pointclouds = true;

  MapsIntegrator features_matcher(tree_l_, tree_r_, config_);
  auto res = features_matcher.compute();
  result_transf_ = res.transformation;
}

TEST_F(MapsIntegratorTest, Test_fr_cascade_step2)
{
  std::string octomap_name = "fr_079";
  auto octomap_min = Vector3f(-10, -10, 0.0);
  auto octomap_max = Vector3f(25, 10, 2.0);
  transformation_ = createTransformationMatrix(12, 6, 0.5, ToRad(5), ToRad(5), ToRad(60));

  orig_tree_ = unpackAndGetOctomap(octomap_name);
  PrintOcTreeInfo(*orig_tree_, "orig_tree");

  if (octomap_min != Vector3f{0,0,0} && octomap_max != Vector3f{0,0,0}) {
    cropped_tree_ = CropOcTree(*orig_tree_, octomap_min, octomap_max);
    PrintOcTreeInfo(*cropped_tree_, "cropped_tree");
  } else {
    cropped_tree_ = orig_tree_;
  }

  Eigen::Vector3f cropped_min, cropped_max;
  getMinMaxOctree(*cropped_tree_, cropped_min, cropped_max);
  Vector3f tree_l_max = {10.0, cropped_max(1), cropped_max(2)};
  Vector3f tree_r_min = {8.0, cropped_min(1), cropped_min(2)};

  tree_l_ = CropOcTree(*orig_tree_, cropped_min, tree_l_max);
  auto tree_r_tmp = CropOcTree(*orig_tree_, tree_r_min, cropped_max);
  tree_r_ = FastOcTreeTransform(*tree_r_tmp, transformation_);
  PrintOcTreeInfo(*tree_l_, "tree_l");
  PrintOcTreeInfo(*tree_r_, "tree_r");

  config_.show_visualization_ = true;
  config_.show_two_pointclouds = true;

  MapsIntegrator features_matcher(tree_l_, tree_r_, config_);
  auto res = features_matcher.compute();
  result_transf_ = res.transformation;
}

TEST_F(MapsIntegratorTest, Test_fr_cascade_step3)
{
  std::string octomap_name = "fr_079";
  auto octomap_min = Vector3f(-10, -10, 0.0);
  auto octomap_max = Vector3f(40, 10, 2.0);
  transformation_ = createTransformationMatrix(12, 6, 0.5, ToRad(5), ToRad(5), ToRad(60));

  orig_tree_ = unpackAndGetOctomap(octomap_name);
  PrintOcTreeInfo(*orig_tree_, "orig_tree");

  if (octomap_min != Vector3f{0,0,0} && octomap_max != Vector3f{0,0,0}) {
    cropped_tree_ = CropOcTree(*orig_tree_, octomap_min, octomap_max);
    PrintOcTreeInfo(*cropped_tree_, "cropped_tree");
  } else {
    cropped_tree_ = orig_tree_;
  }

  Eigen::Vector3f cropped_min, cropped_max;
  getMinMaxOctree(*cropped_tree_, cropped_min, cropped_max);
  Vector3f tree_l_max = {25.0, cropped_max(1), cropped_max(2)};
  Vector3f tree_r_min = {22.0, cropped_min(1), cropped_min(2)};

  tree_l_ = CropOcTree(*orig_tree_, cropped_min, tree_l_max);
  auto tree_r_tmp = CropOcTree(*orig_tree_, tree_r_min, cropped_max);
  tree_r_ = FastOcTreeTransform(*tree_r_tmp, transformation_);
  PrintOcTreeInfo(*tree_l_, "tree_l");
  PrintOcTreeInfo(*tree_r_, "tree_r");

  config_.show_visualization_ = true;
  config_.show_two_pointclouds = true;

  MapsIntegrator features_matcher(tree_l_, tree_r_, config_);
  auto res = features_matcher.compute();
  result_transf_ = res.transformation;
}

TEST_F(MapsIntegratorTest, Test_fr_campus)
{
  std::string octomap_name = "fr_campus";
  auto cloud_min = Vector3f(-10, -10, 1.0);
  auto cloud_max = Vector3f(40, 10, 10.0);
  transformation_ = createTransformationMatrix(10, 5, 0.3, ToRad(5), ToRad(5), ToRad(10));
  x_common_ = 8;
  PrepareSceneAndModelWithXDivision(octomap_name, cloud_min, cloud_max);

  config_.show_visualization_ = true;
  config_.show_two_pointclouds = true;

  MapsIntegrator features_matcher(tree_l_, tree_r_, config_);
  auto res = features_matcher.compute();
  result_transf_ = res.transformation;
 }

TEST_F(MapsIntegratorTest, Test_pwr_d20_m1)
{
  std::string octomap_name = "pwr_d20_f5_m1";
  transformation_ = createTransformationMatrix(10.5, 5.5, 0.3, ToRad(5), ToRad(5), ToRad(65));
  x_common_ = 4;
  PrepareSceneAndModelWithXDivision(octomap_name);

  config_.show_visualization_ = true;
  config_.show_two_pointclouds = true;

  MapsIntegrator features_matcher(tree_l_, tree_r_, config_);
  features_matcher.compute();
}

TEST_F(MapsIntegratorTest, Test_pwr_d20_m3)
{
  std::string octomap_name = "pwr_d20_f5_m3";
  transformation_ = createTransformationMatrix(10.5, 5.5, 0.3, ToRad(5), ToRad(5), ToRad(65));
  x_common_ = 4;
  PrepareSceneAndModelWithXDivision(octomap_name);

  config_.show_visualization_ = true;
  config_.show_two_pointclouds = true;

  MapsIntegrator features_matcher(tree_l_, tree_r_, config_);
  features_matcher.compute();
}

TEST_F(MapsIntegratorTest, Test_pwr_d20_m4)
{
  std::string octomap_name = "pwr_d20_f5_m4";
  transformation_ = createTransformationMatrix(16, 6, 0.3, ToRad(5), ToRad(5), ToRad(90));
  x_common_ = 4;
  PrepareSceneAndModelWithXDivision(octomap_name);

  config_.show_visualization_ = true;
  config_.show_two_pointclouds = true;

  MapsIntegrator features_matcher(tree_l_, tree_r_, config_);
  features_matcher.compute();
}

TEST_F(MapsIntegratorTest, Test_pwr_d20_m3_step1)
{
  std::string octomap_name = "pwr_d20_f5_m3";
  transformation_ = createTransformationMatrix(14, 6, 0.5, ToRad(5), ToRad(5), ToRad(30));

  orig_tree_ = unpackAndGetOctomap(octomap_name);
  PrintOcTreeInfo(*orig_tree_, "orig_tree");

  Vector3f map1_min = {-17.0, 12, 0};
  Vector3f map1_max = {6.0, 20, 2};

  Vector3f map2_min = {2.0, 6, 0};
  Vector3f map2_max = {20.0, 20, 2};

  tree_l_ = CropOcTree(*orig_tree_, map1_min, map1_max);
  auto tree_r_tmp = CropOcTree(*orig_tree_, map2_min, map2_max);
  tree_r_ = FastOcTreeTransform(*tree_r_tmp, transformation_);
  PrintOcTreeInfo(*tree_l_, "tree_l");
  PrintOcTreeInfo(*tree_r_, "tree_r");

  config_.show_visualization_ = true;
  config_.show_two_pointclouds = true;

  MapsIntegrator features_matcher(tree_l_, tree_r_, config_);
  auto res = features_matcher.compute();
  result_transf_ = res.transformation;
}

TEST_F(MapsIntegratorTest, Test_pwr_d20_m3_step2)
{
  std::string octomap_name = "pwr_d20_f5_m3";
  transformation_ = createTransformationMatrix(18, 4, 0.5, ToRad(5), ToRad(5), ToRad(30));

  orig_tree_ = unpackAndGetOctomap(octomap_name);
  PrintOcTreeInfo(*orig_tree_, "orig_tree");

  Vector3f map1_min = {-17.0, 6, 0};
  Vector3f map1_max = {20.0, 20, 2};

  Vector3f map2p1_min = {-18.0, -17, 0};
  Vector3f map2p1_max = {7.0, 4, 2};

  Vector3f map2p2_min = {-10.0, -17, 0};
  Vector3f map2p2_max = {0.0, 20, 2};

  auto tree2_p1 = CropOcTree(*orig_tree_, map2p1_min, map2p1_max);
  auto tree2_p2 = CropOcTree(*orig_tree_, map2p2_min, map2p2_max);
  auto tree_r_tmp = FastSumOctrees(*tree2_p1, *tree2_p2);

  tree_l_ = CropOcTree(*orig_tree_, map1_min, map1_max);
  tree_r_ = FastOcTreeTransform(*tree_r_tmp, transformation_);
  PrintOcTreeInfo(*tree_l_, "tree_l");
  PrintOcTreeInfo(*tree_r_, "tree_r");

  config_.template_alignment.cell_size_x = 4.5;
  config_.template_alignment.cell_size_y = 4.5;
  config_.show_visualization_ = true;
  config_.show_two_pointclouds = true;

  MapsIntegrator features_matcher(tree_l_, tree_r_, config_);
  auto res = features_matcher.compute();
  result_transf_ = res.transformation;
}


TEST_F(MapsIntegratorTest, Test_pwr_d20_m4_t2)
{
  std::string octomap_name = "pwr_d20_f5_m4";
  transformation_ = createTransformationMatrix(20, 6, 0.3, ToRad(5), ToRad(5), ToRad(15));
  x_common_ = 5;
  PrepareSceneAndModelWithXDivision(octomap_name);

  config_.show_visualization_ = true;
  config_.show_two_pointclouds = true;

  MapsIntegrator features_matcher(tree_l_, tree_r_, config_);
  features_matcher.compute();
}

/*TEST_F(MapsIntegratorTest, Test_multi_fr)
{
  std::string octomap_name = "fr_079";
  auto cloud_min = Vector4f(-10, -10, -0.5, 1.0);
  auto cloud_max = Vector4f(7, 7, 2.0, 1.0);
  transformation_ = createTransformationMatrix(8.5, 1.5, 0.1, ToRadians(3), ToRadians(2), ToRadians(25));
  x_common_ = 4;
  PrepareSceneAndModelWithXDivision(octomap_name, cloud_min, cloud_max);

  std::vector<float> max_correspondence_distance_vec = {0.5, 1.0, 2.0, 5.0, 10.0, 20.0};
  std::vector<float> min_sample_distance_vec = {0.05, 0.1, 0.2, 0.5, 1.0};
  std::vector<float> descriptors_radius_vec = {0.5, 1.5, 2.0, 3.0};
  std::vector<float> downsampling_radius_vec = {0.1, 0.15, 0.2, 0.3};
  std::vector<float> normal_radius_vec = {2.0, 5.0, 10.0, 20.0};
  std::vector<float> cell_size_vec = {1, 2, 2.5, 3};
  std::vector<float> icp_max_nn_dist_vec = {0.1, 0.2, 0.5, 0.};

  for (auto icp_max_nn_dist : icp_max_nn_dist_vec) {
    config_.icp.max_nn_dist = icp_max_nn_dist;
    MapsIntegrator features_matcher(config_);
    features_matcher.computeWithModelDivision(cloud_l, cloud_r);
  }

  configure();
  for (auto cell_size : cell_size_vec) {
    config_.cell_size_x_ = cell_size;
    config_.cell_size_y_ = cell_size;
    MapsIntegrator features_matcher(config_);
    features_matcher.computeWithModelDivision(cloud_l, cloud_r);
  }

  configure();
  for (auto normal_radius : normal_radius_vec) {
    config_.feature_cloud.normal_radius = normal_radius;
    MapsIntegrator features_matcher(config_);
    features_matcher.computeWithModelDivision(cloud_l, cloud_r);
  }

  configure();
  for (auto downsampling_radius : downsampling_radius_vec) {
    config_.feature_cloud.downsampling_radius = downsampling_radius;
    MapsIntegrator features_matcher(config_);
    features_matcher.computeWithModelDivision(cloud_l, cloud_r);
  }

  configure();
  for (auto descriptors_radius : descriptors_radius_vec) {
    config_.feature_cloud.descriptors_radius = descriptors_radius;
    MapsIntegrator features_matcher(config_);
    features_matcher.computeWithModelDivision(cloud_l, cloud_r);
  }

  configure();
  for (auto min_sample_distance : min_sample_distance_vec) {
    config_.template_alignment.min_sample_distance = min_sample_distance;
    MapsIntegrator features_matcher(config_);
    features_matcher.computeWithModelDivision(cloud_l, cloud_r);
  }

  configure();
  for (auto max_correspondence_distance : max_correspondence_distance_vec) {
    config_.template_alignment.max_correspondence_distance = max_correspondence_distance;
    MapsIntegrator features_matcher(config_);
    features_matcher.computeWithModelDivision(cloud_l, cloud_r);
  }

//  for (auto max_correspondence_distance : max_correspondence_distance_vec) {
//    config_.template_alignment.max_correspondence_distance = max_correspondence_distance;
//    for (auto min_sample_distance : min_sample_distance_vec) {
//      config_.template_alignment.min_sample_distance = min_sample_distance;
//      for (auto descriptors_radius : descriptors_radius_vec) {
//        config_.feature_cloud.descriptors_radius = descriptors_radius;
//        for (auto downsampling_radius : downsampling_radius_vec) {
//          config_.feature_cloud.downsampling_radius = downsampling_radius;
//          for (auto normal_radius : normal_radius_vec) {
//            config_.feature_cloud.normal_radius = normal_radius;
//            for (auto cell_size : cell_size_vec) {
//              config_.cell_size_x_ = cell_size;
//              config_.cell_size_y_ = cell_size;
//              for (auto icp_max_nn_dist : icp_max_nn_dist_vec) {
//                config_.icp.max_nn_dist = icp_max_nn_dist_vec;
//
//                MapsIntegrator features_matcher(config_);
//                features_matcher.computeWithModelDivision(cloud_l, cloud_r);
//              }
//            }
//          }
//        }
//      }
//    }
//  }
}*/

// TEST(SpiralTest, GenerateSpiralTraverse)
// {
//   Eigen::Vector2f rectangle_min (-4, -2);
//   Eigen::Vector2f rectangle_max (2, 3);
//   Eigen::Vector2f step_xy (1, 2);

//   auto cells_seq = generateBlocksInSpiralOrder(rectangle_min, rectangle_max, step_xy);

//   for (size_t i = 0; i < cells_seq.size(); ++i) {
//     auto cell = cells_seq[i];
//     std::cout << "Block nr: " << i << "  min: (" << cell.min(0) << ", " << cell.min(1) << ")  max: (" << cell.max(0) << ", " << cell.max(1) << ")\n";
//   }
// }

// int main(int argc, char **argv)
// {
//   ::testing::InitGoogleTest(&argc, argv);
//   return RUN_ALL_TESTS();
// }

