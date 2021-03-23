#include <gtest/gtest.h>
#include <iostream>
#include <fstream>

#include <transformations.h>
#include <utils.h>
#include <math.h>
#include "test_utils.h"

#include <conversions.h>
#include <octomap_tools/maps_integrator.h>

using namespace Eigen;
using namespace octomap_tools;
using namespace octomap;
using namespace std::chrono;

class MapsIntegratorTest : public ::testing::Test
{
 public:
  MapsIntegratorTest() {
    date_and_time_ = getCurrentDateAndTime();
    Configure();
  }

  ~MapsIntegratorTest() {
  }

  void Configure() {
    cfg.template_alignment.divide_model = false;
    cfg.template_alignment.cell_size_x = 3;
    cfg.template_alignment.cell_size_y = 3;
    cfg.template_alignment.method = FeaturesMatching::AlignmentMethodType::SampleConsensus;

   // Feature matching
    cfg.template_alignment.model_size_thresh_ = 400;
    cfg.template_alignment.keypoints_thresh_ = 40;

    cfg.template_alignment.feature_cloud.normal_radius = 10.0;
    cfg.template_alignment.feature_cloud.downsampling_radius = 0.15;
    cfg.template_alignment.feature_cloud.descriptors_radius = 1.0;
    cfg.template_alignment.feature_cloud.keypoints_method = FeatureCloud::KeypointsExtractionMethod::Iss3d;
    cfg.template_alignment.feature_cloud.debug = true;

    // Sample Consensus
    cfg.template_alignment.sac.min_sample_distance = 0.2;
    cfg.template_alignment.sac.max_correspondence_distance = 100.0;
    cfg.template_alignment.sac.nr_iterations = 800;
    cfg.template_alignment.sac.fitness_score_dist = 1.0;
    cfg.template_alignment.sac.samples_num = 5;
    cfg.template_alignment.sac.nn_for_each_sample_num = 10;
    cfg.template_alignment.sac.modified_version = false;
    cfg.template_alignment.sac.mod_feature_max_dist = 0.7;
    cfg.template_alignment.sac.mod_feature_max_dist_diff = 0.1;

    // ISS 3D
    cfg.template_alignment.feature_cloud.iss_salient_radius = 0.12;
    cfg.template_alignment.feature_cloud.iss_non_max_radius = 0.08;
    cfg.template_alignment.feature_cloud.iss_threshold21 = 0.975;
    cfg.template_alignment.feature_cloud.iss_threshold32 = 0.975;
    cfg.template_alignment.feature_cloud.iss_min_neighbours = 6;
    cfg.template_alignment.feature_cloud.iss_num_of_threads = 2;

    // ICP
    cfg.icp_correction = true;
    cfg.icp.max_iter = 500;
    cfg.icp.max_nn_dist = 0.5;
    cfg.icp.fitness_eps = 0.0005;
    cfg.icp.fitness_score_dist = 0.5;
    cfg.icp.transf_eps = 0.0001;
    cfg.icp.scene_inflation_dist = 2.5;
    cfg.icp.visualize = false;
    cfg.icp.output_dir = date_and_time_ + "_";
    cfg.icp.crop_scene = true;

    // Other
  }

  void DumpTestInfoToFile(const OcTreePtr original_tree,
    const Eigen::Matrix4f& real_transf, const Eigen::Matrix4f& est_transf) {

    std::string file_path = cfg.output_dir + "_result.txt";
    std::ofstream file(file_path, std::ios_base::app);

    file << "real_transformation:\n" << transfMatrixToXyzRpyString(real_transf, "  ");
    file << "real_error: " << transformationsError(real_transf, est_transf) << "\n\n";
    file << OcTreeInfoToString(*original_tree, "original_map");
  }

  MapsIntegrator::Config cfg;
  Eigen::Matrix4f transformation_;
  std::string date_and_time_;
  Eigen::Matrix4f result_transf_;
  float x_common_;
};

TEST_F(MapsIntegratorTest, Test_fr) {
  auto original_tree = unpackAndGetOctomap("fr_079");
  PrintOcTreeInfo(*original_tree, "original_tree");

  auto scene = CropOcTree(*original_tree, Vector3f(-10, -10, 0.0), Vector3f(2, 10, 2.0));
  auto init_model = CropOcTree(*original_tree, Vector3f(-2, -10, 0.0), Vector3f(15, 10, 2.0));

  // Transform model
  auto T = createTransformationMatrix(12, 6, 0.5, ToRad(5.0), ToRad(5.0), ToRad(60.0));
  auto model = FastOcTreeTransform(*init_model, T);

  cfg.output_dir = getCurrentDateAndTime() + '_';
  MapsIntegrator maps_integrator(scene, model, cfg);
  auto res = maps_integrator.EstimateTransformation();

  DumpTestInfoToFile(original_tree, T, res.transformation);
}

// TEST_F(MapsIntegratorTest, CheckOverlap) {
//   auto original_tree = unpackAndGetOctomap("fr_079");
//   PrintOcTreeInfo(*original_tree, "original_tree");

//   auto scene = CropOcTree(*original_tree, Vector3f(-10, -10, 0.0), Vector3f(2, 10, 2.0));
//   auto init_model = CropOcTree(*original_tree, Vector3f(-2, -10, 0.0), Vector3f(10, 10, 2.0));

//   // Transform model
//   auto T = createTransformationMatrix(12, 6, 0.5, ToRad(5.0), ToRad(5.0), ToRad(60.0));
//   auto model = FastOcTreeTransform(*init_model, T);


//   // SaveOcTreeToFile(*scene, "fr_scene.ot");
//   // SaveOcTreeToFile(*model, "fr_model.ot");

//   cfg.output_dir = getCurrentDateAndTime() + '_';
//   MapsIntegrator maps_integrator(scene, model, cfg);
//   auto res = maps_integrator.EstimateTransformation();

//   // maps_integrator.DumpConfigAndResultsToFile();
//   // DumpTestInfoToFile(original_tree, scene, model, T, res.transformation, res.fitness_score);
// }

// TEST_F(MapsIntegratorTest, Test_fr) {
//   auto original_tree = unpackAndGetOctomap("fr_079");
//   PrintOcTreeInfo(*original_tree, "original_tree");

//   auto scene = CropOcTree(*original_tree, Vector3f(-10, -10, 0.0), Vector3f(2, 10, 2.0));
//   auto init_model = CropOcTree(*original_tree, Vector3f(-2, -10, 0.0), Vector3f(10, 10, 2.0));

//   // Transform model
//   auto T = createTransformationMatrix(12, 6, 0.5, ToRad(5.0), ToRad(5.0), ToRad(60.0));
//   auto model = FastOcTreeTransform(*init_model, T);


//   // SaveOcTreeToFile(*scene, "fr_scene.ot");
//   // SaveOcTreeToFile(*model, "fr_model.ot");

//   cfg.output_dir = getCurrentDateAndTime() + '_';
//   MapsIntegrator maps_integrator(scene, model, cfg);
//   auto res = maps_integrator.EstimateTransformation();

//   // maps_integrator.DumpConfigAndResultsToFile();
//   // DumpTestInfoToFile(original_tree, scene, model, T, res.transformation, res.fitness_score);
// }

// TEST_F(MapsIntegratorTest, Test_fr_cascade_step2) {
//   auto original_tree = unpackAndGetOctomap("fr_079");

//   auto scene = CropOcTree(*original_tree, Vector3f(-10, -10, 0.0), Vector3f(11, 10, 2.0));
//   auto init_model = CropOcTree(*original_tree, Vector3f(7, -10, 0.0), Vector3f(25, 10, 2.0));

//   // Transform model
//   auto T = createTransformationMatrix(12, 6, 0.5, ToRad(5.0), ToRad(5.0), ToRad(60.0));
//   auto model = FastOcTreeTransform(*init_model, T);

//   MapsIntegrator maps_integrator(scene, model, cfg);
//   auto res = maps_integrator.EstimateTransformation();
//   result_transf_ = res.transformation;
// }

// TEST_F(MapsIntegratorTest, Test_fr_cascade_step3) {
//   auto original_tree = unpackAndGetOctomap("fr_079");

//   auto scene = CropOcTree(*original_tree, Vector3f(-10, -10, 0.0), Vector3f(25, 10, 2.0));
//   auto init_model = CropOcTree(*original_tree, Vector3f(21, -10, 0.0), Vector3f(40, 10, 2.0));

//   // Transform model
//   auto T = createTransformationMatrix(12, 6, 0.5, ToRad(5.0), ToRad(5.0), ToRad(60.0));
//   auto model = FastOcTreeTransform(*init_model, T);

//   MapsIntegrator maps_integrator(scene, model, cfg);
//   auto res = maps_integrator.EstimateTransformation();
//   result_transf_ = res.transformation;
// }

// TEST_F(MapsIntegratorTest, Test_fr_campus) {
//   auto original_tree = unpackAndGetOctomap("fr_campus");
//   PrintOcTreeInfo(*original_tree, "original_tree");

//   auto scene = CropOcTree(*original_tree, Vector3f(-10, -10, 0.0), Vector3f(29, 10, 10.0));
//   auto init_model = CropOcTree(*original_tree, Vector3f(21, -10, 0.0), Vector3f(40, 10, 10.0));

//   // Transform model
//   auto T = createTransformationMatrix(10, 5, 0.3, ToRad(0.0), ToRad(0.0), ToRad(10.0));
//   auto model = FastOcTreeTransform(*init_model, T);

//   MapsIntegrator maps_integrator(scene, model, cfg);
//   auto res = maps_integrator.EstimateTransformation();
//   result_transf_ = res.transformation;
//  }

// TEST_F(MapsIntegratorTest, Test_pwr_d20_m1) {
//   auto original_tree = unpackAndGetOctomap("pwr_d20_f5_m1");
//   // Limits: x(-18.55, 3.8)  y(-15.3, 12.25)  z(0.05, 1.45)

//   auto scene = CropOcTree(*original_tree, Vector3f(-20, -16, 0.0), Vector3f(-6, 13, 2.0));
//   auto init_model = CropOcTree(*original_tree, Vector3f(-10, -16, 0.0), Vector3f(4, 13, 2.0));

//   // Transform model
//   auto T = createTransformationMatrix(10.5, 5.5, 0.3, ToRad(0.0), ToRad(0.0), ToRad(5.0));
//   auto model = FastOcTreeTransform(*init_model, T);

//   MapsIntegrator maps_integrator(scene, model, cfg);
//   auto res = maps_integrator.EstimateTransformation();
//   result_transf_ = res.transformation;
// }

// TEST_F(MapsIntegratorTest, Test_pwr_d20_m3) {
//    auto original_tree = unpackAndGetOctomap("pwr_d20_f5_m3");

//   auto scene = CropOcTree(*original_tree, Vector3f(-20, -16, 0.0), Vector3f(-6, 13, 2.0));
//   auto init_model = CropOcTree(*original_tree, Vector3f(-10, -16, 0.0), Vector3f(4, 13, 2.0));

//   // Transform model
//   auto T = createTransformationMatrix(10.5, 5.5, 0.3, ToRad(5.0), ToRad(5.0), ToRad(40.0));
//   auto model = FastOcTreeTransform(*init_model, T);

//   MapsIntegrator maps_integrator(scene, model, cfg);
//   maps_integrator.EstimateTransformation();
// }

// TEST_F(MapsIntegratorTest, Test_pwr_d20_m4) {
//   auto original_tree = unpackAndGetOctomap("pwr_d20_f5_m4");

//   auto scene = CropOcTree(*original_tree, Vector3f(-20, -16, 0.0), Vector3f(-6, 13, 2.0));
//   auto init_model = CropOcTree(*original_tree, Vector3f(-10, -16, 0.0), Vector3f(4, 13, 2.0));

//   // Transform model
//   auto T = createTransformationMatrix(16.0, 6.0, 0.3, ToRad(5.0), ToRad(5.0), ToRad(40.0));
//   auto model = FastOcTreeTransform(*init_model, T);

//   MapsIntegrator maps_integrator(scene, model, cfg);
//   maps_integrator.EstimateTransformation();
// }

// TEST_F(MapsIntegratorTest, Test_pwr_d20_m3_step1)
// {
//   std::string octomap_name = "pwr_d20_f5_m3";
//   transformation_ = createTransformationMatrix(14, 6, 0.5, ToRad(5), ToRad(5), ToRad(30));

//   orig_tree_ = unpackAndGetOctomap(octomap_name);
//   PrintOcTreeInfo(*orig_tree_, "orig_tree");

//   Vector3f map1_min = {-17.0, 12, 0};
//   Vector3f map1_max = {6.0, 20, 2};

//   Vector3f map2_min = {2.0, 6, 0};
//   Vector3f map2_max = {20.0, 20, 2};

//   tree_l_ = CropOcTree(*orig_tree_, map1_min, map1_max);
//   auto tree_r_tmp = CropOcTree(*orig_tree_, map2_min, map2_max);
//   tree_r_ = FastOcTreeTransform(*tree_r_tmp, transformation_);
//   PrintOcTreeInfo(*tree_l_, "tree_l");
//   PrintOcTreeInfo(*tree_r_, "tree_r");

//   cfg.show_visualization_ = true;
//   cfg.show_two_pointclouds = true;

//   MapsIntegrator maps_integrator(tree_l_, tree_r_, cfg);
//   auto res = maps_integrator.EstimateTransformation();
//   result_transf_ = res.transformation;
// }

// TEST_F(MapsIntegratorTest, Test_pwr_d20_m3_step2)
// {
//   std::string octomap_name = "pwr_d20_f5_m3";
//   transformation_ = createTransformationMatrix(18, 4, 0.5, ToRad(5), ToRad(5), ToRad(30));

//   orig_tree_ = unpackAndGetOctomap(octomap_name);
//   PrintOcTreeInfo(*orig_tree_, "orig_tree");

//   Vector3f map1_min = {-17.0, 6, 0};
//   Vector3f map1_max = {20.0, 20, 2};

//   Vector3f map2p1_min = {-18.0, -17, 0};
//   Vector3f map2p1_max = {7.0, 4, 2};

//   Vector3f map2p2_min = {-10.0, -17, 0};
//   Vector3f map2p2_max = {0.0, 20, 2};

//   auto tree2_p1 = CropOcTree(*orig_tree_, map2p1_min, map2p1_max);
//   auto tree2_p2 = CropOcTree(*orig_tree_, map2p2_min, map2p2_max);
//   auto tree_r_tmp = FastSumOctrees(*tree2_p1, *tree2_p2);

//   tree_l_ = CropOcTree(*orig_tree_, map1_min, map1_max);
//   tree_r_ = FastOcTreeTransform(*tree_r_tmp, transformation_);
//   PrintOcTreeInfo(*tree_l_, "tree_l");
//   PrintOcTreeInfo(*tree_r_, "tree_r");

//   cfg.template_alignment.cell_size_x = 4.5;
//   cfg.template_alignment.cell_size_y = 4.5;
//   cfg.show_visualization_ = true;
//   cfg.show_two_pointclouds = true;

//   MapsIntegrator maps_integrator(tree_l_, tree_r_, cfg);
//   auto res = maps_integrator.EstimateTransformation();
//   result_transf_ = res.transformation;
// }


// TEST_F(MapsIntegratorTest, Test_pwr_d20_m4_t2)
// {
//   std::string octomap_name = "pwr_d20_f5_m4";
//   transformation_ = createTransformationMatrix(20, 6, 0.3, ToRad(5), ToRad(5), ToRad(15));
//   x_common_ = 5;
//   PrepareSceneAndModelWithXDivision(octomap_name);

//   cfg.show_visualization_ = true;
//   cfg.show_two_pointclouds = true;

//   MapsIntegrator maps_integrator(tree_l_, tree_r_, cfg);
//   maps_integrator.EstimateTransformation();
// }

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
    cfg.icp.max_nn_dist = icp_max_nn_dist;
    MapsIntegrator maps_integrator(cfg);
    maps_integrator.EstimateTransformationWithModelDivision(cloud_l, cloud_r);
  }

  configure();
  for (auto cell_size : cell_size_vec) {
    cfg.cell_size_x_ = cell_size;
    cfg.cell_size_y_ = cell_size;
    MapsIntegrator maps_integrator(cfg);
    maps_integrator.EstimateTransformationWithModelDivision(cloud_l, cloud_r);
  }

  configure();
  for (auto normal_radius : normal_radius_vec) {
    cfg.feature_cloud.normal_radius = normal_radius;
    MapsIntegrator maps_integrator(cfg);
    maps_integrator.EstimateTransformationWithModelDivision(cloud_l, cloud_r);
  }

  configure();
  for (auto downsampling_radius : downsampling_radius_vec) {
    cfg.feature_cloud.downsampling_radius = downsampling_radius;
    MapsIntegrator maps_integrator(cfg);
    maps_integrator.EstimateTransformationWithModelDivision(cloud_l, cloud_r);
  }

  configure();
  for (auto descriptors_radius : descriptors_radius_vec) {
    cfg.feature_cloud.descriptors_radius = descriptors_radius;
    MapsIntegrator maps_integrator(cfg);
    maps_integrator.EstimateTransformationWithModelDivision(cloud_l, cloud_r);
  }

  configure();
  for (auto min_sample_distance : min_sample_distance_vec) {
    cfg.template_alignment.min_sample_distance = min_sample_distance;
    MapsIntegrator maps_integrator(cfg);
    maps_integrator.EstimateTransformationWithModelDivision(cloud_l, cloud_r);
  }

  configure();
  for (auto max_correspondence_distance : max_correspondence_distance_vec) {
    cfg.template_alignment.max_correspondence_distance = max_correspondence_distance;
    MapsIntegrator maps_integrator(cfg);
    maps_integrator.EstimateTransformationWithModelDivision(cloud_l, cloud_r);
  }

//  for (auto max_correspondence_distance : max_correspondence_distance_vec) {
//    cfg.template_alignment.max_correspondence_distance = max_correspondence_distance;
//    for (auto min_sample_distance : min_sample_distance_vec) {
//      cfg.template_alignment.min_sample_distance = min_sample_distance;
//      for (auto descriptors_radius : descriptors_radius_vec) {
//        cfg.feature_cloud.descriptors_radius = descriptors_radius;
//        for (auto downsampling_radius : downsampling_radius_vec) {
//          cfg.feature_cloud.downsampling_radius = downsampling_radius;
//          for (auto normal_radius : normal_radius_vec) {
//            cfg.feature_cloud.normal_radius = normal_radius;
//            for (auto cell_size : cell_size_vec) {
//              cfg.cell_size_x_ = cell_size;
//              cfg.cell_size_y_ = cell_size;
//              for (auto icp_max_nn_dist : icp_max_nn_dist_vec) {
//                cfg.icp.max_nn_dist = icp_max_nn_dist_vec;
//
//                MapsIntegrator maps_integrator(cfg);
//                maps_integrator.EstimateTransformationWithModelDivision(cloud_l, cloud_r);
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
