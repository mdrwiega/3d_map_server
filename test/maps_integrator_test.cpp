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
    cfg.global_alignment_method = GlobalAlignment::Method::FeatureMatching;
    cfg.template_alignment.divide_model = true;
    cfg.template_alignment.cell_size_x = 4;
    cfg.template_alignment.cell_size_y = 4;
    cfg.template_alignment.method = FeaturesMatching::AlignmentMethodType::GeometryConsistencyClustering;

   // Feature matching
    cfg.template_alignment.model_size_thresh_ = 100;
    cfg.template_alignment.keypoints_thresh_ = 50;

    cfg.template_alignment.feature_cloud.normal_radius = 10.0;
    cfg.template_alignment.feature_cloud.downsampling_radius = 0.1;
    cfg.template_alignment.feature_cloud.descriptors_radius = 1.0;
    cfg.template_alignment.feature_cloud.keypoints_method = FeatureCloud::KeypointsExtractionMethod::Iss3d;
    cfg.template_alignment.show_visualizer = false;

    // Sample Consensus
    cfg.template_alignment.sac.min_sample_distance = 0.05;
    cfg.template_alignment.sac.max_correspondence_distance = 100.0;
    cfg.template_alignment.sac.nr_iterations = 2200;
    cfg.template_alignment.sac.fitness_score_dist = 1.0;
    cfg.template_alignment.sac.samples_num = 4;
    cfg.template_alignment.sac.nn_for_each_sample_num = 4;
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
    cfg.icp.max_nn_dist = 1.0;
    cfg.icp.fitness_eps = 0.0005;
    cfg.icp.fitness_score_dist = 0.5;
    cfg.icp.transf_eps = 0.0001;
    cfg.icp.scene_inflation_dist = 2.5;
    cfg.icp.visualize = false;
    cfg.icp.output_dir = date_and_time_ + "_";
    cfg.icp.crop_scene = true;

    // Other
    cfg.output_dir = getCurrentDateAndTime() + '_';
    cfg.show_visualizer = false;
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
  auto scene = CropOcTree(*original_tree, Vector3f(-10, -10, 0.0), Vector3f(2, 10, 2.0));
  auto init_model = CropOcTree(*original_tree, Vector3f(-2, -10, 0.0), Vector3f(15, 10, 2.0));

  // Transform model
  auto T = createTransformationMatrix(12, 6, 0.5, ToRad(5.0), ToRad(5.0), ToRad(60.0));
  auto model = FastOcTreeTransform(*init_model, T);

  MapsIntegrator maps_integrator(scene, model, cfg);
  auto res = maps_integrator.EstimateTransformation();

  DumpTestInfoToFile(original_tree, T, res.transformation);
}

TEST_F(MapsIntegratorTest, CheckOverlap) {
  auto original_tree = unpackAndGetOctomap("fr_079");
  auto scene = CropOcTree(*original_tree, Vector3f(-10, -10, 0.0), Vector3f(2, 10, 2.0));
  auto init_model = CropOcTree(*original_tree, Vector3f(-2, -10, 0.0), Vector3f(10, 10, 2.0));

  // Transform model
  auto T = createTransformationMatrix(12, 6, 0.5, ToRad(5.0), ToRad(5.0), ToRad(60.0));
  auto model = FastOcTreeTransform(*init_model, T);

  cfg.output_dir = getCurrentDateAndTime() + '_';
  MapsIntegrator maps_integrator(scene, model, cfg);
  auto res = maps_integrator.EstimateTransformation();

  DumpTestInfoToFile(original_tree, T, res.transformation);
}

TEST_F(MapsIntegratorTest, Test_fr_step1) {
  auto original_tree = unpackAndGetOctomap("fr_079");
  auto scene = CropOcTree(*original_tree, Vector3f(-10, -10, 0.0), Vector3f(2, 10, 2.0));
  auto init_model = CropOcTree(*original_tree, Vector3f(-2, -10, 0.0), Vector3f(10, 10, 2.0));

  // Transform model
  auto T = createTransformationMatrix(12, 6, 0.5, ToRad(5.0), ToRad(5.0), ToRad(60.0));
  auto model = FastOcTreeTransform(*init_model, T);

  MapsIntegrator maps_integrator(scene, model, cfg);
  auto res = maps_integrator.EstimateTransformation();

  DumpTestInfoToFile(original_tree, T, res.transformation);
}

TEST_F(MapsIntegratorTest, Test_fr_cascade_step2) {
  auto original_tree = unpackAndGetOctomap("fr_079");
  auto scene = CropOcTree(*original_tree, Vector3f(-10, -10, 0.0), Vector3f(11, 10, 2.0));
  auto init_model = CropOcTree(*original_tree, Vector3f(7, -10, 0.0), Vector3f(25, 10, 2.0));

  // Transform model
  auto T = createTransformationMatrix(12, 6, 0.5, ToRad(5.0), ToRad(5.0), ToRad(60.0));
  auto model = FastOcTreeTransform(*init_model, T);

  MapsIntegrator maps_integrator(scene, model, cfg);
  auto res = maps_integrator.EstimateTransformation();

  DumpTestInfoToFile(original_tree, T, res.transformation);
}

TEST_F(MapsIntegratorTest, Test_fr_cascade_step3) {
  auto original_tree = unpackAndGetOctomap("fr_079");
  auto scene = CropOcTree(*original_tree, Vector3f(-10, -10, 0.0), Vector3f(25, 10, 2.0));
  auto init_model = CropOcTree(*original_tree, Vector3f(21, -10, 0.0), Vector3f(40, 10, 2.0));

  // Transform model
  auto T = createTransformationMatrix(12, 6, 0.5, ToRad(5.0), ToRad(5.0), ToRad(60.0));
  auto model = FastOcTreeTransform(*init_model, T);

  MapsIntegrator maps_integrator(scene, model, cfg);
  auto res = maps_integrator.EstimateTransformation();

  DumpTestInfoToFile(original_tree, T, res.transformation);
}

TEST_F(MapsIntegratorTest, Test_fr_campus) {
  auto original_tree = unpackAndGetOctomap("fr_campus");
  auto scene = CropOcTree(*original_tree, Vector3f(-10, -10, 0.0), Vector3f(29, 10, 10.0));
  auto init_model = CropOcTree(*original_tree, Vector3f(21, -10, 0.0), Vector3f(40, 10, 10.0));

  // Transform model
  auto T = createTransformationMatrix(10, 5, 0.3, ToRad(0.0), ToRad(0.0), ToRad(10.0));
  auto model = FastOcTreeTransform(*init_model, T);

  MapsIntegrator maps_integrator(scene, model, cfg);
  auto res = maps_integrator.EstimateTransformation();
  DumpTestInfoToFile(original_tree, T, res.transformation);
 }

TEST_F(MapsIntegratorTest, Test_pwr_d20_m1) {
  auto original_tree = unpackAndGetOctomap("pwr_d20_f5_m1");
  // Limits: x(-18.55, 3.8)  y(-15.3, 12.25)  z(0.05, 1.45)
  auto scene = CropOcTree(*original_tree, Vector3f(-20, -16, 0.0), Vector3f(-6, 13, 2.0));
  auto init_model = CropOcTree(*original_tree, Vector3f(-10, -16, 0.0), Vector3f(4, 13, 2.0));

  // Transform model
  auto T = createTransformationMatrix(10.5, 5.5, 0.3, ToRad(0.0), ToRad(0.0), ToRad(5.0));
  auto model = FastOcTreeTransform(*init_model, T);

  MapsIntegrator maps_integrator(scene, model, cfg);
  auto res = maps_integrator.EstimateTransformation();
  DumpTestInfoToFile(original_tree, T, res.transformation);
}

TEST_F(MapsIntegratorTest, Test_pwr_d20_m3) {
   auto original_tree = unpackAndGetOctomap("pwr_d20_f5_m3");
  auto scene = CropOcTree(*original_tree, Vector3f(-20, -16, 0.0), Vector3f(-6, 13, 2.0));
  auto init_model = CropOcTree(*original_tree, Vector3f(-10, -16, 0.0), Vector3f(4, 13, 2.0));

  // Transform model
  auto T = createTransformationMatrix(10.5, 5.5, 0.3, ToRad(5.0), ToRad(5.0), ToRad(40.0));
  auto model = FastOcTreeTransform(*init_model, T);

  MapsIntegrator maps_integrator(scene, model, cfg);
  auto res = maps_integrator.EstimateTransformation();
  DumpTestInfoToFile(original_tree, T, res.transformation);
}

TEST_F(MapsIntegratorTest, Test_pwr_d20_m4) {
  auto original_tree = unpackAndGetOctomap("pwr_d20_f5_m4");
  auto scene = CropOcTree(*original_tree, Vector3f(-20, -16, 0.0), Vector3f(-6, 13, 2.0));
  auto init_model = CropOcTree(*original_tree, Vector3f(-10, -16, 0.0), Vector3f(4, 13, 2.0));

  // Transform model
  auto T = createTransformationMatrix(16.0, 6.0, 0.3, ToRad(5.0), ToRad(5.0), ToRad(40.0));
  auto model = FastOcTreeTransform(*init_model, T);

  MapsIntegrator maps_integrator(scene, model, cfg);
  auto res = maps_integrator.EstimateTransformation();
  DumpTestInfoToFile(original_tree, T, res.transformation);
}

TEST_F(MapsIntegratorTest, Test_pwr_d20_m3_step1)
{
  std::string octomap_name = "pwr_d20_f5_m3";
  auto T = createTransformationMatrix(14, 6, 0.5, ToRad(5), ToRad(5), ToRad(30));

  auto original_tree = unpackAndGetOctomap(octomap_name);
  PrintOcTreeInfo(*original_tree, "orig_tree");

  Vector3f map1_min = {-17.0, 12, 0};
  Vector3f map1_max = {6.0, 20, 2};

  Vector3f map2_min = {2.0, 6, 0};
  Vector3f map2_max = {20.0, 20, 2};

  auto tree_l_ = CropOcTree(*original_tree, map1_min, map1_max);
  auto tree_r_tmp = CropOcTree(*original_tree, map2_min, map2_max);
  auto tree_r_ = FastOcTreeTransform(*tree_r_tmp, transformation_);
  PrintOcTreeInfo(*tree_l_, "tree_l");
  PrintOcTreeInfo(*tree_r_, "tree_r");

  MapsIntegrator maps_integrator(tree_l_, tree_r_, cfg);
  auto res = maps_integrator.EstimateTransformation();
  DumpTestInfoToFile(original_tree, T, res.transformation);
}

TEST_F(MapsIntegratorTest, Test_pwr_d20_m3_step2)
{
  std::string octomap_name = "pwr_d20_f5_m3";
  auto T = createTransformationMatrix(18, 4, 0.5, ToRad(5), ToRad(5), ToRad(30));

  auto original_tree = unpackAndGetOctomap(octomap_name);
  PrintOcTreeInfo(*original_tree, "orig_tree");

  Vector3f map1_min = {-17.0, 6, 0};
  Vector3f map1_max = {20.0, 20, 2};

  Vector3f map2p1_min = {-18.0, -17, 0};
  Vector3f map2p1_max = {7.0, 4, 2};

  Vector3f map2p2_min = {-10.0, -17, 0};
  Vector3f map2p2_max = {0.0, 20, 2};

  auto tree2_p1 = CropOcTree(*original_tree, map2p1_min, map2p1_max);
  auto tree2_p2 = CropOcTree(*original_tree, map2p2_min, map2p2_max);
  auto tree_r_tmp = FastSumOctrees(*tree2_p1, *tree2_p2);

  auto tree_l_ = CropOcTree(*original_tree, map1_min, map1_max);
  auto tree_r_ = FastOcTreeTransform(*tree_r_tmp, transformation_);
  PrintOcTreeInfo(*tree_l_, "tree_l");
  PrintOcTreeInfo(*tree_r_, "tree_r");

  cfg.template_alignment.cell_size_x = 4.5;
  cfg.template_alignment.cell_size_y = 4.5;

  MapsIntegrator maps_integrator(tree_l_, tree_r_, cfg);
  auto res = maps_integrator.EstimateTransformation();
  DumpTestInfoToFile(original_tree, T, res.transformation);
}
