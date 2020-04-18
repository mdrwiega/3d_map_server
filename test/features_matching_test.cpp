/******************************************************************************
 * Copyright (c) 2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <octomap_tools/transformations.h>
#include <octomap_tools/utils.h>
#include <octomap_tools/math.h>
#include "test_utils.h"
#include <octomap_tools/feature_cloud.h>
#include <octomap_tools/features_matching.h>
#include <octomap_tools/conversions.h>

using namespace octomap_tools;
using namespace octomap;
using namespace std::chrono;

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

  void ShowPointCloudAndKeypoints(const Eigen::Matrix4f& T) {
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addCoordinateSystem(1.0);

    // Scene --> BLUE
    auto scene = scene_->GetPointCloud();
    pcl::visualization::PointCloudColorHandlerCustom<Point> scene_color(scene, 0, 0, 255);
    viewer.addPointCloud(scene, scene_color, "scene");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene");

    // Model --> GREEN
    auto model  = model_->GetPointCloud();
    pcl::visualization::PointCloudColorHandlerCustom<Point> model_color(model, 0, 255, 0);
    viewer.addPointCloud(model, model_color, "model");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "model");


    // Scene Keypoints --> RED
    auto keypoints = scene_->GetKeypoints();
    pcl::visualization::PointCloudColorHandlerCustom<Point> keypoints_color(keypoints, 255, 0, 0);
    viewer.addPointCloud(keypoints, keypoints_color, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "keypoints");

    // Model Keypoints --> ORANGE
    auto model_keypoints = model_->GetKeypoints();
    pcl::visualization::PointCloudColorHandlerCustom<Point> model_keypoints_color(model_keypoints, 255, 160, 0);
    viewer.addPointCloud(model_keypoints, model_keypoints_color, "model_keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "model_keypoints");

    // Model after transformation --> YELLOW
    PointCloud::Ptr transformed_model(new PointCloud());
    pcl::transformPointCloud(*model, *transformed_model, T);
    pcl::visualization::PointCloudColorHandlerCustom<Point> transformed_model_color_handler(transformed_model, 255, 255, 0);
    viewer.addPointCloud(transformed_model, transformed_model_color_handler, "transformed_model");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_model");

    while (!viewer.wasStopped()) {
      viewer.spinOnce(100);
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
  auto cloud_min = Vector3f(-5, -5, 0.0);
  auto cloud_max = Vector3f(5, 5, 2.0);
  PrepareOcTree(octomap_name, cloud_min, cloud_max);

  auto T = createTransformationMatrix(15.0, 0.5, 0.0, ToRad(0), ToRad(0), ToRad(15));
  auto tree_model = FastOcTreeTransform(*cropped_tree_, T);

  auto scene_cloud = OcTreeToPointCloud(*cropped_tree_);
  auto model_cloud = OcTreeToPointCloud(*tree_model);

  // Prepare scene
  scene_ = std::make_shared<FeatureCloud>(scene_cloud, cfg_.feature_cloud);
  std::cout << "\nScene size: " << scene_->GetPointCloud()->size() << std::endl;
  scene_->ExtractKeypoints();
  scene_->ComputeSurfaceNormals();
  scene_->ComputeDescriptors();

  // Prepare model
  model_ = std::make_shared<FeatureCloud>(model_cloud, cfg_.feature_cloud);
  std::cout << "\nModel size: " << model_->GetPointCloud()->size() << std::endl;
  model_->ExtractKeypoints();
  model_->ComputeSurfaceNormals();
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

  ShowPointCloudAndKeypoints(result.transformation);
}

