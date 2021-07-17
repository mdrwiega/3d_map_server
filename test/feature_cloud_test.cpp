/******************************************************************************
 * Copyright (c) 2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <gtest/gtest.h>

#include <iostream>
#include <chrono>
#include <thread>

#include <pcl/visualization/pcl_visualizer.h>

#include <common/utils.h>
#include <alignment/feature_matching/feature_cloud.h>
#include <common/conversions.h>
#include "test_utils.h"

using namespace octomap_tools;
using namespace octomap;
using namespace std::chrono;

class FeatureCloudTest : public ::testing::Test {
 public:
  FeatureCloudTest() {
    Configure();
  }

  void Configure() {
    cfg_.downsampling_radius = 0.15;
    cfg_.descriptors_radius = 0.5;
    cfg_.keypoints_method = FeatureCloud::KeypointsExtractionMethod::Iss3d;
    cfg_.iss_min_neighbours = 10;
    cfg_.iss_num_of_threads = 8;
    cfg_.iss_threshold21 = 0.975;
    cfg_.iss_threshold32 = 0.975;
  }

  void PrepareOcTree(const std::string& octomap_packed_file,
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

  // Open a PCL Visualizer window and show extracted keypoints
  void ShowPointCloudAndKeypoints() {
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addCoordinateSystem(1.0);

    // Cloud --> BLUE
    auto cloud = feature_cloud_->GetPointCloud();
    pcl::visualization::PointCloudColorHandlerCustom<Point> cloud_color(cloud, 0, 0, 255);
    viewer.addPointCloud(cloud, cloud_color, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    // Keypoints --> RED
    auto keypoints = feature_cloud_->GetKeypoints();
    pcl::visualization::PointCloudColorHandlerCustom<Point> keypoints_color(keypoints, 255, 0, 0);
    viewer.addPointCloud(keypoints, keypoints_color, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "keypoints");

    while (!viewer.wasStopped()) {
      viewer.spinOnce(100);
    }
  }

  OcTreePtr orig_tree_;
  OcTreePtr cropped_tree_;
  FeatureCloud::Config cfg_;
  std::shared_ptr<FeatureCloud> feature_cloud_;
  Eigen::Matrix4f result_transf_;
};

TEST_F(FeatureCloudTest, Test_fr) {
  std::string octomap_name = "fr_079";
  auto cloud_min = Vector3f(-5, -5, 0.0);
  auto cloud_max = Vector3f(5, 5, 2.0);
  PrepareOcTree(octomap_name, cloud_min, cloud_max);

  auto cloud = OcTreeToPointCloud(*cropped_tree_);
  feature_cloud_ = std::make_shared<FeatureCloud>(cloud, cfg_);

  feature_cloud_->ExtractKeypoints();

  feature_cloud_->ComputeSurfaceNormals();

  feature_cloud_->ComputeDescriptors();

  ShowPointCloudAndKeypoints();
}

