/******************************************************************************
 * Copyright (c) 2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>

#include <octomap_tools/transformations.h>
#include <octomap_tools/utils.h>
#include "test_utils.h"
#include <octomap_tools/feature_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "../include/octomap_tools/conversions.h"

using namespace Eigen;
using namespace octomap_tools;
using namespace octomap;
using namespace std::chrono;

class FeatureCloudTest : public ::testing::Test
{
 public:
  FeatureCloudTest() {
    configure();
  }

  ~FeatureCloudTest() {
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

  void configure() {
    cfg_.normal_radius = 15.0;
    cfg_.downsampling_radius = 0.15;
    cfg_.descriptors_radius = 1.5;
  }

  void ShowPointCloudAndKeypoints() {
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addCoordinateSystem(1.0);

    // Cloud --> BLUE
    auto cloud = feature_cloud_->getPointCloud();
    pcl::visualization::PointCloudColorHandlerCustom<Point> cloud_color(cloud, 0, 0, 255);
    viewer.addPointCloud(cloud, cloud_color, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    // Keypoints --> RED
    auto keypoints = feature_cloud_->getKeypoints();
    pcl::visualization::PointCloudColorHandlerCustom<Point> keypoints_color(keypoints, 255, 0, 0);
    viewer.addPointCloud(keypoints, keypoints_color, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keypoints");

    while (!viewer.wasStopped()) {
      viewer.spinOnce(100);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  OcTreePtr orig_tree_;
  OcTreePtr cropped_tree_;
  FeatureCloud::Config cfg_;
  std::shared_ptr<FeatureCloud> feature_cloud_;
  Eigen::Matrix4f result_transf_;
};

TEST_F(FeatureCloudTest, Test_fr)
{
  std::string octomap_name = "fr_079";
  auto cloud_min = Vector3f(-5, -5, 0.0);
  auto cloud_max = Vector3f(5, 5, 2.0);
  PrepareOcTree(octomap_name, cloud_min, cloud_max);

  auto cloud = OcTreeToPointCloud(*cropped_tree_);
  feature_cloud_ = std::make_shared<FeatureCloud>(cloud, cfg_);
  auto start = std::chrono::high_resolution_clock::now();
  feature_cloud_->downsampleAndExtractKeypoints();
  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
  std::cout << "From cloud (size: " << feature_cloud_->getPointCloud()->size() << ") extracted "
      << feature_cloud_->getKeypoints()->size() << " keypoints in: "
      << diff.count() << " ms." << std::endl;
  feature_cloud_->computeSurfaceNormals();
  feature_cloud_->computeDescriptors();
  ShowPointCloudAndKeypoints();
}

