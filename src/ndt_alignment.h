#pragma once

#include <pcl/point_types.h>

#include <octomap_tools/types.h>

#include <iostream>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;


namespace octomap_tools {

class NdtAlignment {
public:

  struct Config {
  };

  struct Result {
    float fitness_score1{std::numeric_limits<float>::max()};
    Eigen::Matrix4f transformation;
    float processing_time_ms;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  NdtAlignment(const Config& config, PointCloudPtr& scene, PointCloudPtr& model) :
    scene_cloud_(scene),
    model_cloud_(model),
    cfg_(config) {
  }

  Result Align()
  {
    Result result;

    auto start = std::chrono::high_resolution_clock::now();
    // Based on PCL documentation
    // https://pointclouds.org/documentation/tutorials/normal_distributions_transform.html

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);
    approximate_voxel_filter.setInputCloud(model_cloud_);
    approximate_voxel_filter.filter(*filtered_cloud);
    std::cout << "Filtered cloud size: " << filtered_cloud->size() << " points" << std::endl;

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(200);

    ndt.setInputSource(filtered_cloud);
    ndt.setInputTarget(scene_cloud_);

    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*output_cloud, init_guess);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
              << " score: " << ndt.getFitnessScore() << std::endl;

    result.transformation = ndt.getFinalTransformation();

    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    result.processing_time_ms = static_cast<float>(diff.count());

    return result;
  }

 private:
  PointCloudPtr scene_cloud_;
  PointCloudPtr model_cloud_;
  Config cfg_;
};

}