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

#include <alignment/alignment_method.h>
#include <alignment/alignment_validator.h>

using namespace std::chrono_literals;


namespace octomap_tools {

class NdtAlignment : public AlignmentMethod {
public:

  struct Config {};

  NdtAlignment(const Config& config, PointCloudPtr& scene, PointCloudPtr& model) :
    scene_(scene),
    model_(model),
    cfg_(config) {
  }

  AlignmentMethod::Result Align()
  {
    Result result;

    auto start = std::chrono::high_resolution_clock::now();
    // Based on PCL documentation
    // https://pointclouds.org/documentation/tutorials/normal_distributions_transform.html

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.05, 0.05, 0.05);
    approximate_voxel_filter.setInputCloud(model_);
    approximate_voxel_filter.filter(*filtered_cloud);
    std::cout << "Filtered cloud size: " << filtered_cloud->size() << " points" << std::endl;

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    ndt.setTransformationEpsilon(0.001);
    ndt.setStepSize(0.05);
    ndt.setResolution(0.5);
    ndt.setMaximumIterations(300);

    ndt.setInputSource(filtered_cloud);
    ndt.setInputTarget(scene_);

    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*output_cloud, init_guess);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
              << " score: " << ndt.getFitnessScore() << std::endl;

    result.transformation = ndt.getFinalTransformation();

    AlignmentValidator<Point> validator;
    validator.calculateCorrespondences(model_, scene_, result.transformation);
    result.fitness_score1 = validator.calcFitnessScore1();
    result.fitness_score2 = validator.calcFitnessScore2();
    result.fitness_score3 = validator.calcFitnessScore3();

    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    result.processing_time_ms = static_cast<float>(diff.count());

    return result;
  }

 private:
  PointCloudPtr scene_;
  PointCloudPtr model_;
  Config cfg_;
};

}