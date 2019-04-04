/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <iostream>
#include <chrono>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include "maps_integrator_visualizer.h"

namespace octomap_tools {

/**
 * @brief Calculates transformation with ICP method between two input pointclouds: model and scene
 */
class ICP {
 public:
  struct Config {
    unsigned max_iter = 100;     // Max number of iterations (ICP)
    float max_nn_dist = 0.3;     // Max correspondence distance (NN)
    float fitness_eps = 0.05;    // Euclidean fitness epsilon (ICP)
    float transf_eps = 0.001;    // Transformation epsilon (ICP)
    float scene_inflation_dist = 1.0;
    bool visualize = false;
    std::string files_path_and_pattern;
  };

  struct Result {
    float fitness_score;
    Eigen::Matrix4f transformation;
  };

  ICP(const PointCloudPtr& scene, const PointCloudPtr& model, Config config) :
    scene_(scene),
    model_(model),
    cfg_(config) {
  }

  PointCloudPtr cropSceneToInflatedModel() {
    // Crop scene to model + inflation
    pcl::CropBox<Point> boxFilter;
    boxFilter.setInputCloud(scene_);
    Point pmin, pmax;
    pcl::getMinMax3D(*model_, pmin, pmax);
    auto inflation = cfg_.scene_inflation_dist;
    boxFilter.setMin({pmin.x - inflation, pmin.y - inflation, pmin.z - inflation, 1});
    boxFilter.setMax({pmax.x + inflation, pmax.y + inflation, pmax.z + inflation, 1});
    PointCloudPtr cropped_scene (new PointCloud);
    boxFilter.filter(*cropped_scene);
    return cropped_scene;
  }

  Result align() {
    pcl::IterativeClosestPoint <Point, Point> icp;
    icp.setMaxCorrespondenceDistance(cfg_.max_nn_dist);
    icp.setMaximumIterations(cfg_.max_iter);
    icp.setTransformationEpsilon(cfg_.transf_eps);
    icp.setEuclideanFitnessEpsilon(cfg_.fitness_eps);

    icp.setInputSource(model_);
    icp.setInputTarget(scene_);

    PointCloud registration_output;
    icp.align (registration_output);
    return Result {(float)icp.getFitnessScore(cfg_.max_nn_dist), icp.getFinalTransformation()};
  }

  Result cropAndAlign() {
    auto start = std::chrono::high_resolution_clock::now();

    auto cropped_scene = cropSceneToInflatedModel();
    auto result = align();

    auto t_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);

    std::cout << "\nICP has converged, score is " << result.fitness_score << std::endl;
    std::cout << "ICP for scene (size: " << cropped_scene->size() << ") and model (size: "
        << model_->size() << ") takes: " << t_diff.count() << " ms." << std::endl;

    if (cfg_.visualize) {
      MapsIntegratorVisualizer::Config visualizar_cfg { false, true, cfg_.files_path_and_pattern + "matching_icp.png" };
      MapsIntegratorVisualizer visualizer(visualizar_cfg);
      visualizer.visualizeICP(cropped_scene, model_, result.transformation);
    }
    return result;
  }
 private:
  PointCloudPtr scene_;
  PointCloudPtr model_;
  Config cfg_;
};

}
