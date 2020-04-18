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
#include <pcl/visualization/pcl_visualizer.h>

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
    float fitness_score_dist = 0.5;
    float scene_inflation_dist = 1.0;
    bool visualize = false;
    bool crop_scene = true;
    std::string files_path_and_pattern;
  };

  struct Result {
    double fitness_score;
    Eigen::Matrix4f transformation;
    double processing_time_ms;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  ICP(const PointCloudPtr& scene, const PointCloudPtr& model, Config config) :
    scene_(scene),
    model_(model),
    cfg_(config) {
  }

  Result align() {
    auto start = std::chrono::high_resolution_clock::now();
    PointCloudPtr scene (new PointCloud);
    if (cfg_.crop_scene) {
      cropSceneToInflatedModel(scene);
    } else {
      scene = scene_;
    }

    if (model_->size() == 0) {
      throw std::runtime_error("ICP: Empty model");
    }

    if (scene_->size() == 0) {
      throw std::runtime_error("ICP: Empty scene");
    }

    pcl::IterativeClosestPoint <Point, Point> icp;
    icp.setMaxCorrespondenceDistance(cfg_.max_nn_dist);
    icp.setMaximumIterations(cfg_.max_iter);
    icp.setTransformationEpsilon(cfg_.transf_eps);
    icp.setEuclideanFitnessEpsilon(cfg_.fitness_eps);
    icp.setInputSource(model_);
    icp.setInputTarget(scene);

    PointCloud dummy_output;
    icp.align (dummy_output);

    auto diff_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start).count();

    if (cfg_.visualize) {
      MapsIntegratorVisualizer::Config visualizar_cfg { false, true, cfg_.files_path_and_pattern + "matching_icp.png" };
      MapsIntegratorVisualizer visualizer(visualizar_cfg);
      visualizer.visualizeICP(scene, model_, icp.getFinalTransformation());
    }
    return Result {icp.getFitnessScore(cfg_.fitness_score_dist), icp.getFinalTransformation(), static_cast<double>(diff_time)};
  }

 protected:
  void cropSceneToInflatedModel(PointCloudPtr& cropped_scene) {
    // Crop scene to model + inflation
    pcl::CropBox<Point> boxFilter;
    boxFilter.setInputCloud(scene_);
    Point pmin, pmax;
    pcl::getMinMax3D(*model_, pmin, pmax);
    auto inflation = cfg_.scene_inflation_dist;
    boxFilter.setMin({pmin.x - inflation, pmin.y - inflation, pmin.z - inflation, 1});
    boxFilter.setMax({pmax.x + inflation, pmax.y + inflation, pmax.z + inflation, 1});
    boxFilter.filter(*cropped_scene);
  }

 private:
  PointCloudPtr scene_;
  PointCloudPtr model_;
  Config cfg_;
};

}
