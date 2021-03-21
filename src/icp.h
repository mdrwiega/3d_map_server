#pragma once

#include <iostream>
#include <chrono>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <octomap_tools/maps_integrator_visualizer.h>

using std::chrono::high_resolution_clock;

namespace octomap_tools {

/**
 * @brief Calculates transformation with ICP method between two input pointclouds: model and scene
 */
class ICP {
 public:
  struct Config {
    int max_iter = 100;     // Max number of iterations (ICP)
    float max_nn_dist = 0.3;     // Max correspondence distance (NN)
    float fitness_eps = 0.05;    // Euclidean fitness epsilon (ICP)
    float fitness_score_dist = 0.5;
    float transf_eps = 0.001;    // Transformation epsilon (ICP)
    float scene_inflation_dist = 1.0;
    bool visualize = false;
    bool crop_scene = true;
    std::string output_dir;
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
    cfg_(std::move(config)) {
  }

  Result Align() {
    auto start = high_resolution_clock::now();

    PointCloudPtr scene (new PointCloud);
    if (cfg_.crop_scene) {
      cropSceneToInflatedModel(scene);
    } else {
      scene = scene_;
    }

    if (model_->empty()) {
      throw std::runtime_error("ICP: Empty model");
    }
    if (scene_->empty()) {
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
    icp.align(dummy_output);

    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(high_resolution_clock::now() - start);

    if (cfg_.visualize) {
      MapsIntegratorVisualizer visualizer({ false, true, cfg_.output_dir + "icp.png" });
      visualizer.VisualizeICP(scene, model_, icp.getFinalTransformation());
    }

    // Prepare result
    Result result;
    result.fitness_score = icp.getFitnessScore(cfg_.fitness_score_dist);
    result.transformation = icp.getFinalTransformation();
    result.processing_time_ms = static_cast<double>(dt.count());
    return result;
  }

 protected:
  void cropSceneToInflatedModel(PointCloudPtr& cropped_scene) {
    // Crop scene to model + inflation
    pcl::CropBox<Point> box_filter;
    box_filter.setInputCloud(scene_);
    Point pmin, pmax;
    pcl::getMinMax3D(*model_, pmin, pmax);
    auto inflation = cfg_.scene_inflation_dist;
    box_filter.setMin({pmin.x - inflation, pmin.y - inflation, pmin.z - inflation, 1});
    box_filter.setMax({pmax.x + inflation, pmax.y + inflation, pmax.z + inflation, 1});
    box_filter.filter(*cropped_scene);
  }

 private:
  PointCloudPtr scene_;
  PointCloudPtr model_;
  Config cfg_;
};

} // namespace octomap_tools