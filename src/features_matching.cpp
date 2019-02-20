#include "features_matching.h"

#include <thread>
#include <iostream>
#include <iomanip>
#include <ctime>

#include "md_utils/math/transformations.h"
#include "utils/types_conversions.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/board.h>
#include <pcl/registration/icp.h>

#include <features_matching_visualizer.h>
#include <octomap_integrator.h>

namespace octomap_tools {

FeatureMatching::Result FeatureMatching::computeWithSingleModel(PointCloudPtr& scene_cloud, PointCloudPtr& model_cloud,
                                         bool box_filter_model,
                                         const Eigen::Vector4f& box_filter_min,
                                         const Eigen::Vector4f& box_filter_max) {
  auto start = std::chrono::high_resolution_clock::now();

  setFullModel(model_cloud);
  DivideFullModelIntoBlocks(cfg_.cell_size_x_, cfg_.cell_size_y_);

  FeatureCloudPtr scene = FeatureCloudPtr (new FeatureCloud);
  FeatureCloudPtr model = FeatureCloudPtr (new FeatureCloud);

  scene->setInputCloud(scene_cloud);

  if (box_filter_model) {
    PointCloudPtr filtered_model (new PointCloud);
    pcl::CropBox<Point> boxFilter;
    boxFilter.setMin(box_filter_min);
    boxFilter.setMax(box_filter_max);
    boxFilter.setInputCloud(model_cloud);
    boxFilter.filter(*filtered_model);
    model->setInputCloud(filtered_model);
  } else {
    model->setInputCloud(model_cloud);
  }

  if (model->getPointCloud()->size() < cfg_.model_size_thresh_) {
    std::cout << "Model is too small (size= " << model->getPointCloud()->size() << "\n";
    return Result();
  }

  template_align_.addTemplateCloud (*model);
  template_align_.setTargetCloud (*scene);

  TemplateAlignment::Result best_alignment;
  template_align_.findBestAlignment (best_alignment);
  printf ("Best fitness score: %f\n", best_alignment.fitness_score);

  Point pmin, pmax;
  pcl::getMinMax3D(*model->getPointCloud(), pmin, pmax);
  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);

  Result result (diff.count(), best_alignment.fitness_score, pmin, pmax, best_alignment.final_transformation);

  if (cfg_.show_visualization_) {
    FeatureMatchingVisualizer::Config visualizar_cfg { false, true, cfg_.files_path_and_pattern + "matching.png" };
    FeatureMatchingVisualizer visualizer(visualizar_cfg);
    visualizer.visualize(best_alignment.final_transformation, scene, model, full_model_, spiral_blocks_);
  }
  return result;
}

FeatureMatching::Result FeatureMatching::computeWithModelDivision(PointCloudPtr& scene_cloud,
                                                                  PointCloudPtr& model_cloud) {
  auto start = std::chrono::high_resolution_clock::now();
  auto scene = std::make_shared<FeatureCloud>(scene_cloud, cfg_.feature_cloud);

  template_align_.setTargetCloud (*scene);

  setFullModel(model_cloud);
  DivideFullModelIntoBlocks(cfg_.cell_size_x_, cfg_.cell_size_y_);

  pcl::CropBox<Point> boxFilter;
  boxFilter.setInputCloud(model_cloud);
  Point pmin, pmax;
  pcl::getMinMax3D(*model_cloud, pmin, pmax);

  // Add templates
  for (size_t i = 0; i < spiral_blocks_.size(); ++i) {
    auto cell = spiral_blocks_[i];
    Eigen::Vector4f model_min = {cell.min(0), cell.min(1), pmin.z, 1};
    Eigen::Vector4f model_max = {cell.max(0), cell.max(1), pmax.z, 1};

    boxFilter.setMin(model_min);
    boxFilter.setMax(model_max);
    PointCloudPtr filtered_model (new PointCloud);
    boxFilter.filter(*filtered_model);

    auto model = std::make_shared<FeatureCloud>(filtered_model,cfg_.feature_cloud);
    model->setInputCloud(filtered_model);

    if (filtered_model->size() < cfg_.model_size_thresh_) {
      std::cout << "Model is too small (size: " << filtered_model->size() << ")\n";
      continue;
    }
    if (model->getKeypoints()->size() < cfg_.keypoints_thresh_) {
      std::cout << "Not enought keypoints: " << model->getKeypoints()->size() << "\n";
      continue;
    }

    template_align_.addTemplateCloud (*model);
    std::cout << "Added template with " << model->getKeypoints()->size() << " keypoints\n";
  }

  TemplateAlignment::Result best_alignment;
  int best_idx = template_align_.findBestAlignment(best_alignment);
  printf ("Best fitness score: %f\n", best_alignment.fitness_score);

  if (cfg_.icp_correction) {
      auto start_icp = std::chrono::high_resolution_clock::now();

      PointCloud::Ptr icp_model(new PointCloud);
      pcl::transformPointCloud(*template_align_.getTemplate(best_idx).getPointCloud(), *icp_model, best_alignment.final_transformation);

      // Crop scene to model + inflation
      boxFilter.setInputCloud(scene->getPointCloud());
      pcl::getMinMax3D(*icp_model, pmin, pmax);
      auto inflation = cfg_.icp.scene_inflation_dist;
      boxFilter.setMin({pmin.x - inflation, pmin.y - inflation, pmin.z - inflation, 1});
      boxFilter.setMax({pmax.x + inflation, pmax.y + inflation, pmax.z + inflation, 1});
      PointCloudPtr cropped_scene (new PointCloud);
      boxFilter.filter(*cropped_scene);

      pcl::IterativeClosestPoint <Point, Point> icp;
      icp.setMaxCorrespondenceDistance(cfg_.icp.max_nn_dist);
      icp.setMaximumIterations(cfg_.icp.max_iter);
      icp.setTransformationEpsilon(cfg_.icp.transf_eps);
      icp.setEuclideanFitnessEpsilon(cfg_.icp.fitness_eps);

      icp.setInputSource(icp_model);
      icp.setInputTarget(cropped_scene);

      PointCloud registration_output;
      icp.align (registration_output);
      std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;

      auto diff_icp = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::high_resolution_clock::now() - start_icp);
      std::cout << "ICP for scene (size: " << cropped_scene->size() << ") and model (size: "
                << icp_model->size() << ") takes: " << diff_icp.count() << " ms." << std::endl;

      if (cfg_.visualize_icp) {
        FeatureMatchingVisualizer::Config visualizar_cfg { false, true, cfg_.files_path_and_pattern + "matching_icp.png" };
        FeatureMatchingVisualizer visualizer(visualizar_cfg);
        visualizer.visualizeICP(cropped_scene, icp_model, icp.getFinalTransformation());
      }

      if (icp.getFitnessScore() < best_alignment.fitness_score) {
        best_alignment.fitness_score = icp.getFitnessScore();
        best_alignment.final_transformation = best_alignment.final_transformation * icp.getFinalTransformation();
      }
  }

  // Create result
  FeatureCloudPtr best_model = FeatureCloudPtr (new FeatureCloud);
  *best_model = template_align_.getTemplate(best_idx);
  pcl::getMinMax3D(*(best_model->getPointCloud()), pmin, pmax);
  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
  result_ = Result(diff.count(), best_alignment.fitness_score, pmin, pmax, best_alignment.final_transformation);

  if (cfg_.dump_to_file_) {
    DumpConfigAndResultsToFile();
  }

  if (cfg_.show_visualization_) {
    FeatureMatchingVisualizer::Config visualizar_cfg { false, true, cfg_.files_path_and_pattern + "matching.png" };
    FeatureMatchingVisualizer visualizer(visualizar_cfg);
    visualizer.visualize(best_alignment.final_transformation, scene, best_model, full_model_, spiral_blocks_);
  }
  return result_;
}

void FeatureMatching::DivideFullModelIntoBlocks(float block_size_x, float block_size_y) {
  if (!full_model_) {
    throw std::runtime_error(std::string("Full model is not set!"));
  }

  Eigen::Vector4f map_min, map_max;
  pcl::getMinMax3D (*full_model_, map_min, map_max);
  Eigen::Vector2f rectangle_min (map_min.x(), map_min.y());
  Eigen::Vector2f rectangle_max (map_max.x(), map_max.y());
  Eigen::Vector2f step_xy (block_size_x, block_size_y);

  std::cout << "Generation of cells for rectangle min: (" << rectangle_min(0) << ", " << rectangle_min(1) << ")  max: (" << rectangle_max(0) << ", " << rectangle_max(1) << ")\n";
  spiral_blocks_ = generateBlocksInSpiralOrder(rectangle_min, rectangle_max, step_xy);
}

void FeatureMatching::setFullModel(PointCloudPtr& full_model) {
  full_model_ = PointCloudPtr(new PointCloud);
  *full_model_ = *full_model;
}

void FeatureMatching::DumpConfigAndResultsToFile() {
  std::ofstream file(cfg_.files_path_and_pattern + "_cfg_results.txt");
  file << cfg_.toString();
  file << result_.toString();
}

std::string FeatureMatching::Config::toString() {
  std::stringstream stream;
  md::TablePrinter tp(&stream);
  const std::string columns[] = {
      "model_ss", "scene_ss", "rf_rad", "descr_rad", "cg_size",
      "cg_thresh", "model_size_thresh", "keypoints_thresh", "cell_size_x", "cell_size_y",
      "icp.max_iter", "icp.max_nn_dist", "icp.fitness_eps", "icp.transf_eps", "icp.scene_infl_dist",
      "fc.normal_radius", "fc.downsampling_radius", "fc.descriptors_radius", "ta.min_sample_dist", "ta.max_corr_dist",
      "ta.nr_iter"
  };

  for (const auto& i : columns) {
    tp.addColumn(i);
  }

  tp.printTitle("Feature matching parameters");
  tp.printHeader();

  tp << model_ss_ << scene_ss_ << rf_rad_ << descr_rad_ << cg_size_
      << cg_thresh_ << model_size_thresh_ << keypoints_thresh_ << cell_size_x_ << cell_size_y_
      << icp.max_iter << icp.max_nn_dist << icp.fitness_eps << icp.transf_eps << icp.scene_inflation_dist
      << feature_cloud.normal_radius << feature_cloud.downsampling_radius << feature_cloud.descriptors_radius
      << template_alignment.min_sample_distance << template_alignment.max_correspondence_distance
      << template_alignment.nr_iterations;

  tp.printFooter();
  return stream.str();
}

}
