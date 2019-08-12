#include <thread>
#include <iostream>
#include <iomanip>
#include <ctime>

#include "md_utils/math/transformations.h"
#include "octomap_tools/types_conversions.h"

#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/board.h>
#include <pcl/filters/crop_box.h>

#include "utils/table_printer.h"

#include <octomap_tools/transformations.h>
#include "../include/octomap_tools/maps_integrator.h"
#include <octomap_tools/thread_pool.h>

namespace octomap_tools {


void threadFcn(int nr, PointCloudPtr& full_model, std::vector<Rectangle> rec, TemplateAlignment& template_align,
               MapsIntegrator::Config& cfg) {
  Point pmin, pmax;
  pcl::getMinMax3D(*full_model, pmin, pmax);
  Eigen::Vector4f model_min = {rec[nr].min(0), rec[nr].min(1), pmin.z, 1};
   Eigen::Vector4f model_max = {rec[nr].max(0), rec[nr].max(1), pmax.z, 1};

   pcl::CropBox<Point> boxFilter;
   boxFilter.setInputCloud(full_model);
   boxFilter.setMin(model_min);
   boxFilter.setMax(model_max);
   PointCloudPtr filtered_model (new PointCloud);
   boxFilter.filter(*filtered_model);

   if (filtered_model->size() < cfg.model_size_thresh_) {
     std::cout << "Model is too small (size: " << filtered_model->size() << ")\n";
     return;
   }

   auto model = std::make_shared<FeatureCloud>(filtered_model, cfg.feature_cloud);
   model->downsampleAndExtractKeypoints();
   if (model->getKeypoints()->size() < cfg.keypoints_thresh_) {
     std::cout << "Not enought keypoints: " << model->getKeypoints()->size() << "\n";
     return;
   }
   model->computeSurfaceNormals();
   model->computeDescriptors();

   auto res = template_align.addTemplateCloudAndAlign(*model);
   std::cout << "Added template with " << model->getKeypoints()->size() << " keypoints\n";

   //     if (res.fitness_score < cfg_.fitness_score_thresh) {
   //       std::cout << "Fitness score: " << res.fitness_score << " is smaller than threshold: " << cfg_.fitness_score_thresh << "\n";
   //       break;
   //     }
}

TemplateAlignment::Result MapsIntegrator::initialAlignment(
    const FeatureCloudPtr& scene, PointCloud& best_model) {
  TemplateAlignment template_align_(cfg_.template_alignment);
  template_align_.setTargetCloud (*scene);
  divideFullModelIntoBlocks(cfg_.cell_size_x_, cfg_.cell_size_y_);

  std::vector<Result, Eigen::aligned_allocator<Result>> results_;

  std::shared_ptr<thread_pool::ThreadPool> thread_pool = thread_pool::createThreadPool();
  std::vector<std::future<void>> thread_futures;
  for (std::uint32_t i = 0; i < spiral_blocks_.size(); ++i) {
      thread_futures.emplace_back(thread_pool->submit(
          threadFcn, i, std::ref(model_), std::ref(spiral_blocks_), std::ref(template_align_), std::ref(cfg_)));
  }

  for (auto& it: thread_futures) {
      it.wait();
  }

  best_model = template_align_.getBestTemplate();
  return template_align_.findBestAlignment();
}

MapsIntegrator::Result MapsIntegrator::compute() {
  auto start = std::chrono::high_resolution_clock::now();

  auto scene = std::make_shared<FeatureCloud>(scene_, cfg_.feature_cloud);
  scene->processInput();

  PointCloud best_model_pointcloud;
  TemplateAlignment::Result init_alignment = initialAlignment(scene, best_model_pointcloud);

  std::cout << "Best fitness score: " << init_alignment.fitness_score << "\n";
  std::cout << "Final transformation: " << init_alignment.final_transformation << "\n";
  if (cfg_.icp_correction) {
      PointCloud::Ptr icp_model(new PointCloud);
      pcl::transformPointCloud(best_model_pointcloud, *icp_model, init_alignment.final_transformation);
      std::cout << "Point 3\n";
      ICP icp(scene->getPointCloud(), icp_model, cfg_.icp);
      ICP::Result icp_result = icp.cropAndAlign();
      if (icp_result.fitness_score < init_alignment.fitness_score) {
        init_alignment.fitness_score = icp_result.fitness_score;
        init_alignment.final_transformation = init_alignment.final_transformation * icp_result.transformation;
      }
  }
  // Create result
  Point pmin, pmax;
  pcl::getMinMax3D(best_model_pointcloud, pmin, pmax);
  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
  result_ = Result(diff.count(), init_alignment.fitness_score, pmin, pmax, init_alignment.final_transformation);
  std::cout << "Configuration\n" << cfg_.toTable();
  result_.PrintResult();

  OcTreePtr merged_tree;
  if (cfg_.integrate_octomaps) {
    merged_tree = integrateOctrees(init_alignment.final_transformation);
  }

  if (cfg_.dump_to_file_) {
    DumpConfigAndResultsToFile();
  }
  if (cfg_.show_visualization_) {
    MapsIntegratorVisualizer::Config visualizar_cfg { false, true, cfg_.files_path_and_pattern + "matching.png" };
    MapsIntegratorVisualizer visualizer(visualizar_cfg);
    PointCloud::Ptr best_model_ptr(new PointCloud);
    *best_model_ptr = best_model_pointcloud;
    FeatureCloudPtr best_model = std::make_shared<FeatureCloud>();
    best_model->setInputCloud(best_model_ptr);
    visualizer.visualize(init_alignment.final_transformation, scene, best_model, model_, spiral_blocks_);
  }
  std::cout << "POINT 3\n";
  if (cfg_.show_two_pointclouds) {
    PointCloudPtr transformed_model (new PointCloud);
    pcl::transformPointCloud(*model_, *transformed_model, init_alignment.final_transformation);
    MapsIntegratorVisualizer::Config visualizar_cfg { false, true, cfg_.files_path_and_pattern + "matching_2clouds.png" };
    MapsIntegratorVisualizer visualizer(visualizar_cfg);
    visualizer.visualizeClouds(scene->getPointCloud(), transformed_model);
  }
  if (cfg_.show_integrated_octomaps) {

  }
  std::cout << "POINT 4\n";

  return result_;
}

OcTreePtr MapsIntegrator::integrateOctrees(const Eigen::Matrix4f& transformation) {
  auto start = std::chrono::high_resolution_clock::now();

  float octree_res = 0.05;
  auto tree_scene = PointCloudToOctree(*scene_, octree_res);
  auto tree_model = PointCloudToOctree(*model_, octree_res);
  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
  std::cout << "Pointclouds converted to octrees in: " << diff.count() << " ms." << std::endl;

  start = std::chrono::high_resolution_clock::now();
  auto tree_model_transformed = transformOctree(tree_model, transformation);
  diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
  std::cout << "Model octomap transformed in: " << diff.count() << " ms." << std::endl;

  start = std::chrono::high_resolution_clock::now();
  auto merged_tree = sumOctrees(*tree_model_transformed, tree_scene);
  diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
  std::cout << "Octomaps merged in: " << diff.count() << " ms." << std::endl;
  return merged_tree;
}

void MapsIntegrator::divideFullModelIntoBlocks(float block_size_x, float block_size_y) {
  if (model_->size() <= 0) {
    throw std::runtime_error(std::string("Full model is not set!"));
  }

  Eigen::Vector4f map_min, map_max;
  pcl::getMinMax3D (*model_, map_min, map_max);
  Eigen::Vector2f rectangle_min (map_min.x(), map_min.y());
  Eigen::Vector2f rectangle_max (map_max.x(), map_max.y());
  Eigen::Vector2f step_xy (block_size_x, block_size_y);

  std::cout << "Generation of cells for rectangle "
      << "min: (" << rectangle_min(0) << ", " << rectangle_min(1) << ")  "
      << "max: (" << rectangle_max(0) << ", " << rectangle_max(1) << ")\n";
  spiral_blocks_ = generateBlocksInSpiralOrder(rectangle_min, rectangle_max, step_xy);
}

void MapsIntegrator::DumpConfigAndResultsToFile() {
  std::ofstream file(cfg_.files_path_and_pattern + "cfg_results.txt");
  file << cfg_.toTable();
  file << result_.toString();
}

MapsIntegrator::Result::Result(float _time_ms, float _fitness_score,
                                Point _model_min, Point _model_max, Eigen::Matrix4f _tf) :
  time_ms(_time_ms),
  fitness_score(_fitness_score),
  model_min(_model_min),
  model_max(_model_max),
  transformation(_tf) {
}

std::string MapsIntegrator::Result::toString() {
  std::stringstream stream;
  md::TablePrinter tp(&stream);
  const std::string columns[] = {
      "time[ms]", "fitness_score", "model_min_x", "model_min_y", "model_max_x", "model_max_y"
  };

  for (const auto& i : columns) {
    tp.addColumn(i);
  }

  tp.printTitle("Feature matching results");
  tp.printHeader();
  tp << time_ms << fitness_score << model_min.x << model_min.y << model_max.x << model_max.y;
  tp.printFooter();
  return stream.str();
}

void MapsIntegrator::Result::PrintResult() {
  std::cout << "Result:" << std::endl;
  std::cout << "Processing time: " << time_ms / 1000.0 << " s." << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Block  MIN: (" << model_min.x << ", " << model_min.y << ", " << model_min.z << ")  "
      << "MAX: (" << model_max.x << ", " << model_max.y << ", " << model_max.z << ")\n";
  std::cout << md::transformationMatrixToString(transformation);
}

std::string MapsIntegrator::Config::toString() {
  std::stringstream stream;
  auto header = getHeader();
  for (const auto& i : header) {
    if (&i != &header.back())
      stream << i << "; ";
    else
      stream << i << std::endl;
  }

  stream << model_size_thresh_ << "; " << fitness_score_thresh << "; " << keypoints_thresh_ << "; " << cell_size_x_ << "; " << cell_size_y_ << "; "
      << icp.max_iter << "; " << icp.max_nn_dist << "; " << icp.fitness_eps << "; " << icp.transf_eps << "; " << icp.scene_inflation_dist
      << "; " << feature_cloud.normal_radius << "; " << feature_cloud.downsampling_radius << "; " << feature_cloud.descriptors_radius
      << "; " << template_alignment.min_sample_distance << "; " << template_alignment.max_correspondence_distance
      << "; " << template_alignment.nr_iterations << std::endl;
  return stream.str();
}

std::vector<std::string> MapsIntegrator::Config::getHeader() {
  return  { "model_size_thresh", "fitness_score_thresh", "keypoints_thresh", "cell_size_x", "cell_size_y",
            "icp.max_iter", "icp.max_nn_dist", "icp.fitness_eps", "icp.transf_eps", "icp.scene_infl_dist",
            "fc.normal_radius", "fc.downsampling_radius", "fc.descriptors_radius", "ta.min_sample_dist", "ta.max_corr_dist",
            "ta.nr_iter" };
}

std::string MapsIntegrator::Config::toTable() {
  std::stringstream stream;
  md::TablePrinter tp(&stream);

  for (const auto& i : getHeader()) {
    tp.addColumn(i);
  }

  tp.printTitle("Features matching parameters");
  tp.printHeader();

  tp
      << model_size_thresh_ << fitness_score_thresh << keypoints_thresh_ << cell_size_x_ << cell_size_y_
      << icp.max_iter << icp.max_nn_dist << icp.fitness_eps << icp.transf_eps << icp.scene_inflation_dist
      << feature_cloud.normal_radius << feature_cloud.downsampling_radius << feature_cloud.descriptors_radius
      << template_alignment.min_sample_distance << template_alignment.max_correspondence_distance
      << template_alignment.nr_iterations;

  tp.printFooter();
  return stream.str();
}

}
