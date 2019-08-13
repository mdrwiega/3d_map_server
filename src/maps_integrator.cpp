#include "octomap_tools/maps_integrator.h"

#include <iostream>
#include <iomanip>
#include <ctime>

#include "md_utils/math/transformations.h"
#include "octomap_tools/types_conversions.h"

#include <pcl/point_cloud.h>

#include "utils/table_printer.h"

#include <octomap_tools/transformations.h>
#include <octomap_tools/features_matching.h>
#include "utils/math.h"

namespace octomap_tools {

MapsIntegrator::Result MapsIntegrator::compute() {
  auto start = std::chrono::high_resolution_clock::now();

  // Initial features based alignment
  PointCloud::Ptr best_model(new PointCloud);
  FeaturesMatching features_matching(cfg_.template_alignment, scene_, model_);
  FeaturesMatching::Result ia_result = features_matching.initialAlignment(*best_model);

  std::cout << std::setprecision(6) << "\nIA fitness score: " << ia_result.fitness_score << "\n";
  std::cout << std::setprecision(1) << "IA processing takes: " << ia_result.processing_time_ms << " ms\n";
  std::cout << "IA transformation:" << transfMatrixToXyzRpyString(ia_result.transformation) << "\n";

  // ICP correction
  if (cfg_.icp_correction) {
      PointCloud::Ptr icp_model(new PointCloud);
      pcl::transformPointCloud(*best_model, *icp_model, ia_result.transformation);
      ICP icp(scene_, icp_model, cfg_.icp);
      ICP::Result icp_result = icp.align();

      std::cout << std::setprecision(6) << "\nICP fitness score is " << icp_result.fitness_score << std::endl;
      std::cout << std::setprecision(1) << "ICP processing takes: " << icp_result.processing_time_ms << " ms." << std::endl;

      if (icp_result.fitness_score < ia_result.fitness_score) {
        ia_result.fitness_score = icp_result.fitness_score;
        ia_result.transformation = icp_result.transformation * ia_result.transformation;
        std::cout << "ICP transformation:\n" << transfMatrixToXyzRpyString(ia_result.transformation) << "\n";
      }
  }

  // Create result
  Point pmin, pmax;
  pcl::getMinMax3D(*best_model, pmin, pmax);
  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
  result_ = Result(diff.count(), ia_result.fitness_score, pmin, pmax, ia_result.transformation);
//  std::cout << "Configuration\n" << cfg_.toTable();
  result_.PrintResult();

  OcTreePtr merged_tree;
  if (cfg_.integrate_octomaps) {
    merged_tree = integrateOctrees(ia_result.transformation);
  }

  if (cfg_.dump_to_file_) {
    DumpConfigAndResultsToFile();
  }
  if (cfg_.show_visualization_) {
    MapsIntegratorVisualizer::Config visualizar_cfg { false, true, cfg_.files_path_and_pattern + "matching.png" };
    MapsIntegratorVisualizer visualizer(visualizar_cfg);
    visualizer.visualize(ia_result.transformation, scene_, best_model, model_, spiral_blocks_);
  }
  if (cfg_.show_two_pointclouds) {
    PointCloudPtr transformed_model (new PointCloud);
    pcl::transformPointCloud(*model_, *transformed_model, ia_result.transformation);
    MapsIntegratorVisualizer::Config visualizar_cfg { false, true, cfg_.files_path_and_pattern + "matching_2clouds.png" };
    MapsIntegratorVisualizer visualizer(visualizar_cfg);
    visualizer.visualizeClouds(scene_, transformed_model);
  }
  if (cfg_.show_integrated_octomaps) {

  }
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
  std::cout << "Final result:" << std::fixed << std::setprecision(1) << std::endl;
  std::cout << "Total processing time: " << time_ms / 1000.0 << " s." << std::endl;
  std::cout << "Fitness score: " << std::setprecision(6) << fitness_score << std::endl;
  std::cout << "Transformation: " << transfMatrixToXyzRpyString(transformation) << std::endl;
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

  stream << template_alignment.model_size_thresh_ << "; " << fitness_score_thresh << "; " << template_alignment.keypoints_thresh_ << "; " << template_alignment.cell_size_x << "; " << template_alignment.cell_size_y << "; "
      << icp.max_iter << "; " << icp.max_nn_dist << "; " << icp.fitness_eps << "; " << icp.transf_eps << "; " << icp.scene_inflation_dist
      << "; " << template_alignment.feature_cloud.normal_radius << "; " << template_alignment.feature_cloud.downsampling_radius << "; " << template_alignment.feature_cloud.descriptors_radius
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
      << template_alignment.model_size_thresh_ << fitness_score_thresh << template_alignment.keypoints_thresh_ << template_alignment.cell_size_x << template_alignment.cell_size_y
      << icp.max_iter << icp.max_nn_dist << icp.fitness_eps << icp.transf_eps << icp.scene_inflation_dist
      << template_alignment.feature_cloud.normal_radius << template_alignment.feature_cloud.downsampling_radius << template_alignment.feature_cloud.descriptors_radius
      << template_alignment.min_sample_distance << template_alignment.max_correspondence_distance
      << template_alignment.nr_iterations;

  tp.printFooter();
  return stream.str();
}

}
