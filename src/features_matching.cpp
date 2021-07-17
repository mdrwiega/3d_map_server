#include <features_matching.h>

#include <chrono>
#include <memory>

#include <ros/console.h>

#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <octomap_tools/maps_integrator_visualizer.h>

#include <common/thread_pool.h>
#include <common/conversions.h>
#include <validation.h>
#include <model_decomposition.h>

using namespace pcl;
using namespace pcl::registration;

namespace octomap_tools {

FeaturesMatching::FeaturesMatching(const Config& config, PointCloudPtr& scene, PointCloudPtr& model) :
  scene_cloud_(scene),
  model_cloud_(model),
  cfg_(config) {
}

FeaturesMatching::Result FeaturesMatching::align() {
  auto start = std::chrono::high_resolution_clock::now();

  // Calculate scene descriptors
  auto scene = std::make_shared<FeatureCloud>(scene_cloud_, cfg_.feature_cloud);
  scene->ProcessInput();

  // Calculate model descriptors
  auto model = std::make_shared<FeatureCloud>(model_cloud_, cfg_.feature_cloud);
  model->ProcessInput();

  auto result = FeaturesMatching::Align(0, cfg_, model, scene);

  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
  result.processing_time_ms = static_cast<float>(diff.count());

  return result;
}

FeaturesMatching::Result FeaturesMatching::DivideModelAndAlign(PointCloud& best_model) {
  auto start = std::chrono::high_resolution_clock::now();

  // Calculate scene descriptors
  auto scene = std::make_shared<FeatureCloud>(scene_cloud_, cfg_.feature_cloud);
  scene->ProcessInput();

  // Divide model into submodels/blocks
  Eigen::Vector4f map_min, map_max;
  pcl::getMinMax3D (*model_cloud_, map_min, map_max);
  std::vector<Rectangle> blocks = RectangularModelDecomposition(
    map_min, map_max, cfg_.cell_size_x, cfg_.cell_size_y);

  PCL_INFO("\nStarting features matching alignment. There are %d blocks\n", blocks.size());

  // Initialize thread pool and start workers
  thread_pool::ThreadPool thread_pool;
  std::vector<std::future<FeaturesMatching::ThreadResult>> thread_futures;
  for (std::uint32_t i = 0; i < blocks.size(); ++i) {
    thread_futures.emplace_back(thread_pool.Submit(
        AlignmentThread, i, std::ref(model_cloud_), std::ref(blocks[i]), std::ref(scene), std::ref(cfg_)));
  }

  // Wait till all workers finish
  std::vector<FeaturesMatching::ThreadResult> results;
  for (auto& it: thread_futures) {
    it.wait();
    auto result = it.get();
    results.emplace_back(result);
    if (result.result.fitness_score1 < 0.05) {
      PCL_ERROR("\nModel matched. Stop processing.\n");
      break;
    }
  }


  // thread_pool::ThreadPool thread_pool;
  // size_t threads_num = std::thread::hardware_concurrency();
  // unsigned tasks_num = blocks.size();
  // unsigned tasks_allocated = 0;
  // std::vector<FeaturesMatching::ThreadResult> results;

  // while (tasks_allocated < tasks_num) {
  //   std::vector<std::future<FeaturesMatching::ThreadResult>> thread_futures;
  //   for (size_t i = 0; i < threads_num; ++i) {
  //     if (tasks_allocated < tasks_num) {
  //       thread_futures.emplace_back(thread_pool.Submit(
  //           AlignmentThread, tasks_allocated, std::ref(model_cloud_), std::ref(blocks[tasks_allocated]), std::ref(scene), std::ref(cfg_)));
  //       tasks_allocated++;
  //     }
  //     else {
  //       break;
  //     }
  //   }

  //   // Wait till all workers finish
  //   for (auto& it: thread_futures) {
  //     it.wait();
  //     auto result = it.get();
  //     results.emplace_back(result);
  //     PCL_INFO("\nModel fitness score: %.2f\n", result.result.fitness_score1);
  //     if (result.result.fitness_score1 < 0.1) {
  //       PCL_ERROR("\nModel matched. Stop processing.\n");
  //       tasks_allocated = tasks_num;
  //       break;
  //     }
  //   }


  // }


  // Find best result from all threads results
  ThreadResult t_result = FindBestAlignment(results);

  // Crop model to get the best matched block
  Point pmin, pmax;
  pcl::getMinMax3D(*model_cloud_, pmin, pmax);
  Eigen::Vector4f model_min = {blocks[t_result.thread_num].min(0), blocks[t_result.thread_num].min(1), pmin.z, 1};
  Eigen::Vector4f model_max = {blocks[t_result.thread_num].max(0), blocks[t_result.thread_num].max(1), pmax.z, 1};
  pcl::CropBox<Point> box_filter;
  box_filter.setInputCloud(model_cloud_);
  box_filter.setMin(model_min);
  box_filter.setMax(model_max);
  box_filter.filter(best_model);

  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);

  auto result = t_result.result;
  result.processing_time_ms = static_cast<float>(diff.count());
  return result;
}

FeaturesMatching::ThreadResult FeaturesMatching::AlignmentThread(int nr,
                                                                PointCloudPtr& full_model,
                                                                Rectangle block,
                                                                const FeatureCloudPtr& scene,
                                                                FeaturesMatching::Config& cfg) {
  Point pmin, pmax;
  pcl::getMinMax3D(*full_model, pmin, pmax);
  Eigen::Vector4f model_min = {block.min(0), block.min(1), pmin.z, 1};
  Eigen::Vector4f model_max = {block.max(0), block.max(1), pmax.z, 1};

  pcl::CropBox<Point> box_filter;
  box_filter.setInputCloud(full_model);
  box_filter.setMin(model_min);
  box_filter.setMax(model_max);
  PointCloudPtr filtered_model (new PointCloud);
  box_filter.filter(*filtered_model);

  if (filtered_model->size() < cfg.model_size_thresh_) {
    PCL_WARN("\nT%d: Model is too small (size: %d)", nr, filtered_model->size());
    return FeaturesMatching::ThreadResult();
  }

  auto model = std::make_shared<FeatureCloud>(filtered_model, cfg.feature_cloud);
  model->ExtractKeypoints();
  if (model->GetKeypoints()->size() < cfg.keypoints_thresh_) {
    PCL_WARN("\nT%d: Not enough keypoints: %d", nr, model->GetKeypoints()->size());
    return FeaturesMatching::ThreadResult();
  }
  model->ComputeSurfaceNormals();
  model->ComputeDescriptors();

  FeaturesMatching::ThreadResult t_result;
  t_result.valid = true;
  t_result.thread_num = nr;
  t_result.result = FeaturesMatching::Align(nr, cfg, model, scene);
  return t_result;
}

FeaturesMatching::Result FeaturesMatching::Align(int nr,
                                                 FeaturesMatching::Config& cfg,
                                                 const FeatureCloudPtr& model,
                                                 const FeatureCloudPtr& scene) {
  Point ppmin, ppmax;
  pcl::getMinMax3D(*(model->GetPointCloud()), ppmin, ppmax);
  std::cout << "\nTask " << nr << ": Align model with " << model->GetKeypoints()->size() << " keypoints";//, "
            // << std::setprecision(2) << std::fixed
            // << "Pmin: (" << ppmin.x << ", " << ppmin.y << ", " << ppmin.z << ")  "
            // << "Pmax: (" << ppmax.x << ", " << ppmax.y << ", " << ppmax.z << ")\n"
            // << "  Scene size: " << scene->GetPointCloud()->size()
            // << " keypoints: " << scene->GetKeypoints()->size()
            // << " descriptors: " << scene->GetDescriptors()->size()
            // << "\n"
            // << "  Model size: " << model->GetPointCloud()->size()
            // << " keypoints: " << model->GetKeypoints()->size()
            // << " descriptors: " << model->GetDescriptors()->size()
            // << "\n";

  auto start = std::chrono::high_resolution_clock::now();

  std::shared_ptr<AlignmentMethod> aligner;

  switch(cfg.method) {
    case AlignmentMethodType::KdTreeSearch:
      aligner.reset(new KdTreeBasedAlignment(cfg.kdts));
      break;
    case AlignmentMethodType::SampleConsensus:
      aligner.reset(new SampleConsensusAlignment(cfg.sac));
      break;
    case AlignmentMethodType::GeometryConsistencyClustering:
      aligner.reset(new GeometryClusteringAlignment(cfg.gc));
      break;
    case AlignmentMethodType::Hough3DClustering:
      aligner.reset(new Hough3dClusteringAlignment(cfg.hough));
      break;
  }

  FeaturesMatching::Result result;

  if (aligner) {
    auto res = aligner->align(model, scene);
    result.transformation = res.transformation;
    result.processing_time_ms = res.processing_time_ms;
    result.fitness_score1 = res.fitness_score1;
    result.fitness_score2 = res.fitness_score2;
    result.fitness_score3 = res.fitness_score3;
    if (res.features_correspondences) {
      result.features_correspondences = res.features_correspondences;
    }
  }

  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
      std::cout <<"\nTask " << nr <<  ": aligned with fs2: " << result.fitness_score2 << " in " << diff.count() << " ms.";

  // if (cfg.show_visualizer || cfg.output_to_file) {
  //   MapsIntegratorVisualizer visualizer(
  //     { cfg.show_visualizer, cfg.output_to_file, cfg.output_dir + "feature_matching.png" });

  //   if (!result.features_correspondences) {
  //     result.features_correspondences = FindFeaturesCorrespondencesWithKdTree(
  //       model->GetDescriptors(), scene->GetDescriptors(), 1.0);
  //   }
  //   visualizer.VisualizeFeatureMatching(
  //     scene, model, result.transformation, result.features_correspondences);
  // }

  return result;
}

FeaturesMatching::ThreadResult FeaturesMatching::FindBestAlignment(
    const std::vector<FeaturesMatching::ThreadResult>& results) {

  float lowest_score = std::numeric_limits<float>::infinity();
  size_t best_result_index = 0;

  for (size_t i = 0; i < results.size(); ++i) {
    if (results[i].valid) {
      if (results[i].result.fitness_score2 < lowest_score) {
        lowest_score = results[i].result.fitness_score2;
        best_result_index = i;
      }
    }
  }
  return results[best_result_index];
}

}