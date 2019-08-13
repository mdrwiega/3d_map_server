#include <octomap_tools/features_matching.h>

#include <chrono>

#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>

#include <octomap_tools/thread_pool.h>

namespace octomap_tools {

FeaturesMatching::Result align(int nr, FeaturesMatching::Config& cfg, const FeatureCloudPtr model, const FeatureCloudPtr scene) {
  using DescriptorType = FeatureCloud::DescriptorType;

  auto start = std::chrono::high_resolution_clock::now();
  pcl::SampleConsensusInitialAlignment<Point, Point, DescriptorType> sac_ia_;
  sac_ia_.setMinSampleDistance(cfg.min_sample_distance);
  sac_ia_.setMaxCorrespondenceDistance(cfg.max_correspondence_distance);
  sac_ia_.setMaximumIterations(cfg.nr_iterations);
  sac_ia_.setNumberOfSamples(3);

  Point ppmin, ppmax;
  pcl::getMinMax3D(*(model->getPointCloud()), ppmin, ppmax);
  std::cout << "T" << nr << ": Align template with " << model->getKeypoints()->size() << " keypoints, ";
  std::cout << std::setprecision(2) << std::fixed;
  std::cout << "Pmin: (" << ppmin.x << ", " << ppmin.y << ", " << ppmin.z << ")  ";
  std::cout << "Pmax: (" << ppmax.x << ", " << ppmax.y << ", " << ppmax.z << ")\n";

  sac_ia_.setInputSource(model->getKeypoints());
  sac_ia_.setSourceFeatures (model->getDescriptors());
  sac_ia_.setInputTarget (scene->getKeypoints());
  sac_ia_.setTargetFeatures (scene->getDescriptors());

  PointCloud dummy_output;
  sac_ia_.align(dummy_output);

  FeaturesMatching::Result result;
  result.fitness_score = (float) sac_ia_.getFitnessScore(cfg.fitness_score_dist);
  result.transformation = sac_ia_.getFinalTransformation();

  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
  std::cout << "T" << nr <<  ": Clouds aligned with score: " << result.fitness_score << " in: " << diff.count() << " ms." << std::endl;
  return result;
}

FeaturesMatching::ThreadResult threadFcn(int nr, PointCloudPtr& full_model, std::vector<Rectangle> blocks_,
                                         const FeatureCloudPtr& scene, FeaturesMatching::Config& cfg) {
  Point pmin, pmax;
  pcl::getMinMax3D(*full_model, pmin, pmax);
  Eigen::Vector4f model_min = {blocks_[nr].min(0), blocks_[nr].min(1), pmin.z, 1};
  Eigen::Vector4f model_max = {blocks_[nr].max(0), blocks_[nr].max(1), pmax.z, 1};

  pcl::CropBox<Point> boxFilter;
  boxFilter.setInputCloud(full_model);
  boxFilter.setMin(model_min);
  boxFilter.setMax(model_max);
  PointCloudPtr filtered_model (new PointCloud);
  boxFilter.filter(*filtered_model);

  if (filtered_model->size() < cfg.model_size_thresh_) {
//    std::cout << "T" << nr << ": Model is too small (size: " << filtered_model->size() << ")\n";
    return FeaturesMatching::ThreadResult ();
  }

  auto model = std::make_shared<FeatureCloud>(filtered_model, cfg.feature_cloud);
  model->downsampleAndExtractKeypoints();
  if (model->getKeypoints()->size() < cfg.keypoints_thresh_) {
//    std::cout << "T" << nr << ": Not enough keypoints: " << model->getKeypoints()->size() << "\n";
    return FeaturesMatching::ThreadResult ();
  }
  model->computeSurfaceNormals();
  model->computeDescriptors();

  auto result = align(nr, cfg, model, scene);
  return FeaturesMatching::ThreadResult {true, nr, result.fitness_score, result.transformation};
}

FeaturesMatching::Result FeaturesMatching::initialAlignment(PointCloud& best_model) {
  auto start = std::chrono::high_resolution_clock::now();

  auto scene = std::make_shared<FeatureCloud>(scene_cloud_, cfg_.feature_cloud);
  scene->processInput();
  std::vector<Rectangle> blocks_ = divideFullModelIntoBlocks(cfg_.cell_size_x, cfg_.cell_size_y);

  std::cout << "Starting features matching alignment. There are " << blocks_.size() << " blocks.\n";

  std::shared_ptr<thread_pool::ThreadPool> thread_pool = thread_pool::createThreadPool();
  std::vector<std::future<FeaturesMatching::ThreadResult>> thread_futures;
  for (std::uint32_t i = 0; i < blocks_.size(); ++i) {
    thread_futures.emplace_back(thread_pool->submit(
        threadFcn, i, std::ref(model_cloud_), std::ref(blocks_), std::ref(scene), std::ref(cfg_)));
  }

  std::vector<FeaturesMatching::ThreadResult> results;
  for (auto& it: thread_futures) {
    it.wait();
    results.emplace_back(it.get());
  }

  ThreadResult ia_result = findBestAlignment(results);

  // Get best model
  Point pmin, pmax;
  pcl::getMinMax3D(*model_cloud_, pmin, pmax);
  Eigen::Vector4f model_min = {blocks_[ia_result.thread_num].min(0), blocks_[ia_result.thread_num].min(1), pmin.z, 1};
  Eigen::Vector4f model_max = {blocks_[ia_result.thread_num].max(0), blocks_[ia_result.thread_num].max(1), pmax.z, 1};
  pcl::CropBox<Point> boxFilter;
  boxFilter.setInputCloud(model_cloud_);
  boxFilter.setMin(model_min);
  boxFilter.setMax(model_max);
  boxFilter.filter(best_model);

  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);

  return FeaturesMatching::Result {ia_result.fitness_score, ia_result.transformation, static_cast<float>(diff.count())};
}

std::vector<Rectangle> FeaturesMatching::divideFullModelIntoBlocks(float block_size_x, float block_size_y) {
  if (model_cloud_->size() <= 0) {
    throw std::runtime_error(std::string("Full model is not set!"));
  }

  Eigen::Vector4f map_min, map_max;
  pcl::getMinMax3D (*model_cloud_, map_min, map_max);
  Eigen::Vector2f rectangle_min (map_min.x(), map_min.y());
  Eigen::Vector2f rectangle_max (map_max.x(), map_max.y());
  Eigen::Vector2f step_xy (block_size_x, block_size_y);

  //  std::cout << "Generation of cells for rectangle "
  //      << "min: (" << rectangle_min(0) << ", " << rectangle_min(1) << ")  "
  //      << "max: (" << rectangle_max(0) << ", " << rectangle_max(1) << ")\n";
  return generateBlocksInSpiralOrder(rectangle_min, rectangle_max, step_xy);
}

FeaturesMatching::ThreadResult FeaturesMatching::findBestAlignment(const std::vector<FeaturesMatching::ThreadResult>& results) {
  int block_nr = 0;
  float lowest_score = std::numeric_limits<float>::infinity();
  Eigen::Matrix4f transformation;
  for (auto it : results) {
    if (it.valid) {
      if (it.fitness_score < lowest_score) {
        lowest_score = it.fitness_score;
        block_nr = static_cast<int>(it.thread_num);
        transformation = it.transformation;
      }
    }
  }
  return ThreadResult{true, block_nr, lowest_score, transformation};
}

}
