#include <octomap_tools/features_matching.h>

#include <chrono>

#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/features/board.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#include <octomap_tools/thread_pool.h>

typedef pcl::ReferenceFrame RFType;
typedef pcl::Normal NormalType;

namespace octomap_tools {

FeaturesMatching::Result FeaturesMatching::Align(int nr, FeaturesMatching::Config& cfg,
                                                 const FeatureCloudPtr model, const FeatureCloudPtr scene) {
  using DescriptorType = FeatureCloud::DescriptorType;

  auto start = std::chrono::high_resolution_clock::now();
  pcl::SampleConsensusInitialAlignment<Point, Point, DescriptorType> sac_ia_;
  sac_ia_.setMinSampleDistance(cfg.min_sample_distance);
  sac_ia_.setMaxCorrespondenceDistance(cfg.max_correspondence_distance);
  sac_ia_.setMaximumIterations(cfg.nr_iterations);
  sac_ia_.setNumberOfSamples(3);
  sac_ia_.setInputSource(model->getKeypoints());
  sac_ia_.setSourceFeatures (model->getDescriptors());
  sac_ia_.setInputTarget (scene->getKeypoints());
  sac_ia_.setTargetFeatures (scene->getDescriptors());
  PointCloud dummy_output;
  sac_ia_.align(dummy_output);
  FeaturesMatching::Result result;
  result.fitness_score = (float) sac_ia_.getFitnessScore(cfg.fitness_score_dist);
  result.transformation = sac_ia_.getFinalTransformation();

  bool show_keypoints_ (true);
  bool show_correspondences_ (true);
  bool use_cloud_resolution_ (true);
  bool use_hough_ (true);
  float model_ss_ (0.01f);
  float scene_ss_ (0.03f);
  float rf_rad_ (0.015f);
  float descr_rad_ (0.02f);
  float cg_size_ (0.01f);
  float cg_thresh_ (5.0f);

  auto model_descriptors = model->getDescriptors();
  auto scene_descriptors = scene->getDescriptors();
  //
  //  Find Model-Scene Correspondences with KdTree
  //
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  for (size_t i = 0; i < scene_descriptors->size (); ++i) {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
    }
  }
  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

  //
  //  Actual Clustering
  //
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;

  //  Using Hough3D
  if (false) {
    //
    //  Compute (Keypoints) Reference Frames only for Hough
    //
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<Point, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);

    rf_est.setInputCloud (model->getKeypoints());
    rf_est.setInputNormals (model->getSurfaceNormals());
    rf_est.setSearchSurface (model->getPointCloud());
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene->getKeypoints());
    rf_est.setInputNormals (scene->getSurfaceNormals());
    rf_est.setSearchSurface (scene->getPointCloud());
    rf_est.compute (*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<Point, Point, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model->getKeypoints());
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene->getKeypoints());
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);
  }

  if (true) { // Using GeometricConsistency
    pcl::GeometricConsistencyGrouping<Point, Point> gc_clusterer;
    gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud (model->getKeypoints());
    gc_clusterer.setSceneCloud (scene->getKeypoints());
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }

  //
  //  Output results
  //
  std::cout << "Model instances found: " << rototranslations.size () << std::endl;
  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }

  //
  //  Visualization
  //
  pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
  viewer.setBackgroundColor(255, 255, 255);
  viewer.addCoordinateSystem(1.0);
  viewer.addPointCloud (scene->getPointCloud(), "scene_cloud");

  pcl::PointCloud<Point>::Ptr off_scene_model (new pcl::PointCloud<Point> ());
  pcl::PointCloud<Point>::Ptr off_scene_model_keypoints (new pcl::PointCloud<Point> ());

  if (show_correspondences_ || show_keypoints_)
  {
    //  We are translating the model so that it doesn't end in the middle of the scene representation
    pcl::transformPointCloud (*model->getPointCloud(), *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*model->getKeypoints(), *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<Point> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
  }

  if (show_keypoints_)
  {
    pcl::visualization::PointCloudColorHandlerCustom<Point> scene_keypoints_color_handler (scene->getKeypoints(), 0, 0, 255);
    viewer.addPointCloud (scene->getKeypoints(), scene_keypoints_color_handler, "scene_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<Point> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
  }

  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    pcl::PointCloud<Point>::Ptr rotated_model (new pcl::PointCloud<Point> ());
    pcl::transformPointCloud (*model->getPointCloud(), *rotated_model, rototranslations[i]);

    std::stringstream ss_cloud;
    ss_cloud << "instance" << i;

    pcl::visualization::PointCloudColorHandlerCustom<Point> rotated_model_color_handler (rotated_model, 255, 0, 0);
    viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

    if (show_correspondences_)
    {
      for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
      {
        std::stringstream ss_line;
        ss_line << "correspondence_line" << i << "_" << j;
        Point& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
        Point& scene_point = scene->getKeypoints()->at (clustered_corrs[i][j].index_match);

        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer.addLine<Point, Point> (model_point, scene_point, 0, 255, 0, ss_line.str ());
      }
    }
  }

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  Point ppmin, ppmax;
  pcl::getMinMax3D(*(model->getPointCloud()), ppmin, ppmax);
  std::cout << "T" << nr << ": Align template with " << model->getKeypoints()->size() << " keypoints, ";
  std::cout << std::setprecision(2) << std::fixed;
  std::cout << "Pmin: (" << ppmin.x << ", " << ppmin.y << ", " << ppmin.z << ")  ";
  std::cout << "Pmax: (" << ppmax.x << ", " << ppmax.y << ", " << ppmax.z << ")\n";


  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
  std::cout << "T" << nr <<  ": Clouds aligned with score: " << result.fitness_score << " in: " << diff.count() << " ms." << std::endl;
  return result;
}

FeaturesMatching::Result align(int nr, FeaturesMatching::Config& cfg,
                               const FeatureCloudPtr model, const FeatureCloudPtr scene) {
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

  std::cout << "\nStarting features matching alignment. There are " << blocks_.size() << " blocks.\n";

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
