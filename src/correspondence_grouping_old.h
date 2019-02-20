void FeatureMatching::HoughClustering(
    const CorrespondenceGroupingConfig& config,
    const pcl::CorrespondencesPtr& model_scene_corrs,
    FeatureCloudPtr& model,
    FeatureCloudPtr& scene,
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& rototranslations,
    std::vector<pcl::Correspondences>& clustered_corrs) {
  //  Compute (Keypoints) Reference Frames only for Hough
  pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
  pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());
  pcl::BOARDLocalReferenceFrameEstimation<Point, NormalType, RFType> rf_est;
  rf_est.setFindHoles(true);
  rf_est.setRadiusSearch(config.rf_rad_);
  rf_est.setInputCloud(model->getKeypoints());
  rf_est.setInputNormals(model->getSurfaceNormals());
  rf_est.setSearchSurface(model->getPointCloud());
  rf_est.compute(*model_rf);
  rf_est.setInputCloud(scene->getKeypoints());
  rf_est.setInputNormals(scene->getSurfaceNormals());
  rf_est.setSearchSurface(scene->getPointCloud());
  rf_est.compute(*scene_rf);
  //  Clustering
  pcl::Hough3DGrouping<Point, Point, RFType, RFType> clusterer;
  clusterer.setHoughBinSize(config.cg_size_);
  clusterer.setHoughThreshold(config.cg_thresh_);
  clusterer.setUseInterpolation(true);
  clusterer.setUseDistanceWeight(false);
  clusterer.setInputCloud(model->getKeypoints());
  clusterer.setInputRf(model_rf);
  clusterer.setSceneCloud(scene->getKeypoints());
  clusterer.setSceneRf(scene_rf);
  clusterer.setModelSceneCorrespondences(model_scene_corrs);
  //clusterer.cluster (clustered_corrs);
  clusterer.recognize(rototranslations, clustered_corrs);
}

void FeatureMatching::GeometryConsistencyGrouping(
    const CorrespondenceGroupingConfig& config,
    const pcl::CorrespondencesPtr& model_scene_corrs,
    FeatureCloudPtr& model,
    FeatureCloudPtr& scene,
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& rototranslations,
    std::vector<pcl::Correspondences>& clustered_corrs) {
  pcl::GeometricConsistencyGrouping<Point, Point> gc_clusterer;
  gc_clusterer.setGCSize(config.cg_size_);
  gc_clusterer.setGCThreshold(config.cg_thresh_);
  gc_clusterer.setInputCloud(model->getKeypoints());
  gc_clusterer.setSceneCloud(scene->getKeypoints());
  gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);
  //gc_clusterer.cluster (clustered_corrs);
  gc_clusterer.recognize(rototranslations, clustered_corrs);
}

void FeatureMatching::FindCorrespondencesWithKdTree(const pcl::CorrespondencesPtr& model_scene_corrs,
                         FeatureCloudPtr& model, FeatureCloudPtr& scene) {
  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud(model->getDescriptors());
  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  for (size_t i = 0; i < scene->getDescriptors()->size(); ++i) {
    auto& scene_descriptor = scene->getDescriptors()->at(i);
    std::vector<int> neigh_indices(1);
    std::vector<float> neigh_sqr_dists(1);
//    if (!std::isfinite(scene_descriptor.descriptor[0])) {
//      //skipping NaNs
//      continue;
//    }
    int found_neighs = match_search.nearestKSearch(scene_descriptor, 1,
                                                   neigh_indices,
                                                   neigh_sqr_dists);
    if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)  //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
      pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i),
                               neigh_sqr_dists[0]);
      model_scene_corrs->push_back(corr);
    }
  }
}


void FeatureMatching::calc(FeatureCloudPtr& scene, FeatureCloudPtr& model,
          const CorrespondenceGroupingConfig& config, CGResultsSet& results) {

    auto start = std::chrono::high_resolution_clock::now();
    // Find the best template alignment
    TemplateAlignment::Result best_alignment;
    template_align_.findBestAlignment (best_alignment);

    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    std::cout << "Clouds aligned in: " << diff.count() << " ms." << std::endl;

    // Print the alignment fitness score (values less than 0.00002 are good)
    printf ("Best fitness score: %f\n", best_alignment.fitness_score);



    Point pmin, pmax;
    pcl::getMinMax3D(*model->getPointCloud(), pmin, pmax);
    CGResultEntry result;
    result.model_max = pmax;
    result.model_min = pmin;
    result.transformation = best_alignment.final_transformation;
    results.AppendResultEntry(result);


//  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences());
//  FindCorrespondencesWithKdTree(model_scene_corrs, model, scene);
//
//  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
//  if (model_scene_corrs->size() < config.correspondences_thresh_) {
//    std::cout << "Correspondences below threshold. Exit." << std::endl;
//    return;
//  }
//  //  Actual Clustering
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;
  std::vector<pcl::Correspondences> clustered_corrs;
//
//  if (config.use_hough_) {
//    HoughClustering(config, model_scene_corrs, model, scene, transformations, clustered_corrs);
//  } else {
//    GeometryConsistencyGrouping(config, model_scene_corrs, model, scene,
//                                transformations, clustered_corrs);
//  }
//
//  std::cout << "Model instances found: " << transformations.size () << std::endl;
//  if (transformations.size() == 0) {
//    std::cout << "No model instances. Exit" << std::endl;
//    return;
//  }
//
//  Point pmin, pmax;
//  pcl::getMinMax3D(*model->getPointCloud(), pmin, pmax);
//  // Copy results to data structure
//  for (size_t i = 0; i < transformations.size (); ++i) {
//    CGResultEntry result;
//    result.model_max = pmax;
//    result.model_min = pmin;
//    result.transformation = transformations[i];
//    result.correspondences = clustered_corrs[i].size();
//    results.AppendResultEntry(result);
//  }
  transformations.push_back(best_alignment.final_transformation);

  if (config.show_visualization_) {
    FeatureMatchingVisualizer visualizer;
    visualizer.visualize(config.show_keypoints_, config.show_correspondences_,
                                         transformations, clustered_corrs, scene, model, full_model_,
                                         spiral_blocks_);
  }
}
