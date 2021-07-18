#pragma once

#include <Eigen/Dense>
#include <pcl/correspondence.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/transformation_validation_euclidean.h>

#include <alignment/alignment_method.h>
#include <alignment/alignment_validator.h>

namespace octomap_tools {

template <typename PointSource, typename PointTarget, typename FeatureT>
class SampleConsensusMod : public pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT> {
 public:
  using PointCloudSource = pcl::PointCloud<PointSource>;
  using Matrix4 = Eigen::Matrix<float, 4, 4>;
  typedef typename pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::HuberPenalty HuberPenalty;
  typedef typename pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::ErrorFunctor ErrorFunctor;

  bool findSimilarFeaturesMod (
      const pcl::PointCloud<FeatureT>& input_features, std::vector<int>& sample_indices,
      std::vector<int> &corresponding_indices)
  {
    corresponding_indices.clear();
    std::vector<float> distances; // only for debugging
    std::vector<int> new_sample_indices;

    for (const auto& sample : sample_indices) {
      std::vector<int> nn_indices(this->k_correspondences_);
      std::vector<float> nn_distances(this->k_correspondences_);

      // Find the k features nearest to input point
      this->feature_tree_->nearestKSearch(
        input_features, sample, this->k_correspondences_, nn_indices, nn_distances);

      if (nn_distances[0] > feature_max_dist_) {
        // PCL_DEBUG("\nRejected");
        continue;
      }
      new_sample_indices.push_back(sample);

      // std::cout << "\n";
      // for (const auto el : nn_distances)
      //   std::cout << el << " ";

      int random_correspondence = 0;
      if (nn_distances[1] - nn_distances[0] > feature_max_dist_diff_) {
        random_correspondence = 0;
      } else {
      // Select one at random and add it to corresponding_indices
        random_correspondence = this->getRandomIndex(this->k_correspondences_);
      }
      corresponding_indices.push_back(nn_indices[random_correspondence]);
      distances.push_back(nn_distances[random_correspondence]);
    }

    if (corresponding_indices.size() < 3) {
      PCL_DEBUG("\nBad features\n");
      return false;
    }

    PCL_DEBUG("\nSelected: ");
    for (const auto el : distances)
      PCL_DEBUG("%.2f ", el);
    PCL_DEBUG("\n");

    sample_indices = new_sample_indices;
    return true;
  }

  float computeErrorMetric(const PointCloudSource &cloud, float) {
    std::vector<int> nn_index(1);
    std::vector<float> nn_distance(1);

    const ErrorFunctor& compute_error = *this->error_functor_;
    float error = 0;

    for (int i = 0; i < static_cast<int>(cloud.points.size()); ++i) {
      // Find the distance between point and its nearest neighbor
      this->tree_->nearestKSearch(cloud, i, 1, nn_index, nn_distance);

      // Compute the error
      error += compute_error(nn_distance[0]);
    }
    return error;
  }

  void computeTransformationMod(const Eigen::Matrix4f& guess) {
    if (!this->input_features_) {
      PCL_ERROR("[computeTransformationMod]: No source features were given\n");
      return;
    }
    if (!this->target_features_) {
      PCL_ERROR("[computeTransformationMod]: No target features were given\n");
      return;
    }

    if (this->input_->size () != this->input_features_->size ()) {
      PCL_ERROR("[computeTransformation] ");
      PCL_ERROR("The source points and source feature points should be the same size! Sizes: %ld vs %ld.\n",
                this->input_->size (), this->input_features_->size ());
      return;
    }

    if (this->target_->size () != this->target_features_->size ()) {
      PCL_ERROR("[computeTransformation] ");
      PCL_ERROR("The target points and target feature points should be the same size! Sizes: %ld vs %ld.\n",
                this->target_->size (), this->target_features_->size ());
      return;
    }

    if (!this->error_functor_) {
      this->error_functor_.reset(new HuberPenalty(static_cast<float>(this->corr_dist_threshold_)));
    }

    std::vector<int> sample_indices (this->nr_samples_);
    PointCloudSource input_transformed;
    float error, lowest_error (0);

    this->final_transformation_ = guess;
    int iter = 0;
    this->converged_ = false;
    if (!guess.isApprox(Eigen::Matrix4f::Identity (), 0.01f)) {
      // If guess is not the Identity matrix we check it.
      transformPointCloud(*this->input_, input_transformed, this->final_transformation_);
      lowest_error = this->computeErrorMetric(input_transformed, static_cast<float> (this->corr_dist_threshold_));
      iter = 1;
    }

    int bad_features = 0;

    for (; iter < this->max_iterations_; ++iter) {
      // Draw nr_samples_ random samples
      this->selectSamples(*this->input_, this->nr_samples_, this->min_sample_distance_, sample_indices);

      auto start = std::chrono::high_resolution_clock::now();

      // Find corresponding features in the target cloud
      std::vector<int> corresponding_indices;
      bool found_features = this->findSimilarFeaturesMod(
        *this->input_features_, sample_indices, corresponding_indices);
      if (!found_features && iter != 0) {
        bad_features++;
        continue;
      }

      // Estimate the transform from the samples to their corresponding points
      this->transformation_estimation_->estimateRigidTransformation (
        *this->input_, sample_indices, *this->target_, corresponding_indices, this->transformation_);

      // Tranform the data and compute the error
      transformPointCloud (*this->input_, input_transformed, this->transformation_);
      error = this->computeErrorMetric (input_transformed, static_cast<float> (this->corr_dist_threshold_));

      auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::high_resolution_clock::now() - start);
      PCL_DEBUG("SAC iter: %d time_ms: %d error: %.3f lowest_error: %.3f\n", iter, diff.count(), error, lowest_error);

      // If the new error is lower, update the final transformation
      if (iter == 0 || error < lowest_error) {
        lowest_error = error;
        this->final_transformation_ = this->transformation_;
        this->converged_ = true;
      }
    }

    PCL_DEBUG("\nBad features: %d, bad_features/iterations: %.2f\n",
      bad_features, static_cast<float>(bad_features)/this->max_iterations_);
  }

  inline void alignMod() {
    auto guess = Matrix4::Identity();

    if (!this->initCompute())
      return;

    // Perform the actual transformation computation
    this->converged_ = false;
    this->final_transformation_ = this->transformation_ = this->previous_transformation_ = Matrix4::Identity();

    computeTransformationMod(guess);

    this->deinitCompute();
  }

  pcl::CorrespondencesPtr getCorrespondences() {
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());

    // Transform the input dataset using the final transformation
    PointCloudSource input_transformed;
    transformPointCloud (*(this->input_), input_transformed, this->final_transformation_);

    for (size_t i = 0; i < input_transformed.points.size(); ++i) {
      std::vector<int> nn_indices(1);
      std::vector<float> nn_dists(1);

      // Find its nearest neighbor in the target
      int k = this->tree_->nearestKSearch(input_transformed.points[i], 1, nn_indices, nn_dists);
      if(k == 1 && nn_dists[0]) {
        pcl::Correspondence corr(nn_indices[0], static_cast<int>(i), nn_dists[0]);
        correspondences->push_back(corr);
      }
    }
    return correspondences;
  }

  void setFeatureMaxDist(float feature_max_dist) {
    feature_max_dist_ = feature_max_dist;
  }

  void setFeatureMaxDistDiff(float feature_max_dist_diff) {
    feature_max_dist_diff_ = feature_max_dist_diff;
  }

 private:
  float feature_max_dist_;
  float feature_max_dist_diff_;
};

class SampleConsensusAlignment : public FeatureAlignmentMethod {
 public:

  struct Config {
    float min_sample_distance = 0.2;
    float max_correspondence_distance = 100.0;
    int nr_iterations = 1000;
    float fitness_score_dist = 1.0;
    unsigned samples_num = 5;
    unsigned nn_for_each_sample_num = 10;
    bool modified_version = true; // use modified version of algorithm
    float mod_feature_max_dist = 0.7; // if dist between point and its NN is bigger then the correspondence is rejected
    float mod_feature_max_dist_diff = 0.1; // max dist between first and second NN
                                     // big value means that a feature is distinctive enough
  };

  SampleConsensusAlignment(const Config& cfg)
    : cfg(cfg) {
  }

  AlignmentMethod::Result align(const FeatureCloudPtr& model, const FeatureCloudPtr& scene) {

    SampleConsensusMod<Point, Point, FeatureCloud::DescriptorType> sac;
    sac.setMinSampleDistance(cfg.min_sample_distance);
    sac.setMaxCorrespondenceDistance(cfg.max_correspondence_distance);
    sac.setMaximumIterations(cfg.nr_iterations);
    sac.setNumberOfSamples(cfg.samples_num);
    sac.setCorrespondenceRandomness(cfg.nn_for_each_sample_num);
    sac.setFeatureMaxDist(cfg.mod_feature_max_dist);
    sac.setFeatureMaxDistDiff(cfg.mod_feature_max_dist_diff);

    sac.setInputSource(model->GetKeypoints());
    sac.setSourceFeatures(model->GetDescriptors());
    sac.setInputTarget(scene->GetKeypoints());
    sac.setTargetFeatures(scene->GetDescriptors());

    auto start = std::chrono::high_resolution_clock::now();

    if (cfg.modified_version) {
      sac.alignMod();
    }
    else {
      PointCloud dummy_output;
      sac.align(dummy_output);
    }

    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
    PCL_DEBUG("SAC Aligned in %d ms, converged: %d", diff.count(), sac.hasConverged());

    AlignmentMethod::Result result;
    result.transformation = sac.getFinalTransformation();
    result.processing_time_ms = diff.count();

    AlignmentValidator<Point> validator;
    validator.setCorrespondences(sac.getCorrespondences());
    result.fitness_score1 = validator.calcFitnessScore1();
    result.fitness_score2 = validator.calcFitnessScore2();
    result.fitness_score3 = validator.calcFitnessScore3();

    return result;
  }

 private:
  Config cfg;
};

} // namespace octomap_tools