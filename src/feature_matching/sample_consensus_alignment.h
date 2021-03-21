#pragma once

#include <Eigen/Dense>
#include <pcl/correspondence.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/transformation_validation_euclidean.h>

#include <feature_matching/alignment_method.h>
#include <validation.h>

namespace octomap_tools {

template <typename PointSource, typename PointTarget, typename FeatureT>
class SampleConsensusMod : public pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT> {
 public:
  using PointCloudSource = pcl::PointCloud<PointSource>;
  using Matrix4 = Eigen::Matrix<float, 4, 4>;
  typedef typename pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::HuberPenalty HuberPenalty;
  typedef typename pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::ErrorFunctor ErrorFunctor;

  bool findSimilarFeaturesMod (
      const pcl::PointCloud<FeatureT> &input_features, const std::vector<int> &sample_indices,
      std::vector<int> &corresponding_indices)
  {
    std::vector<float> distances(sample_indices.size(), 0); // only for debugging
    corresponding_indices.resize(sample_indices.size());

    std::cout << "\nIteration samples size: " << sample_indices.size() << "\n";

    for (size_t i = 0; i < sample_indices.size(); ++i) {
      std::vector<int> nn_indices(this->k_correspondences_);
      std::vector<float> nn_distances(this->k_correspondences_);

      // Find the k features nearest to input point
      this->feature_tree_->nearestKSearch(
        input_features, sample_indices[i], this->k_correspondences_, nn_indices, nn_distances);

      if (nn_distances[0] > 0.7)
        return false;

      std::cout << "\n";
      for (int x = 0; x < nn_distances.size(); x++) {
        std::cout << nn_distances[x] << " ";
      }
      int random_correspondence = 0;
      if (nn_distances[1] - nn_distances[0] > 0.1) {
        random_correspondence = 0;
      } else {
      // Select one at random and add it to corresponding_indices
        random_correspondence = this->getRandomIndex(this->k_correspondences_);
      }
      corresponding_indices[i] = nn_indices[random_correspondence];
      distances[i] = nn_distances[random_correspondence];
    }

    std::cout << "\n";
    for (int x = 0; x < distances.size(); x++)
      std::cout << distances[x] << " ";
    std::cout << "\n";
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
    std::vector<int> corresponding_indices (this->nr_samples_);
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
      bool found_features = this->findSimilarFeaturesMod(
        *this->input_features_, sample_indices, corresponding_indices);
      if (!found_features && iter != 0) {
        bad_features++;
        continue;
      }

      auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::high_resolution_clock::now() - start);
      PCL_DEBUG("Similar features found in %d ms\n", diff.count());
      start = std::chrono::high_resolution_clock::now();

      // Estimate the transform from the samples to their corresponding points
      this->transformation_estimation_->estimateRigidTransformation (
        *this->input_, sample_indices, *this->target_, corresponding_indices, this->transformation_);

      diff = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start);
      PCL_DEBUG("Rigid Transformation estimated in %d ms\n", diff.count());
      start = std::chrono::high_resolution_clock::now();

      // Tranform the data and compute the error
      transformPointCloud (*this->input_, input_transformed, this->transformation_);
      error = this->computeErrorMetric (input_transformed, static_cast<float> (this->corr_dist_threshold_));

      diff = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::high_resolution_clock::now() - start);
      PCL_DEBUG("Error metric computed in %d ms\n", diff.count());
      PCL_DEBUG("SAC iter: %d error: %.3f lowest_error: %.3f\n", iter, error, lowest_error);

      // If the new error is lower, update the final transformation
      if (iter == 0 || error < lowest_error) {
        lowest_error = error;
        this->final_transformation_ = this->transformation_;
        this->converged_ = true;
      }
    }

    PCL_DEBUG("Bad features: %d, bad_features/iterations: %.2f\n",
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

};

class SampleConsensusAlignment : public AlignmentMethod {
 public:

  struct Config {
    bool modified_version;
    float min_sample_distance;
    float max_correspondence_distance;
    int nr_iterations;
    float fitness_score_dist;
    unsigned samples_num;
    unsigned nn_for_each_sample_num;
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
    result.fitness_score = static_cast<float>(sac.getFitnessScore(cfg.fitness_score_dist));
    result.transformation = sac.getFinalTransformation();

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