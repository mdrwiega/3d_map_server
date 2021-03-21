#pragma once

#include <Eigen/Dense>
#include <pcl/correspondence.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/transformation_validation_euclidean.h>

#include <feature_matching/alignment_method.h>
#include <validation.h>

namespace octomap_tools {

template <typename PointSource, typename PointTarget, typename FeatureT>
class SampleConsensusInitialAlignmentMod : public
  pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT> {
 public:
  using PointCloudSource = pcl::PointCloud<PointSource>;
  using Matrix4 = Eigen::Matrix<float, 4, 4>;
  typedef typename pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::HuberPenalty HuberPenalty;
  typedef typename pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::TruncatedError TruncatedError;
  typedef typename pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::ErrorFunctor ErrorFunctor;

bool findSimilarFeaturesMod (
    const pcl::PointCloud<FeatureT> &input_features, const std::vector<int> &sample_indices,
    std::vector<int> &corresponding_indices)
{
  this->k_correspondences_ = 10;

  std::vector<int> nn_indices (this->k_correspondences_);
  std::vector<float> nn_distances (this->k_correspondences_);

  std::vector<float> distances (sample_indices.size(), 0);

  corresponding_indices.resize (sample_indices.size ());
  for (size_t i = 0; i < sample_indices.size (); ++i)
  {
    // Find the k features nearest to input_features.points[sample_indices[i]]
    this->feature_tree_->nearestKSearch (input_features, sample_indices[i], this->k_correspondences_, nn_indices, nn_distances);

    if (nn_distances[0] > 0.8)
      return false;

  // std::cout << "\n";
    // for (int x = 0; x < nn_distances.size(); x++) {
    //   std::cout << nn_distances[x] << " ";
    // }
    int random_correspondence = 0;
    if (nn_distances[1] - nn_distances[0] > 0.03) {
      random_correspondence = 0;
    } else {
    // Select one at random and add it to corresponding_indices
      random_correspondence = this->getRandomIndex (this->k_correspondences_);
    }
    corresponding_indices[i] = nn_indices[random_correspondence];
    distances[i] = nn_distances[random_correspondence];
  }

  // std::cout << "\n";
  // for (int x = 0; x < distances.size(); x++)
  //   std::cout << distances[x] << " ";
  // std::cout << "\n";
  return true;
}

float computeErrorMetric (const PointCloudSource &cloud, float)
{
  std::vector<int> nn_index (1);
  std::vector<float> nn_distance (1);

  const ErrorFunctor & compute_error = *this->error_functor_;
  float error = 0;

  for (int i = 0; i < static_cast<int> (cloud.points.size ()); ++i)
  {
    // Find the distance between cloud.points[i] and its nearest neighbor in the target point cloud
    this->tree_->nearestKSearch (cloud, i, 1, nn_index, nn_distance);

    // Compute the error
    error += compute_error (nn_distance[0]);
  }
  return (error);
}

void computeTransformationMod (PointCloudSource &output, const Eigen::Matrix4f& guess)
{
  // Some sanity checks first
  if (!this->input_features_)
  {
    PCL_ERROR ("[pcl::%s::computeTransformation] ", this->getClassName ().c_str ());
    PCL_ERROR ("No source features were given! Call setSourceFeatures before aligning.\n");
    return;
  }
  if (!this->target_features_)
  {
    PCL_ERROR ("[pcl::%s::computeTransformation] ", this->getClassName ().c_str ());
    PCL_ERROR ("No target features were given! Call setTargetFeatures before aligning.\n");
    return;
  }

  if (this->input_->size () != this->input_features_->size ())
  {
    PCL_ERROR ("[pcl::%s::computeTransformation] ", this->getClassName ().c_str ());
    PCL_ERROR ("The source points and source feature points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n",
               this->input_->size (), this->input_features_->size ());
    return;
  }

  if (this->target_->size () != this->target_features_->size ())
  {
    PCL_ERROR ("[pcl::%s::computeTransformation] ", this->getClassName ().c_str ());
    PCL_ERROR ("The target points and target feature points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n",
               this->target_->size (), this->target_features_->size ());
    return;
  }

  if (!this->error_functor_)
    this->error_functor_.reset (new HuberPenalty (static_cast<float> (this->corr_dist_threshold_)));


  std::vector<int> sample_indices (this->nr_samples_);
  std::vector<int> corresponding_indices (this->nr_samples_);
  PointCloudSource input_transformed;
  float error, lowest_error (0);

  this->final_transformation_ = guess;
  int i_iter = 0;
  this->converged_ = false;
  if (!guess.isApprox (Eigen::Matrix4f::Identity (), 0.01f)) 
  {
    // If guess is not the Identity matrix we check it.
    transformPointCloud (*this->input_, input_transformed, this->final_transformation_);
    lowest_error = this->computeErrorMetric (input_transformed, static_cast<float> (this->corr_dist_threshold_));
    i_iter = 1;
  }

  std::cout << "\n";

  auto start1 = std::chrono::high_resolution_clock::now();

  int bad_features = 0;

  for (; i_iter < this->max_iterations_; ++i_iter)
  {
    // Draw nr_samples_ random samples
    this->selectSamples (*this->input_, this->nr_samples_, this->min_sample_distance_, sample_indices);

  //  auto start = std::chrono::high_resolution_clock::now();

    // Find corresponding features in the target cloud
    bool found_features = this->findSimilarFeaturesMod (*this->input_features_, sample_indices, corresponding_indices);
    if (!found_features && i_iter != 0) {
      bad_features++;
      continue;
    }

    // auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
    //     std::chrono::high_resolution_clock::now() - start);
    // std::cout << "Similar features found in " << diff.count() << " ms" << std::endl;
    // start = std::chrono::high_resolution_clock::now();

    // Estimate the transform from the samples to their corresponding points
    this->transformation_estimation_->estimateRigidTransformation (
      *this->input_, sample_indices, *this->target_, corresponding_indices, this->transformation_);

    // diff = std::chrono::duration_cast<std::chrono::milliseconds>(
    //     std::chrono::high_resolution_clock::now() - start);
    // std::cout << "Rigid Transformation estimated in " << diff.count() << " ms" << std::endl;
    // start = std::chrono::high_resolution_clock::now();

    // Tranform the data and compute the error
    transformPointCloud (*this->input_, input_transformed, this->transformation_);
    error = this->computeErrorMetric (input_transformed, static_cast<float> (this->corr_dist_threshold_));

    // diff = std::chrono::duration_cast<std::chrono::milliseconds>(
    //     std::chrono::high_resolution_clock::now() - start);
    // std::cout << "Error metric computed in " << diff.count() << " ms" << std::endl;

    // std::cout << "SAC iter: " << i_iter << " error: " << error << " lowest_error: " << lowest_error << "\n\n";

    // If the new error is lower, update the final transformation
    if (i_iter == 0 || error < lowest_error)
    {
      lowest_error = error;
      this->final_transformation_ = this->transformation_;
      this->converged_=true;
    }
  }

  auto diff1 = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start1);
  std::cout << "\nAll Iterations takes " << diff1.count() << " ms" << std::endl;
  std::cout << "Iterations " << this->max_iterations_ << std::endl;
  std::cout << "Bad features " << bad_features << std::endl;
  std::cout << "Bad features/iterations " << (float)bad_features/this->max_iterations_  << std::endl;

  // Apply the final transformation
  transformPointCloud (*this->input_, output, this->final_transformation_);
}


  inline void alignMod(PointCloudSource &output) {
    auto guess = Matrix4::Identity();

    if (!this->initCompute ())
      return;

    // Resize the output dataset
    if (output.points.size () != this->indices_->size ())
      output.points.resize (this->indices_->size ());
    // Copy the header
    output.header   = this->input_->header;
    // Check if the output will be computed for all points or only a subset
    if (this->indices_->size () != this->input_->points.size ())
    {
      output.width    = static_cast<uint32_t> (this->indices_->size ());
      output.height   = 1;
    }
    else
    {
      output.width    = static_cast<uint32_t> (this->input_->width);
      output.height   = this->input_->height;
    }
    output.is_dense = this->input_->is_dense;

    // Copy the point data to output
    for (size_t i = 0; i < this->indices_->size (); ++i)
      output.points[i] = this->input_->points[(*this->indices_)[i]];

    // Set the internal point representation of choice unless otherwise noted
    // if (this->point_representation_ && !this->force_no_recompute_)
    //   this->tree_->setPointRepresentation (this->point_representation_);

    // Perform the actual transformation computation
    this->converged_ = false;
    this->final_transformation_ = this->transformation_ = this->previous_transformation_ = Matrix4::Identity ();

    // Right before we estimate the transformation, we set all the point.data[3] values to 1 to aid the rigid 
    // transformation
    for (size_t i = 0; i < this->indices_->size (); ++i)
      output.points[i].data[3] = 1.0;

    computeTransformationMod (output, guess);

    this->deinitCompute ();
  }

  pcl::CorrespondencesPtr getCorrespondences() {
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());

    // Transform the input dataset using the final transformation
    PointCloudSource input_transformed;
    transformPointCloud (*(this->input_), input_transformed, this->final_transformation_);

    for (size_t i = 0; i < input_transformed.points.size(); ++i)
    {
      std::vector<int> nn_indices(1);
      std::vector<float> nn_dists(1);

      // Find its nearest neighbor in the target
      int k = this->tree_->nearestKSearch(input_transformed.points[i], 1, nn_indices, nn_dists);
      if(k >= 1 && nn_dists[0] <= 0.25) {
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
    float min_sample_distance;
    float max_correspondence_distance;
    int nr_iterations;
    float fitness_score_dist;
  };

  SampleConsensusAlignment(const Config& cfg)
    : cfg(cfg) {
  }

  AlignmentMethod::Result align(const FeatureCloudPtr& model, const FeatureCloudPtr& scene) {
    pcl::CorrespondencesPtr features_correspondences(new pcl::Correspondences);

    // Align feature clouds with Sample Consensus Initial Alignment
    SampleConsensusInitialAlignmentMod<Point, Point, FeatureCloud::DescriptorType> sac;
    sac.setMinSampleDistance(cfg.min_sample_distance);
    sac.setMaxCorrespondenceDistance(cfg.max_correspondence_distance);
    sac.setMaximumIterations(cfg.nr_iterations);
    sac.setNumberOfSamples(3);

    sac.setInputSource(model->GetKeypoints());
    sac.setSourceFeatures(model->GetDescriptors());
    sac.setInputTarget(scene->GetKeypoints());
    sac.setTargetFeatures(scene->GetDescriptors());

    PointCloud dummy_output;
    sac.alignMod(dummy_output);

    double fs = sac.getFitnessScore(cfg.fitness_score_dist);

    std::cout << "Converged?: " << sac.hasConverged()
      << std::setprecision(6) << std::fixed
      << "\nStandard fitness score: " << fs << "\n";

    pcl::registration::TransformationValidationEuclidean<Point, Point> validator1;
    validator1.setMaxRange(0.10);

    double score = validator1.validateTransformation(model->GetPointCloud(), scene->GetPointCloud(),
                                          sac.getFinalTransformation());

    std::cout << "Score: " << score;
    AlignmentMethod::Result result;
    result.fitness_score = static_cast<float>(sac.getFitnessScore(cfg.fitness_score_dist));
    result.transformation = sac.getFinalTransformation();

    // features_correspondences = FindFeaturesCorrespondencesWithKdTree(model->GetDescriptors(), scene->GetDescriptors());

    AlignmentValidator<Point> validator;
    validator.calculateCorrespondences(model->GetPointCloud(), scene->GetPointCloud(), result.transformation);

    std::cout << "\nFitnessScore  : " << result.fitness_score << "\n";
    std::cout << "\nFitnessScore1 : " << validator.calcFitnessScore1() << "\n";
    std::cout << "\nFitnessScore2 : " << validator.calcFitnessScore2() << "\n";
    std::cout << "\nFitnessScore3 : " << validator.calcFitnessScore3() << "\n";

    return result;
  }

 private:
  Config cfg;
};

} // namespace octomap_tools