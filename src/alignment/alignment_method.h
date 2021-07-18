#pragma once

#include <string>
#include <limits>

#include <Eigen/Dense>
#include <pcl/correspondence.h>

#include <common/math.h>
#include <alignment/feature_matching/feature_cloud.h>

namespace octomap_tools {

class AlignmentMethod {
public:
  struct Result {
    float fitness_score1 = std::numeric_limits<float>::max();
    float fitness_score2 = std::numeric_limits<float>::max();
    float fitness_score3 = std::numeric_limits<float>::max();
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    float processing_time_ms = 0;
    pcl::CorrespondencesPtr correspondences;
    pcl::CorrespondencesPtr features_correspondences;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    std::string ToString(std::string header = "") {
      std::stringstream ss;
      if (!header.empty()) {
        ss << "\n" << header << "\n";
      }
      ss << "  fitness_score1: " << fitness_score1 << "\n";
      ss << "  fitness_score2: " << fitness_score2 << "\n";
      ss << "  fitness_score3: " << fitness_score3 << "\n";
      ss << "  time_ms: " << processing_time_ms << "\n";
      ss << "  transformation:\n" << transfMatrixToXyzRpyString(transformation, "    ");
      return ss.str();
    }
  };

  virtual ~AlignmentMethod() = default;

  virtual Result Align() = 0;
};

class GlobalAlignment {
public:
  enum class Method { FeatureMatching };

};

class LocalAlignment {
public:
  enum class Method { ICP, NDT };
};

} // namespace octomap_tools