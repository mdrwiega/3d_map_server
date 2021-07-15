#pragma once

#include <pcl/point_types.h>

#include <octomap_tools/types.h>

namespace octomap_tools {

class NdtAlignment {
public:

  struct Config {
  };

  struct Result {
  };

  NdtAlignment(const Config& config, PointCloudPtr& scene, PointCloudPtr& model)
  {

  }

  Result Align()
  {

    return {};
  }

 private:
  PointCloudPtr scene_cloud_;
  PointCloudPtr model_cloud_;
  Config cfg_;
};

}