#pragma once

#include <octomap_tools/maps_integrator.h>

namespace octomap_tools {

class MapsIntegratorRos {
 public:

  MapsIntegratorRos() = default;

  FeaturesMatching::Config GetConfigurationFromRosParams() {
  FeaturesMatching::Config config;

    return config;
  }
};

} // namespace octomap_tools
