/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

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

}
