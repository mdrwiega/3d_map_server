#pragma once

#include <octomap_tools/maps_integrator.h>

#include <string>

#include <ros/node_handle.h>

#include <octomap_tools/maps_integrator.h>

namespace octomap_tools {

class MapsIntegratorNode {
 private:
  ros::NodeHandle nh_;

 public:
  MapsIntegrator::Config GetConfigFromRosParams();

  void IntegrateMaps(const std::string& map1_filepath, const std::string& map2_filepath);

};

} // namespace octomap_tools
