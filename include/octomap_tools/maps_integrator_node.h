#pragma once

#include <string>

#include <ros/node_handle.h>

#include <octomap_tools/maps_integrator.h>

namespace octomap_tools {

class MapsIntegratorNode {
public:
  MapsIntegrator::Config GetConfigFromRosParams();

  /**
   * Integrate two octomaps from files
   *
   * @param map1_filepath is a file path to the first map
   * @param map2_filepath is a file path to the second map
   */
  void IntegrateMaps(const std::string& map1_filepath, const std::string& map2_filepath);

private:
  ros::NodeHandle nh_;

};

} // namespace octomap_tools
