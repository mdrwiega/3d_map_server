#include <ros/init.h>

#include <octomap_tools/maps_integrator_node.h>
#include <common/utils.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "maps_integrator");

  const std::string map1_filepath = argv[1];
  const std::string map2_filepath = argv[2];

  if (!octomap_tools::exists(map1_filepath)) {
    ROS_ERROR_STREAM("File doesn't exist in path: " << map1_filepath << std::endl);
    exit(1);
  }
  if (!octomap_tools::exists(map2_filepath)) {
    ROS_ERROR_STREAM("File doesn't exist in path: " << map2_filepath << std::endl);
    exit(1);
  }

  ROS_INFO_STREAM("Maps Integrator node started");

  try {
    octomap_tools::MapsIntegratorNode maps_integrator;
    maps_integrator.IntegrateMaps(map1_filepath, map2_filepath);
  } catch (const std::exception& ex) {
    ROS_ERROR_STREAM("Exception" << ex.what());
  }

  return 0;
}