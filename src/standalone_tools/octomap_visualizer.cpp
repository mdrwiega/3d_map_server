#include <cstdlib>
#include <iostream>

#include <common/utils.h>
#include <octomap_tools/octomap_io.h>
#include <octomap_tools/maps_integrator_visualizer.h>
#include <common/conversions.h>

using namespace octomap_tools;

void printHelp(const char* progName) {
  std::cout << "Usage: " << progName
      << " <tree1.ot> [<tree2.ot>] <output_file>\n"
      << "This program shows octomap info.\n";
  exit(0);
}

int main(int argc, char** argv) {
  if (argc != 3 && argc != 4) {
    printHelp(argv[0]);
  }

  MapsIntegratorVisualizer::Config cfg;
  cfg.screen_mode = false;
  cfg.save_to_file = true;

  auto tree1 = LoadOcTreeFromFile(argv[1]);
  auto cloud1 = OcTreeToPointCloud(*tree1);

  PointCloudPtr cloud2;
  if (argc == 3) {
    cfg.filename = argv[2];
  }
  else if (argc == 4) {
    auto tree2 = LoadOcTreeFromFile(argv[2]);
    cloud2 = OcTreeToPointCloud(*tree2);
    cfg.filename = argv[3];
  }

  MapsIntegratorVisualizer vis(cfg);
  vis.Visualize(cloud1, cloud2);

  return 0;
}
