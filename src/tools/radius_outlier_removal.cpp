#include <cstdlib>
#include <iostream>

#include <Eigen/Dense>

#include <utils.h>
#include <octomap_tools/octomap_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <conversions.h>

using namespace octomap_tools;
using namespace Eigen;

void printHelp(const char* progName)
{
  std::cout << "Usage: " << progName
      << " <in_file.ot> <out_file1.ot>"
      << " radius min_neighbors\n"
      << "This program filters octree.\n";
  exit(0);
}

int main(int argc, char** argv)
{
  if (argc != 5)
    printHelp(argv[0]);

  const std::string in_filename = argv[1];
  const std::string out_filename = argv[2];

  const float radius = std::strtof(argv[3], nullptr);
  const float min_neighbors = std::strtof(argv[4], nullptr);

  PointCloudPtr orig_cloud(new PointCloud);
  PointCloudPtr cropped_cloud(new PointCloud);
  auto tree = LoadOcTreeFromFile(in_filename);
  orig_cloud = OcTreeToPointCloud(*tree);

  PrintOcTreeInfo(*tree, "Loaded tree");
  std::cout << PointCloudInfoToString(*orig_cloud, "orig cloud");

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(orig_cloud);
  sor.setRadiusSearch(radius);
  sor.setMinNeighborsInRadius(min_neighbors);
  sor.filter (*cropped_cloud);

  std::cout << PointCloudInfoToString(*cropped_cloud, "cropped_cloud");

  SavePointCloudAsOctreeToFile(cropped_cloud, out_filename, tree->getResolution());

  return 0;
}
