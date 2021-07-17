/**
 * This tool filters octree (.ot)
 * The plane of split is determined by three points:
 * p1=(x1,y1,z1), p2=(x2,y2,z2), p3=(x3,y3,z3)
 *
 * Usage: <input_file.ot> <out1_file.ot> <out2_file.ot> x1 y1 z1 x2 y2 z2 x3 y3 z3
 *
 * Example:
 * ./pointcloud_splitter fr_079.ot out1.ot out2.ot 0 4 0 4 4 0 0 4 4
 * It'll split the octree through plane parallel to axis x at y equal to 4
 */

#include <cstdlib>
#include <iostream>

#include <Eigen/Dense>

#include <common/utils.h>
#include <octomap_tools/octomap_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <common/conversions.h>

using namespace octomap_tools;
using namespace Eigen;

void printHelp(const char* progName)
{
  std::cout << "Usage: " << progName
      << " <in_file.ot> <out_file1.ot>"
      << " x_min x_max y_min y_max z_min z_max\n"
      << "This program filters octree.\n";
  exit(0);
}

int main(int argc, char** argv)
{
  if (argc != 5)
    printHelp(argv[0]);

  const std::string in_filename = argv[1];
  const std::string out_filename = argv[2];

  const float mean_k = std::strtof(argv[3], nullptr);
  const float std_dev = std::strtof(argv[4], nullptr);

  PointCloudPtr orig_cloud(new PointCloud);
  PointCloudPtr cropped_cloud(new PointCloud);
  auto tree = LoadOcTreeFromFile(in_filename);
  orig_cloud = OcTreeToPointCloud(*tree);

  PrintOcTreeInfo(*tree, "Loaded tree");
  std::cout << PointCloudInfoToString(*orig_cloud, "orig cloud");

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (orig_cloud);
  sor.setMeanK (mean_k);
  sor.setStddevMulThresh (std_dev);
  sor.filter (*cropped_cloud);

  std::cout << PointCloudInfoToString(*cropped_cloud, "cropped_cloud");
  SavePointCloudAsOctreeToFile(cropped_cloud, out_filename, tree->getResolution());
  return 0;
}
