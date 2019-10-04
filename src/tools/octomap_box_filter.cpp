/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

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

#include <pcl/filters/crop_box.h>

#include <octomap_tools/utils.h>
#include "md_utils/math/geometry.h"
#include <octomap_tools/types_conversions.h>

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
  if (argc != 9)
    printHelp(argv[0]);

  const std::string inFilename = argv[1];
  const std::string outFilename1 = argv[2];

  const float x_min = std::strtof(argv[3], nullptr);
  const float x_max = std::strtof(argv[4], nullptr);
  const float y_min = std::strtof(argv[5], nullptr);
  const float y_max = std::strtof(argv[6], nullptr);
  const float z_min = std::strtof(argv[7], nullptr);
  const float z_max = std::strtof(argv[8], nullptr);

  PointCloudPtr orig_cloud(new PointCloud);
  PointCloudPtr cropped_cloud(new PointCloud);
  Vector4f octomap_min = Vector4f{x_min, y_min, z_min, 0};
  Vector4f octomap_max = Vector4f{x_max, y_max, z_max, 0};
  auto tree = loadOctreeFromFile(inFilename);
  *orig_cloud = OctreeToPointCloud(*tree);

  printOcTreeInfo(*tree, "Loaded tree");
  std::cout << pointcloudInfoToString(*orig_cloud, "orig cloud");

  pcl::CropBox<Point> boxFilter;
  boxFilter.setMin(octomap_min);
  boxFilter.setMax(octomap_max);
  boxFilter.setInputCloud(orig_cloud);
  boxFilter.filter(*cropped_cloud);
  printPointcloudInfo(*cropped_cloud, "cropped_cloud");
  std::cout << pointcloudInfoToString(*cropped_cloud, "cropped_cloud");

  writePointCloudAsOctreeToFile(cropped_cloud, outFilename1);

  return 0;
}
