/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

/**
 * This tool splits pointcloud (.pcd) into two parts
 * The plane of split is determined by three points:
 * p1=(x1,y1,z1), p2=(x2,y2,z2), p3=(x3,y3,z3)
 *
 * Usage: <input_file.pcd> <out1_file.pcd> <out2_file.pcd> x1 y1 z1 x2 y2 z2 x3 y3 z3
 *
 * Example:
 * ./pointcloud_splitter fr_079.pcd out1.pcd out2.pcd 0 4 0 4 4 0 0 4 4
 * It'll split the pointcloud through plane parallel to axis x at y equal to 4
 */

#include <cstdlib>
#include <iostream>

#include <Eigen/Dense>

#include "../utils/pointcloud_utils.h"
#include "md_utils/math/geometry.h"

using namespace octomap_tools;
using namespace Eigen;

void printHelp(const char* progName)
{
  std::cout << "Usage: " << progName
      << " <in_file.pcd> <out_file1.pcd> <out_file2.pcd>"
      << " x1 y1 z1 x2 y2 z2 x3 y3 z3\n"
      << "This program splits pointcloud into two parts.\n"
      << "The plane of split is determined by three points.\n";
  exit(0);
}

int main(int argc, char** argv)
{
  if (argc != 13)
    printHelp(argv[0]);

  const std::string inFilename = argv[1];
  const std::string outFilename1 = argv[2];
  const std::string outFilename2 = argv[3];

  Matrix3f A;
  auto spaceSize = static_cast<size_t>(A.rows());

  for (size_t i = 0, j = 4; i < spaceSize; i++, j += spaceSize)
  {
    A.row(i) = Vector3f{
      std::strtof(argv[j], nullptr),
          std::strtof(argv[j+1], nullptr),
          std::strtof(argv[j+2], nullptr)};
  }

  const auto plane = md::planeFromThreePoints(A);
  const auto cloud = readPointCloudFromFile(inFilename);

  printPointcloudInfo(*cloud, inFilename);

  PointCloud out1, out2;
  splitPointcloud(plane, *cloud, out1, out2);

  printPointcloudInfo(out1, outFilename1);
  printPointcloudInfo(out2, outFilename2);

  // Fill in the cloud data
  out1.width    = out1.size();
  out1.height   = 1;
  out1.is_dense = false;
  pcl::io::savePCDFileASCII (outFilename1, out1);
  std::cout << "\nSaved " << out1.points.size () << " data points to "
      << outFilename1 << std::endl;

  out2.width    = out2.size();
  out2.height   = 1;
  out2.is_dense = false;
  pcl::io::savePCDFileASCII (outFilename2, out2);
  std::cout << "Saved " << out2.points.size () << " data points to "
      << outFilename2 << std::endl;

  return 0;
}
