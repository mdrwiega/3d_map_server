/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

/**
 * This tool finds transformation between two octrees (.ot).
 * The initial transformation should be passed
 * in form of: x y z roll pitch yaw
 *
 * Usage: <in_file1.ot> <in_file2.ot> <out_file.ot> x y z r p y
 *        voxelSize maxIter maxCorrespondenceDist fitnessEpsilon
 *
 * Example: ./octomap_merge
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "../octomap_merger.h"
#include "utils/OctreeUtils.hh"
#include "utils/Logger.hh"

using namespace octomap_tools;
using namespace Eigen;

void printHelp(const char* progName)
{
  std::cout << "Usage: " << progName
      << " <in_file1.ot> <in_file2.ot> x y z r p y\n"
      << " voxelSize maxIter maxCorrespondenceDist fitnessEpsilon\n"
      << "This program finds transformation between two octrees.\n";
}

int main(int argc, char** argv)
{
  if (argc != 7)
  {
    printHelp(argv[0]);
    return -1;
  }

  const std::string inFilename1 = argv[1];
  const std::string inFilename2 = argv[2];

  float voxelSize       = std::atof(argv[3]);
  int   maxIterations   = std::atoi(argv[4]);
  float maxCorrespDist  = std::atof(argv[5]);
  float fitnessEps      = std::atof(argv[6]);

  auto tree1 = readOctreeFromFile(inFilename1);
  auto tree2 = readOctreeFromFile(inFilename2);

  PointCloud::Ptr cloud1(new PointCloud);
  PointCloud::Ptr cloud2(new PointCloud);

  tree2PointCloud(tree1.get(), *cloud1);
  tree2PointCloud(tree2.get(), *cloud2);

  OctomapMerger merger;
  auto tfFinal = merger.computeTransBetweenPointclouds(
      *cloud1, *cloud2, voxelSize, maxIterations, maxCorrespDist, fitnessEps);

  auto vec = tfFinal.col(3).transpose();
  Eigen::Matrix3f rot = tfFinal.block(0, 0, 3, 3);

  LOG_INF() << "RPY: " << rot.eulerAngles(0,1,2).transpose();
  LOG_INF() << "Translation: " << vec << "\n";

  return 0;
}
