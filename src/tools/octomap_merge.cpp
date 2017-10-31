/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

/**
 * This tool merges two octomaps (.ot) into one.
 * The initial transformation should be passed
 * in form of: x y z roll pitch yaw
 *
 * Usage: <in_file1.ot> <in_file2.ot> <out_file.ot> x y z r p y
 *        saveToFile voxelSize maxIter maxCorrespondenceDist fitnessEpsilon
 *
 * Example: ./octomap_merge
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "../octomap_merger.h"
#include "utils/OctreeUtils.hh"

using namespace octomap_tools;
using namespace Eigen;

void printHelp(const char* progName)
{
  std::cout << "Usage: " << progName
      << " <in_file1.ot> <in_file2.ot> <out_file.ot> x y z r p y\n"
      << "This program merge two octomaps into one.\n"
      << "The initial transformation should be passed.\n";
}

int main(int argc, char** argv)
{
  if (argc != 10 && argc != 15)
  {
    printHelp(argv[0]);
    return -1;
  }

  const std::string inFilename1 = argv[1];
  const std::string inFilename2 = argv[2];
  const std::string outFilename = argv[3];

  float x = std::atof(argv[4]);
  float y = std::atof(argv[5]);
  float z = std::atof(argv[6]);
  float roll  = std::atof(argv[7]);
  float pitch = std::atof(argv[8]);
  float yaw   = std::atof(argv[9]);

  bool saveToFile = (0 != std::atof(argv[10]));
  float voxelSize = std::atof(argv[11]);
  int maxIterations = std::atoi(argv[12]);
  float maxCorrespDist = std::atof(argv[13]);
  float fitnessEps = std::atof(argv[14]);

  auto tree1 = readOctreeFromFile(inFilename1);
  auto tree2 = readOctreeFromFile(inFilename2);

  auto transform = createTransformationMatrix(x,y,z,roll, pitch, yaw);

  OctomapMerger merger;
  auto treeOut = merger.mergeOcTrees(*tree1, *tree2, transform);

  if (saveToFile)
    writeOcTreeToFile(*treeOut, outFilename);

  return 0;
}
