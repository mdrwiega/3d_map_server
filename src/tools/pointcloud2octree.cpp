/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

/**
 * Program converts octree (.bt or .ot) format to pointcloud (.pcd).
 * Usage: <input_file.ot> <output_file.pcd>
 */

#include <stdlib.h>

#include <octomap/octomap.h>

#include "../utils/pointcloud_utils.h"

using namespace octomap;
using namespace octomap_tools;

void writePointCloudAsOctreeToFile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                   const std::string fileName)
{
  Pointcloud scan;

  for (const auto& i : cloud->points)
    scan.push_back(i.x, i.y, i.z);

  std::unique_ptr<OcTree> tree(new OcTree(0.1));

  octomap::point3d sensorPose{0,0,0};
  tree->insertPointCloud(scan, sensorPose);

  std::cout << "Pruning octree\n\n";
  tree->updateInnerOccupancy();
  tree->prune();

  std::cerr << "Writing OcTree file" << std::endl;
  if (!tree->write(fileName))
    std::cerr << "Error writing to " << fileName << std::endl;
}

void printHelp(const char* progName)
{
  std::cout << "Usage: " << progName << " <input_file.pcd> <output_file.ot>\n";
  std::cout << "This program converts pointcloud to octree.\n";
  exit(0);
}

int main(int argc, char** argv)
{
  if (argc != 3)
    printHelp(argv[0]);

  const std::string inFilename = argv[1];
  const std::string outFilename = argv[2];

  auto cloud = readPointCloudFromFile(inFilename);

  writePointCloudAsOctreeToFile(cloud, outFilename);

  return 0;
}
