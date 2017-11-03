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
#include <iostream>
#include <memory>

#include <octomap/octomap.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "../utils/octree_utils.h"
#include "../utils/pointcloud_utils.h"

using namespace octomap;
using namespace octomap_tools;

void writeOcTreeAsPointCloudToFIle(OcTree& tree, const std::string fileName)
{
  auto cloud = convertOctreeToPointcloud(tree);

  // Fill in the cloud data
  cloud.width    = cloud.size();
  cloud.height   = 1;
  cloud.is_dense = false;

  pcl::io::savePCDFileASCII (fileName, cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to " << fileName << std::endl;
}

void printHelp(const char* progName)
{
  std::cout << "Usage: " << progName << " <input_file.ot> <output_file.pcd>\n";
  std::cout << "This program converts octree to pointcloud.\n";
  exit(0);
}

int main(int argc, char** argv)
{
  if (argc != 3)
    printHelp(argv[0]);

  const std::string inFilename = argv[1];
  const std::string outFilename = argv[2];

  std::unique_ptr<OcTree> tree = readOctreeFromFile(inFilename);

  writeOcTreeAsPointCloudToFIle(*tree, outFilename);

  return 0;
}
