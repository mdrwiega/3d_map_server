/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <string.h>
#include <stdlib.h>

#include <octomap/octomap.h>
#include <octomap/octomap_timing.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
using namespace octomap;

void printUsage(char* self){
  cerr << "USAGE: " << self << " <InputFile.bt> <OutputFile.pcd>\n";
  cerr << "This tool creates a point cloud of the occupied cells\n";
  exit(0);
}


int main(int argc, char** argv)
{
  if (argc != 3)
    printUsage(argv[0]);

  string inputFilename = argv[1];
  string outputFilename = argv[2];

  OcTree* tree = new OcTree(0.1);
  if (!tree->readBinary(inputFilename)){
    OCTOMAP_ERROR("Could not open file, exiting.\n");
    exit(1);
  }

  unsigned int maxDepth = tree->getTreeDepth();
  cout << "tree depth is " << maxDepth << endl;
  
  // expand collapsed occupied nodes until all occupied leaves are at maximum depth
  vector<OcTreeNode*> collapsed_occ_nodes;
  do {
    collapsed_occ_nodes.clear();
    for (auto it = tree->begin(); it != tree->end(); ++it)
    {
      if(tree->isNodeOccupied(*it) && it.getDepth() < maxDepth)
      {
        collapsed_occ_nodes.push_back(&(*it));
      }
    }
    for (auto it = collapsed_occ_nodes.begin(); it != collapsed_occ_nodes.end(); ++it)
    {
      tree->expandNode(*it);
    }
    cout << "expanded " << collapsed_occ_nodes.size() << " nodes" << endl;
  } while(collapsed_occ_nodes.size() > 0);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (auto it = tree->begin(); it != tree->end(); ++it)
  {
    if(tree->isNodeOccupied(*it))
    {
        auto p = it.getCoordinate();
        cloud.push_back({p.x(), p.y(), p.z()});
    }
  }

  delete tree;

  // Fill in the cloud data
  cloud.width    = cloud.size();
  cloud.height   = 1;
  cloud.is_dense = false;

  pcl::io::savePCDFileASCII (outputFilename, cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to " << outputFilename << std::endl;
  
  return 0;
}
