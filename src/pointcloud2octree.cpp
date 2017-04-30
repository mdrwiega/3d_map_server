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

#include <pcl/io/file_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
using namespace octomap;

void printUsage(char* self){
  cerr << "USAGE: " << self << " <InputFile.pcd> <OutputFile.ot>\n";
  cerr << "This tool creates a point cloud of the occupied cells\n";
  exit(0);
}


int main(int argc, char** argv) {
  if (argc != 3)
    printUsage(argv[0]);

  string inputFilename = argv[1];
  string outputFilename = argv[2];

  // Read data from .pcd file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (inputFilename, *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file %s \n", inputFilename.c_str());
    return (-1);
  }
  std::cout << "Loaded " << cloud->width * cloud->height
            << " data points from " << inputFilename << std::endl;



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
    for (OcTree::iterator it = tree->begin(); it != tree->end(); ++it)
    {
      if(tree->isNodeOccupied(*it) && it.getDepth() < maxDepth)
      {
        collapsed_occ_nodes.push_back(&(*it));
      }
    }
    for (vector<OcTreeNode*>::iterator it = collapsed_occ_nodes.begin(); it != collapsed_occ_nodes.end(); ++it)
    {
      tree->expandNode(*it);
    }
    cout << "expanded " << collapsed_occ_nodes.size() << " nodes" << endl;
  } while(collapsed_occ_nodes.size() > 0);

  vector<point3d> pcl;
  for (OcTree::iterator it = tree->begin(); it != tree->end(); ++it)
  {
    if(tree->isNodeOccupied(*it))
    {
      pcl.push_back(it.getCoordinate());
    }
  }

  // prune octree
   cout << "Pruning octree" << endl << endl;
   tree->updateInnerOccupancy();
   tree->prune();

   // write octree to file
   if(output_filename.empty()) {
       cerr << "Error: No input files found." << endl << endl;
       exit(1);
   }

   cout << "Writing octree to " << output_filename << endl << endl;

   tree->writeBinary(output_filename.c_str());

   cout << "done" << endl << endl;


  delete tree;

  return 0;
}
