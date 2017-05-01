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

using namespace octomap;

std::unique_ptr<OcTree> readOctreeFromFile(const std::string fileName)
{
    std::cout << "\nReading OcTree file\n===========================\n";
    std::ifstream file(fileName.c_str(), std::ios_base::in | std::ios_base::binary);

    if (!file.is_open())
    {
        std::cerr << "Filestream to "<< fileName << " not open, nothing read.";
        exit(-1);
    }

    std::istream::pos_type streampos = file.tellg();
    std::unique_ptr<OcTree> tree;

    // Reading new format of octree (.ot)
    if (fileName.length() > 3 && (fileName.compare(fileName.length()-3, 3, ".ot") == 0))
    {
        tree.reset(dynamic_cast<OcTree*>(AbstractOcTree::read(file)));
        if (!tree)
        {
            OCTOMAP_WARNING_STR("Could not detect OcTree in file, trying .bt format.");
            file.clear();
            file.seekg(streampos);
            std::cerr << "Error reading from file " << fileName << std::endl;
            exit(-1);
        }
    }
    else // Reading old format of octree (.bt)
    {
        std::unique_ptr<OcTree> bTree (new OcTree(0.1));

        if (bTree->readBinary(file) && bTree->size() > 1)
            tree.reset(bTree.release());
        else
        {
            std::cerr << "Could not detect binary OcTree format in file.";
            exit(-1);
        }
    }
    return tree;
}

void writeOcTreeAsPointCloudToFIle(OcTree& tree, const std::string fileName)
{
    auto maxDepth = tree.getTreeDepth();
    std::cout << "tree depth is " << maxDepth << std::endl;

    // Expansion of occupied nodes
     std::vector<OcTreeNode*> collapsedOccNodes;
     do {
         collapsedOccNodes.clear();
         for (auto it = tree.begin(); it != tree.end(); ++it)
         {
             if(tree.isNodeOccupied(*it) && it.getDepth() < maxDepth)
             {
                 collapsedOccNodes.push_back(&(*it));
             }
         }
         for (auto it = collapsedOccNodes.begin(); it != collapsedOccNodes.end(); ++it)
         {
             tree.expandNode(*it);
         }
         std::cout << "expanded " << collapsedOccNodes.size() << " nodes" << std::endl;
     } while(collapsedOccNodes.size() > 0);

     pcl::PointCloud<pcl::PointXYZ> cloud;
     for (auto it = tree.begin(); it != tree.end(); ++it)
     {
         if(tree.isNodeOccupied(*it))
         {
             auto p = it.getCoordinate();
             cloud.push_back({p.x(), p.y(), p.z()});
         }
     }

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
