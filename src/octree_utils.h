/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <iostream>
#include <iomanip>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>
#include "octree_utils.h"

using namespace octomap;

namespace octomap_tools {

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

void writePointCloudAsOctreeToFile(PointCloud::Ptr& cloud, const std::string& fileName)
{
    Pointcloud scan;

    for (const auto& i : cloud->points)
    {
        scan.push_back(i.x, i.y, i.z);
    }

    std::unique_ptr<OcTree> tree(new OcTree(0.1));

    octomap::point3d sensorPose{0,0,0};
    tree->insertPointCloud(scan, sensorPose);

    std::cout << "Pruning octree\n\n";
    tree->updateInnerOccupancy();
    tree->prune();

    std::cerr << "Writing OcTree file" << std::endl;
    if (!tree->write(fileName))
    {
        std::cerr << "Error writing to " << fileName << std::endl;
        exit(-2);
    }
}

PointCloud convertOctreeToPointcloud(OcTree& tree)
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

    PointCloud cloud;
    for (auto it = tree.begin(); it != tree.end(); ++it)
    {
        if(tree.isNodeOccupied(*it))
        {
            auto p = it.getCoordinate();
            cloud.push_back({p.x(), p.y(), p.z()});
        }
    }
    return cloud;
}

std::unique_ptr<OcTree> readOctreeFromFile(const std::string fileName)
{
    std::cout << "\nReading OcTree file\n===========================\n";
    std::ifstream file(fileName.c_str(), std::ios_base::in | std::ios_base::binary);

    if (!file.is_open())
    {
        std::cerr << "Filestream to "<< fileName << " not open, nothing read.";
        return nullptr;
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
            return nullptr;
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
            return nullptr;
        }
    }
    return tree;
}

}
