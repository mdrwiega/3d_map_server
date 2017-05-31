/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "OctreeUtils.hh"
#include "utils/Logger.hh"

namespace octomap_tools {

Eigen::Matrix4f createTransformationMatrix(
        float x, float y, float z, float roll, float pitch, float yaw)
{
    using namespace Eigen;
    Matrix4f transform = Matrix4f::Identity();

    Matrix3f m;
    m = AngleAxisf(roll,  Vector3f::UnitX())
      * AngleAxisf(pitch, Vector3f::UnitY())
      * AngleAxisf(yaw,   Vector3f::UnitZ());

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            transform(i,j) = m(i,j);

    transform.col(3) = Vector4f(x, y, z, 1);

    return transform;
}

void writeOcTreeToFile(const octomap::OcTree& tree, const std::string& fileName)
{
    LOG_INF() << "Writing OcTree file: " << fileName << std::endl;
    if (!tree.write(fileName))
    {
        LOG_ERR() << "Error writing to " << fileName << std::endl;
        throw std::runtime_error("Error writing to " + fileName);
    }
}

void writePointCloudAsOctreeToFile(PointCloud::Ptr& cloud, const std::string& fileName)
{
    octomap::Pointcloud scan;

    for (const auto& i : cloud->points)
        scan.push_back(i.x, i.y, i.z);

    std::unique_ptr<octomap::OcTree> tree(new octomap::OcTree(0.1));

    octomap::point3d sensorPose{0,0,0};
    tree->insertPointCloud(scan, sensorPose);

    LOG_INF() << "Pruning octree\n\n";
    tree->updateInnerOccupancy();
    tree->prune();

    writeOcTreeToFile(*tree, fileName);
}

PointCloud convertOctreeToPointcloud(octomap::OcTree& tree)
{
    auto maxDepth = tree.getTreeDepth();
    LOG_INF() << "tree depth is " << maxDepth << std::endl;

    // Expansion of occupied nodes
     std::vector<octomap::OcTreeNode*> collapsedOccNodes;
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
         LOG_INF() << "expanded " << collapsedOccNodes.size() << " nodes" << std::endl;
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

void tree2PointCloud(const octomap::OcTree *tree, pcl::PointCloud<pcl::PointXYZ>& pclCloud)
{
    // Traverse all leafs in the tree
    for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it)
    {
        if (tree->isNodeOccupied(*it))
            pclCloud.push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
    }
}

std::unique_ptr<octomap::OcTree> readOctreeFromFile(const std::string fileName)
{
    LOG_INF() << "\nReading OcTree file: " << fileName
              << "\n===========================\n";
    std::ifstream file(fileName.c_str(), std::ios_base::in | std::ios_base::binary);

    if (!file.is_open())
    {
        LOG_ERR() << "Filestream to "<< fileName << " not open, nothing read.";
        return nullptr;
    }

    std::istream::pos_type streampos = file.tellg();
    std::unique_ptr<octomap::OcTree> tree;

    // Reading new format of octree (.ot)
    if (fileName.length() > 3 && (fileName.compare(fileName.length()-3, 3, ".ot") == 0))
    {
        tree.reset(dynamic_cast<octomap::OcTree*>(octomap::AbstractOcTree::read(file)));
        if (!tree)
        {
            LOG_ERR() << "Error reading from file " << fileName << std::endl;
            file.clear();
            file.seekg(streampos);
            return nullptr;
        }
    }
    else // Reading old format of octree (.bt)
    {
        std::unique_ptr<octomap::OcTree> bTree (new octomap::OcTree(0.1));

        if (bTree->readBinary(file) && bTree->size() > 1)
            tree.reset(bTree.release());
        else
        {
            LOG_ERR() << "Could not detect binary OcTree format in file.";
            return nullptr;
        }
    }
    return tree;
}

}
