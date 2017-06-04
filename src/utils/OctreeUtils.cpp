/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "OctreeUtils.hh"
#include "utils/Logger.hh"

namespace octomap_tools {

void writeOcTreeToFile(const OcTree& tree, const std::string& fileName)
{
  LOG_INF() << "Writing OcTree file: " << fileName << std::endl;
  if (!tree.write(fileName))
  {
    LOG_ERR() << "Error writing to " << fileName << std::endl;
    throw std::runtime_error("Error writing to " + fileName);
  }
}

void writePointCloudAsOctreeToFile(PointCloud::Ptr& cloud,
                                   const std::string& fileName)
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

PointCloud convertOctreeToPointcloud(OcTree& tree)
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

void tree2PointCloud(const OcTree *tree,
                     pcl::PointCloud<pcl::PointXYZ>& pclCloud)
{
  // Traverse all leafs in the tree
  for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it)
  {
    if (tree->isNodeOccupied(*it))
      pclCloud.push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
  }
}

PointCloud createUniformPointCloud(Point min, Point max, Point step)
{
  if (min.x > max.x || min.y > max.y || min.z > max.z)
    throw std::runtime_error("Incorrect ranges");

  PointCloud cloud;

  for (float i = min.x; i < max.x; i += step.x)
    for (float j = min.y; j < max.y; j += step.y)
      for (float k = min.z; k < max.z; k += step.z)
        cloud.push_back(Point{(float)i, (float)j, (float)k});

  return cloud;
}

std::unique_ptr<OcTree> readOctreeFromFile(const std::string fileName)
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

int getNodeDepth(const OcTree& tree, const octomap::point3d& point,
                 const OcTreeNode& node)
{
  for(int depth = tree.getTreeDepth(); depth > 1; --depth)
  {
    if (tree.search(point, depth) == &node)
      return depth;
  }
  return -1;
}

int getLeafDepth(const OcTree& tree, const OcTreeNode& node)
{
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it)
  {
    if (tree.search(it.getIndexKey()) == &node)
      return static_cast<int>(it.getDepth());
  }
  return -1;
}


void printOcTree(const OcTree& tree, std::string name)
{
  double xMin, xMax, yMin, yMax, zMin, zMax;

  tree.getMetricMin(xMin, yMin, zMin);
  tree.getMetricMax(xMax, yMax, zMax);

  LOG_INF() << "OcTree: " << name;
  LOG_INF() << "Size:" << tree.size() << "  Resolution: " << tree.getResolution();
  LOG_INF() << "MinMax x(" << xMin << ", " << xMax << ")  y(" << yMin << ", " << yMax
      << ")  z(" << zMin << ", " << zMax << ")";
  LOG_INF() << "Leafs:";
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it)
  {
    LOG_TEST() << it.getCoordinate() << "  LogOdds: " << it->getLogOdds()
                                     << "  Depth: " << getLeafDepth(tree, *it);
  }
  LOG_INF();
}

void filterOutPointsNotInRange(const PointCloud& cloudIn,
                               const Point& min, const Point& max,
                               PointCloud& cloudOut)
{
  auto pointInRange = [](const Point& point, const Point& rMin, const Point& rMax){
    return (point.x < rMax.x && point.x > rMin.x) &&
        (point.y < rMax.y && point.y > rMin.y) &&
        (point.z < rMax.z && point.z > rMin.z);
  };

  cloudOut.clear();
  for (const auto& point : cloudIn)
  {
    if (pointInRange(point, min, max))
      cloudOut.push_back(point);
  }
}

void expandNodeOnlyEmptyChilds(OcTreeNode* node, OcTree& tree)
{
  for (unsigned k = 0; k < 8; k++)
  {
    if (!node->childExists(k))
    {
      OcTreeNode* newNode = tree.createNodeChild(node, k);
      newNode->copyData(*node);
    }
  }
}

}
