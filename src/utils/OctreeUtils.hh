/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <iomanip>
#include <stdexcept>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>
#include <Eigen/Dense>

namespace octomap_tools {

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

using OcTree = octomap::OcTree;
using OcTreePtr = std::unique_ptr<octomap::OcTree>;
using OcTreeNode = octomap::OcTreeNode;
using OcTreeKey = octomap::OcTreeKey;

void writeOcTreeToFile(const OcTree& tree, const std::string& fileName);

void writePointCloudAsOctreeToFile(PointCloud::Ptr& cloud,
                                   const std::string& fileName);

PointCloud convertOctreeToPointcloud(OcTree& tree);

PointCloud createUniformPointCloud(Point min, Point max, Point step);

void tree2PointCloud(const OcTree *tree,
                     pcl::PointCloud<pcl::PointXYZ>& pclCloud);

std::unique_ptr<OcTree> readOctreeFromFile(const std::string fileName);

int getNodeDepth(const OcTree& tree, const octomap::point3d& point,
                 const OcTreeNode& node);

int getLeafDepth(const OcTree& tree, const OcTreeNode& node);

void printOcTree(const OcTree& tree, std::string name);

void filterOutPointsNotInRange(const PointCloud& cloudIn,
                               const Point& min, const Point& max,
                               PointCloud& cloudOut);

void expandNodeOnlyEmptyChilds(OcTreeNode* node, OcTree& tree);

void printPointsAndDistances(std::string title,
                             std::vector<Point>& points,
                             std::vector<float>& distances);

int getKeyDepth(const OcTree& tree, const octomap::point3d& point,
                const octomap::OcTreeKey& key);

bool contains(OcTree& tree, const Point& query, float sqRadius,
              const octomap::OcTreeKey& o);

int getClosestChild(const Point& q, const Point& p);

float squaredNorm(const Point& p);

float squaredDistance(const Point& p, const Point& q);

double getVoxelSquaredSideLen(const OcTree& tree,
                              unsigned tree_depth_arg);

double getVoxelSquaredDiameter(const OcTree& tree,
                               unsigned tree_depth_arg);

}
