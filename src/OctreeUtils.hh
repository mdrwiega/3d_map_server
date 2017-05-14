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
#include <stdexcept>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>
#include <Eigen/Dense>

namespace octomap_tools {

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

Eigen::Matrix4f createTransformationMatrix(
        float x, float y, float z, float roll, float pitch, float yaw);

void writeOcTreeToFile(const octomap::OcTree& tree, const std::string& fileName);

void writePointCloudAsOctreeToFile(PointCloud::Ptr& cloud, const std::string& fileName);

PointCloud convertOctreeToPointcloud(octomap::OcTree& tree);

void tree2PointCloud(const octomap::OcTree *tree, pcl::PointCloud<pcl::PointXYZ>& pclCloud);

std::unique_ptr<octomap::OcTree> readOctreeFromFile(const std::string fileName);

}
