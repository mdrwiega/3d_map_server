/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <memory>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <octomap/octomap.h>

namespace octomap_tools {

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

class OctomapMerger
{
public:
    OctomapMerger() = default;
    ~OctomapMerger() = default;

    void filterOutPointsNotInRange(const PointCloud& cloudIn,
            const Point& min, const Point& max, PointCloud& cloudOut);

    void downsamplePointCloud(
            const PointCloud::ConstPtr& cloudIn, PointCloud& cloudOut, double voxelSize);


    void extractIntersectingPointClouds(
            const PointCloud& cloud1, const PointCloud& cloud2, double voxelSize,
            PointCloud& cloud1reduced, PointCloud& cloud2reduced);

    Eigen::Matrix4f getICPTransformation (PointCloud& cloud1, PointCloud& cloud2, double mapRes);

    std::unique_ptr<octomap::OcTree> mergeOctrees(
            const octomap::OcTree& tree1, const octomap::OcTree& tree2,
            const Eigen::Matrix4f& transform);

    static constexpr unsigned MAX_ITER = 500;
};

}
