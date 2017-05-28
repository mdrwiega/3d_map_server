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

    struct EstimationParams
    {
        unsigned maxIter;               // Max number of iterations (ICP)
        float    maxCorrespondenceDist; // Max correspondence distance (ICP)
        float    fitnessEps;            // Euclidean fitness epsilon (ICP)
        float    transfEps;             // Transformation epsilon (ICP)
        float    voxelSize;             // Size of voxel after downsampling
        Point    intersecMargin;        // Margin in intersecting region extraction
    };

    void filterOutPointsNotInRange(const PointCloud& cloudIn,
            const Point& min, const Point& max, PointCloud& cloudOut);

    void downsamplePointCloud(
            const PointCloud::ConstPtr& cloudIn, PointCloud& cloudOut, double voxelSize);

    void extractIntersectingAndDownsamplePointClouds(
            const PointCloud& cloud1, const PointCloud& cloud2,
            float voxelSize, const Point& margin,
            PointCloud& cloud1reduced, PointCloud& cloud2reduced);

    Eigen::Matrix4f computeTransBetweenPointclouds(
            const PointCloud& cloud1, const PointCloud& cloud2, const EstimationParams& params);

    std::unique_ptr<PointCloud> mergePointclouds(
            const PointCloud& cloud1, const PointCloud& cloud2,
            const Eigen::Matrix4f& transform);

    std::unique_ptr<octomap::OcTree> mergeOctrees(
            const octomap::OcTree& tree1, const octomap::OcTree& tree2,
            const Eigen::Matrix4f& transform);
};

}
