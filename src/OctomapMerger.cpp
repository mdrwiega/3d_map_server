/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "OctomapMerger.hh"

#include <iomanip>
#include <stdexcept>

#include <pcl/registration/icp_nl.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>

#include "utils/OctreeUtils.hh"
#include "utils/PointcloudUtils.h"

namespace octomap_tools {

void OctomapMerger::filterOutPointsNotInRange(const PointCloud& cloudIn,
        const Point& min, const Point& max, PointCloud& cloudOut)
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

void OctomapMerger::downsamplePointCloud(const PointCloud::ConstPtr& cloudIn, PointCloud& cloudOut, double voxelSize)
{
    pcl::VoxelGrid<Point> grid;
    grid.setLeafSize(voxelSize, voxelSize, voxelSize);
    grid.setInputCloud(cloudIn);
    grid.filter(cloudOut);
}

void OctomapMerger::extractIntersectingAndDownsamplePointClouds(
        const PointCloud& cloud1, const PointCloud& cloud2,
        float voxelSize, const Point& margin,
        PointCloud& cloud1reduced, PointCloud& cloud2reduced)
{
    PointCloud::Ptr cloud1filtered(new PointCloud);
    PointCloud::Ptr cloud2filtered(new PointCloud);

    Point minCloud1, maxCloud1;
    pcl::getMinMax3D(cloud1, minCloud1, maxCloud1);
    filterOutPointsNotInRange(cloud2, minCloud1, maxCloud1, *cloud2filtered);

    Point minCloud2filtered, maxCloud2filtered;
    pcl::getMinMax3D(*cloud2filtered, minCloud2filtered, maxCloud2filtered);

    auto addPoints = [](const Point& i, const Point& j){ return Point{i.x+j.x, i.y+j.y, i.z+j.z}; };
    auto subPoints = [](const Point& i, const Point& j){ return Point{i.x-j.x, i.y-j.y, i.z-j.z}; };

    minCloud2filtered = subPoints(minCloud2filtered, margin);
    maxCloud2filtered = addPoints(maxCloud2filtered, margin);

    filterOutPointsNotInRange(cloud1, minCloud2filtered, maxCloud2filtered, *cloud1filtered);

    downsamplePointCloud(cloud1filtered, cloud1reduced, voxelSize);
    downsamplePointCloud(cloud2filtered, cloud2reduced, voxelSize);
}

Eigen::Matrix4f OctomapMerger::computeTransBetweenPointclouds(
        const PointCloud& cloud1, const PointCloud& cloud2, const EstimationParams& params)
{
    PointCloud::Ptr source(new PointCloud);
    PointCloud::Ptr target(new PointCloud);

    Point margin {1,1,1};
    extractIntersectingAndDownsamplePointClouds(
            cloud1, cloud2, params.voxelSize, params.intersecMargin, *source, *target);

    pcl::IterativeClosestPoint <Point, Point> icp;
    icp.setMaxCorrespondenceDistance(params.maxCorrespondenceDist);
    icp.setMaximumIterations(params.maxIter);
    icp.setTransformationEpsilon(params.transfEps);
    icp.setEuclideanFitnessEpsilon (params.fitnessEps);

    PointCloud::Ptr icpResult;

    icp.setInputSource(source);
    icp.setInputTarget(target);
    icpResult = source;
    icp.align(*icpResult);

    return icp.getFinalTransformation();
}

std::unique_ptr<PointCloud> OctomapMerger::mergePointclouds(
        const PointCloud& cloud1, const PointCloud& cloud2,
        const Eigen::Matrix4f& transform)
{
    // Initial transformation of cloud2
//    pcl::transformPointCloud(cloud2, cloud2, transform);
//
//    auto transform1 = getTransformationBetweenPointclouds(*cloud1, *cloud2, 0.3) * transform;
//
//    std::cout << "\nTransformation:\n" << transform1 << std::endl;
//    Eigen::Matrix3f rot = transform1.block(0, 0, 3, 3);
//    std::cout << "\nRPY: " << rot.eulerAngles(0,1,2).transpose();
//    std::cout << "\nTranslation: " << transform1.col(3).transpose() << "\n";

    return nullptr;
}

std::unique_ptr<octomap::OcTree> OctomapMerger::mergeOctrees(
        const octomap::OcTree& tree1, const octomap::OcTree& tree2,
        const Eigen::Matrix4f& transform)
{
//    PointCloud::Ptr cloud1(new PointCloud);
//    PointCloud::Ptr cloud2(new PointCloud);
//
//    tree2PointCloud(&tree1, *cloud1);
//    tree2PointCloud(&tree2, *cloud2);
//
//    // Initial transformation of cloud2
//    pcl::transformPointCloud(*cloud2, *cloud2, transform);
//
//
////    PointCloud::Ptr src(new PointCloud);
////    PointCloud::Ptr tgt(new PointCloud);

    return nullptr;
}

}
