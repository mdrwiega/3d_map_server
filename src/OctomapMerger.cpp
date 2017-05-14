/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "OctomapMerger.hh"

#include <iomanip>
#include <iostream>
#include <stdexcept>

#include <pcl/registration/icp_nl.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>

#include "OctreeUtils.hh"
#include "pointcloud_utils.h"

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

void OctomapMerger::extractIntersectingPointClouds(const PointCloud& cloud1, const PointCloud& cloud2, double voxelSize,
                                    PointCloud& cloud1reduced, PointCloud& cloud2reduced)
{
    PointCloud::Ptr cloud1filtered(new PointCloud);
    PointCloud::Ptr cloud2filtered(new PointCloud);

    // Filter out points in cloud2 that are not in cloud1’s range
    Point minCloud1, maxCloud1;
    pcl::getMinMax3D(cloud1, minCloud1, maxCloud1);
    filterOutPointsNotInRange(cloud2, minCloud1, maxCloud1, *cloud2filtered);

    // Filter out points in cloud1 that are not in cloud2’s range
    Point minCloud2filtered, maxCloud2filtered;
    pcl::getMinMax3D(*cloud2filtered, minCloud2filtered, maxCloud2filtered);
//    minCloud2filtered = Point(minCloud2filtered.x-1, minCloud2filtered.y-1, minCloud2filtered.z-1);
//    maxCloud2filtered = Point(maxCloud2filtered.x+1, maxCloud2filtered.y+1, maxCloud2filtered.z+1);
    filterOutPointsNotInRange(cloud1, minCloud2filtered, maxCloud2filtered, *cloud1filtered);

    downsamplePointCloud(cloud1filtered, cloud1reduced, voxelSize);
    downsamplePointCloud(cloud2filtered, cloud2reduced, voxelSize);
}


/**
 *
 * @param[in] tfEst - initial transform estimate
 */
Eigen::Matrix4f OctomapMerger::getICPTransformation (PointCloud& cloud1, PointCloud& cloud2, double mapRes)
{
    PointCloud::Ptr source(new PointCloud);
    PointCloud::Ptr target(new PointCloud);

    std::string cloudName = "cloud1"; printPointcloudInfo(cloud1, cloudName);
    cloudName = "cloud2";             printPointcloudInfo(cloud2, cloudName);

    extractIntersectingPointClouds(cloud1, cloud2, mapRes, *source, *target);

    cloudName = "cloud1reduced"; printPointcloudInfo(*source, cloudName);
    cloudName = "cloud2reduced"; printPointcloudInfo(*target, cloudName);

    // Align
    pcl::IterativeClosestPointNonLinear <Point, Point> icp;
    icp.setTransformationEpsilon(mapRes / 60);
    icp.setMaxCorrespondenceDistance(10 * mapRes);

    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev;
    PointCloud::Ptr icpResult;

    if (source->size() < target->size())
    {
        icp.setInputSource(source);
        icp.setInputTarget(target);
        icpResult = source;
    }
    else
    {
        icp.setInputSource(target);
        icp.setInputTarget(source);
        icpResult = target;
    }
    // Run the same optimization in a loop and visualize the results
    icp.setMaximumIterations(2);

    for (size_t i=0; i < MAX_ITER; ++i)
    {
        // Estimate
        if (source->size() < target->size())
            icp.setInputSource(source);
        else
            icp.setInputSource(target);

        icp.align(*icpResult);

        // accumulate transformation between each Iteration
        Ti = icp.getFinalTransformation() * Ti;

        // if the difference between this transformation and the previous one
        // is smaller than the threshold, refine the process by reducing
        // the maximal correspondence distance
        const auto maxCorrDist = icp.getMaxCorrespondenceDistance();
        const auto lastIncTransformation = icp.getLastIncrementalTransformation();
        if(maxCorrDist > 0.2)
        {
            if (fabs((lastIncTransformation - prev).sum()) < icp.getTransformationEpsilon())
                icp.setMaxCorrespondenceDistance (maxCorrDist - 0.1);
        }
        else if(maxCorrDist > 0.002)
        {
            if (fabs((lastIncTransformation - prev).sum()) < icp.getTransformationEpsilon())
                icp.setMaxCorrespondenceDistance (maxCorrDist - 0.001);
        }
        prev = lastIncTransformation;
    }

    if (source->size() < target->size())
        return Ti;
    else
        return Ti.inverse();
}

std::unique_ptr<octomap::OcTree> OctomapMerger::mergeOctrees(
        const octomap::OcTree& tree1, const octomap::OcTree& tree2,
        const Eigen::Matrix4f& transform)
{
    PointCloud::Ptr cloud1(new PointCloud);
    PointCloud::Ptr cloud2(new PointCloud);

    tree2PointCloud(&tree1, *cloud1);
    tree2PointCloud(&tree2, *cloud2);

    // Initial transformation of cloud2
    pcl::transformPointCloud(*cloud2, *cloud2, transform);


//    PointCloud::Ptr src(new PointCloud);
//    PointCloud::Ptr tgt(new PointCloud);
//    extractIntersectingPointClouds(cloud1, cloud2, 0.1, *src, *tgt);

    auto transform1 = getICPTransformation(*cloud1, *cloud2, 0.3) * transform;
    std::cout << "\nTransformation:\n" << transform1 << std::endl;
    Eigen::Matrix3f rot = transform1.block(0, 0, 3, 3);
    std::cout << "\nRPY: " << rot.eulerAngles(0,1,2).transpose();
    std::cout << "\nTranslation: " << transform1.col(3).transpose() << "\n";

    return nullptr;
}

}
