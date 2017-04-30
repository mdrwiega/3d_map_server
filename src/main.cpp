/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <octomap/octomap.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <cmath>
#include <list>
#include <stdlib.h>

#include <Eigen/SVD>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

using namespace octomap;
using namespace octomath;

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

constexpr unsigned MAX_ITER = 500;

void tree2PointCloud(OcTree *tree, pcl::PointCloud<pcl::PointXYZ>& pclCloud)
{
    // Traverse all leafs in the tree
    for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it)
    {
        if (tree->isNodeOccupied(*it))
            pclCloud.push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
    }
}

bool pointInBBox (pcl::PointXYZ& point, pcl::PointXYZ& bboxMin, pcl::PointXYZ& bboxMax)
{
    return (point.x < bboxMax.x && point.x > bboxMin.x) &&
           (point.y < bboxMax.y && point.y > bboxMin.y) &&
           (point.z < bboxMax.z && point.z > bboxMin.z);
}

void extractIntersectingPointClouds(pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                    pcl::PointCloud<pcl::PointXYZ>& cloud2,
                                    pcl::PointCloud<pcl::PointXYZ >::Ptr& src,
                                    pcl::PointCloud<pcl::PointXYZ >::Ptr& tgt,
                                    Eigen::Matrix4f& tfEst, double mapRes)
{
    // apply the tfEst to cloud2
    pcl::transformPointCloud(cloud2, cloud2, tfEst);

    PointCloud::Ptr cloud1filtered(new PointCloud);
    PointCloud::Ptr cloud2filtered(new PointCloud);

    // get the bounding region of  cloud1 to extract the points from cloud2 contained in the region
    pcl::PointXYZ minCloud1, maxCloud1;
    pcl::getMinMax3D(cloud1, minCloud1, maxCloud1);

    // filter out the points in cloud2 that are not in cloud1’s range
    for (auto it = cloud2.begin(); it != cloud2.end(); it++)
    {
        if (pointInBBox(*it, minCloud1, maxCloud1))
            cloud2filtered->push_back(*it);
    }

    // filter out the points in cloud1 that are not in cloud2’s range
    pcl::PointXYZ minCloud2filtered, maxCloud2filtered;
    pcl::getMinMax3D(*cloud2filtered, minCloud2filtered, maxCloud2filtered);

    minCloud2filtered = pcl::PointXYZ(minCloud2filtered.x-1, minCloud2filtered.y-1, minCloud2filtered.z-1);
    maxCloud2filtered = pcl::PointXYZ(maxCloud2filtered.x+1, maxCloud2filtered.y+1, maxCloud2filtered.z+1);

    for(auto it = cloud1.begin(); it != cloud1.end(); it++)
    {
        if (pointInBBox(*it, minCloud2filtered, maxCloud2filtered))
            cloud1filtered->push_back(*it);
    }

    // Downsample for consistency and speed
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize(10*mapRes, 10*mapRes, 10*mapRes);
    grid.setInputCloud(cloud1filtered);
    grid.filter(*tgt);
    grid.setInputCloud(cloud2filtered);
    grid.filter(*src);
}

/**
 *
 * @param[in] tfEst - initial transform estimate
 */
Eigen::Matrix4f getICPTransformation (pcl::PointCloud<pcl::PointXYZ>& cloud1,
                                      pcl::PointCloud<pcl::PointXYZ>& cloud2,
                                      Eigen::Matrix4f& tfEst,
                                      double mapRes)
{
    PointCloud::Ptr src(new PointCloud);
    PointCloud::Ptr tgt(new PointCloud);

    extractIntersectingPointClouds(cloud1, cloud2, src, tgt, tfEst, mapRes);

    // Align
    pcl::IterativeClosestPointNonLinear <pcl::PointXYZ, pcl::PointXYZ> reg;
    reg.setTransformationEpsilon(mapRes / 60);
    reg.setMaxCorrespondenceDistance(10 * mapRes);

    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev;
    PointCloud::Ptr reg_result;

    if (src->size() < tgt->size())
    {
        reg.setInputSource(src);
        reg.setInputTarget(tgt);

        // Run the same optimization in a loop and visualize the results
        reg_result = src;
        reg.setMaximumIterations(2);

        for (size_t i=0; i < MAX_ITER; ++i)
        {
            // save cloud for visualization purpose
            src = reg_result;

            // Estimate
            reg.setInputSource(src);
            reg.align(*reg_result);

            // accumulate transformation between each Iteration
            Ti = reg.getFinalTransformation() * Ti;

            // if the difference between this transformation and the previous one
            // is smaller than the threshold, refine the process by reducing
            // the maximal correspondence distance
            if(reg.getMaxCorrespondenceDistance() > 0.2)
            {
//                if (fabs((reg.getLastIncrementalTransformation() - prev.sum())) < reg.getTransformationEpsilon())
                    reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance() - 0.1);
            }
            else if(reg.getMaxCorrespondenceDistance() > 0.002)
            {
//                if(fabs(( reg.getLastIncrementalTransformation() - prev.sum())) < reg.getTransformationEpsilon())
                    reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance() - 0.001);
            }
            prev = reg.getLastIncrementalTransformation();
        }
    }
    else
    {
        reg.setInputSource(tgt);
        reg.setInputTarget(src);

        // Run the same optimization in a loop and visualize the results
        reg_result = tgt;
        reg.setMaximumIterations(2);

        for (size_t i= 0;i< MAX_ITER; ++i)
        {
            // save cloud for visualization purpose
            tgt = reg_result;

            // Estimate
            reg.setInputSource(tgt);
            reg.align(*reg_result);

            // accumulate transformation between each Iteration
            Ti = reg.getFinalTransformation() * Ti;

            // if the difference between t his transformation and the previous one
            // is smaller than the threshold, refine the process by reducing
            // the maximal correspondence distance
            if(reg.getMaxCorrespondenceDistance() > 0.2) {
//                if (fabs((reg.getLastIncrementalTransformation() - prev.sum())) < reg.getTransformationEpsilon())
                    reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance() - 0.1);
            }
            else if(reg.getMaxCorrespondenceDistance() > 0.002) {
//                if (fabs((reg.getLastIncrementalTransformation() - prev.sum())) < reg.getTransformationEpsilon())
                reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance() - 0.001);
            }
            prev = reg.getLastIncrementalTransformation();
        }
        Ti = Ti.inverse();
    }
    return Ti * tfEst;
}

int main(int argc, char **argv)
{
    std::string filename1 = std::string(argv [ 1 ]);
    std::string filename2 = std::string(argv [ 2 ]);
    std::string outputFilename = std::string(argv [ 3 ]);
    std::cout << "\nReading octree files...\n";

    double roll, pitch, yaw;

    point3d translation;
    if(argc == 7 || argc == 10)
        translation = point3d(atof(argv[4]), atof(argv[5]), atof(argv[6]));

    if(argc == 10)
    {
        roll  = atof(argv [7]);
        pitch = atof(argv [8]);
        yaw   = atof(argv [9]);
    }
    else
    {
        roll = 0;
        pitch = 0;
        yaw = 0;
    }

    Pose6D pose(translation.x(), translation.y(), translation.z (), roll, pitch, yaw);

    // build a transform matrix
    Eigen::Matrix4f transform;
    std::vector <double> coeffs;
    pose.rot().toRotMatrix(coeffs);
    transform << coeffs [0], coeffs [1], coeffs [2], translation.x(),
        coeffs[3], coeffs[4], coeffs[5], translation.y(),
        coeffs[6], coeffs[7], coeffs[8], translation.z (),
        0, 0, 0, 1;

    OcTree *tree1 = dynamic_cast<OcTree*>(OcTree::read(filename1));
    OcTree *tree2 = dynamic_cast<OcTree*>(OcTree::read(filename2));

    std::cout << "\nInitial transformation: " << transform << std::endl;

    // makepoint clouds from each map
    PointCloud tree1Points;
    tree2PointCloud(tree1, tree1Points);

    PointCloud tree2Points;
    tree2PointCloud(tree2, tree2Points);

    // get refined matrix
    transform = getICPTransformation(tree1Points, tree2Points, transform, 1);

    std::cout << "\nTransformation: " << transform << std::endl;

    delete tree1;
    delete tree2;


  return 0;
}
