/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <iostream>
#include <iomanip>
#include <limits>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

namespace octomap_tools {

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

PointCloud::Ptr readPointCloudFromFile(const std::string fileName)
{
    PointCloud::Ptr cloud (new PointCloud);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file %s \n", fileName.c_str());
        return nullptr;
    }
    std::cout << "Loaded " << cloud->width * cloud->height
            << " data points from " << fileName << std::endl;
    return cloud;
}

/**
 * Splits pointcloud into two parts
 *
 * @param[in] plane - plane equation in form: a*x + b*y + c*y + d = 0
 *                    where elements of vector are factors: a,b,c,d
 * @param[in] in - input pointcloud
 * @param[out] out1 - output pointcloud which contains points bigger than plane
 * @param[out] out2 - output pointcloud which contains points smaller than plane
 */
void splitPointcloud(const Eigen::Vector4f& plane,
        const PointCloud& in, PointCloud& out1, PointCloud& out2)
{
    out1.clear();
    out2.clear();

    for (const auto& p : in.points)
    {
        if ((plane[0] * p.x + plane[1] * p.y + plane[2] * p.z + plane[3]) > 0)
        {
            out1.push_back(p);
        }
        else
        {
            out2.push_back(p);
        }
    }
}

/**
 * Checks if 3D points in matrix (each point in row) are collinear
 */
bool pointsAreCollinear(const Eigen::Matrix3f& points)
{
    Eigen::Vector3f ab = points.row(1) - points.row(0);
    Eigen::Vector3f ac = points.row(2) - points.row(0);
    return ab.cross(ac) == Eigen::Vector3f{0,0,0};
}

/**
 * Calculates plane from three points
 *
 * @param[in] A - matrix with points coordinates in rows
 */
Eigen::Vector4f calculatePlaneFromThreePoints(const Eigen::Matrix3f& A)
{
    if (pointsAreCollinear(A))
    {
        std::cerr << "\nPoints are collinear. Please choose other points.\n";
        std::exit(-1);
    }

    Eigen::Vector3f x = A.colPivHouseholderQr().solve(Eigen::Vector3f{-1,-1,-1});
    std::cout << "\nThe plane:\n"
              << x[0] << " x + " << x[1] << " y + " << x[2] << " z + 1 = 0\n";

    return {x[0], x[1], x[2], 1};
}

void findPointcloudLimits(const PointCloud& cloud,
        float& xMin, float& yMin, float& zMin, float& xMax, float& yMax, float& zMax)
{
    xMin = yMin = zMin = std::numeric_limits<float>::max();
    xMax = yMax = zMax = std::numeric_limits<float>::min();

    for (const auto &p : cloud)
    {
        if (p.x > xMax) xMax = p.x;
        if (p.x < xMin) xMin = p.x;
        if (p.y > yMax) yMax = p.y;
        if (p.y < yMin) yMin = p.y;
        if (p.z > zMax) zMax = p.z;
        if (p.z < zMin) zMin = p.z;
    }
}

void printPointcloudInfo(const PointCloud& cloud, const std::string& cloudName)
{
    float xMin, xMax, yMin, yMax, zMin, zMax;

    findPointcloudLimits(cloud, xMin, yMin, zMin, xMax, yMax, zMax);

    std::cout << "\nPointcloud: " << cloudName
              << "\n---------------------------------------"
              << "\nHeader: " << cloud.header
              << "Height: " << cloud.height
              << "\nWidth: " << cloud.width << std::setprecision(3)
              << "\nLimits x: (" << xMin << ", " << xMax << ")"
              << "\nLimits y: (" << yMin << ", " << yMax << ")"
              << "\nLimits z: (" << zMin << ", " << zMax << ")"
              << "\n";
}

}
