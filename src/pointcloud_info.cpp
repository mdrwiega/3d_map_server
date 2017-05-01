/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

/**
 * This tool shows information about pointcloud (.pcd)
 *
 * Usage: <input_file.pcd>
 */

#include <iostream>
#include <iomanip>
#include <limits>

#include "pointcloud_utils.h"

using namespace octomap_tools;

void printHelp(const char* progName)
{
    std::cout << "Usage: " << progName << " <file.pcd>"
              << "This program shows information about pointcloud.\n";
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

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        printHelp(argv[0]);
        return -1;
    }

    const std::string inFilename = argv[1];

    auto cloud = readPointCloudFromFile(inFilename);
    float xMin, xMax, yMin, yMax, zMin, zMax;

    findPointcloudLimits(*cloud, xMin, yMin, zMin, xMax, yMax, zMax);

    std::cout << "\nPointcloud: " << inFilename
              << "\n---------------------------------------"
              << "\nHeader: " << cloud->header
              << "Height: " << cloud->height
              << "\nWidth: " << cloud->width << std::setprecision(3)
              << "\nLimits x: (" << xMin << ", " << xMax << ")"
              << "\nLimits y: (" << yMin << ", " << yMax << ")"
              << "\nLimits z: (" << zMin << ", " << zMax << ")"
              << "\n" << 1;
    return 0;
}
