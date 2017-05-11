/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

/**
 * This tool splits octree (.ot) into two parts
 * The plane of split is determined by three points:
 * p1=(x1,y1,z1), p2=(x2,y2,z2), p3=(x3,y3,z3)
 *
 * Usage: <input_file.ot> <out1_file.ot> <out2_file.ot> x1 y1 z1 x2 y2 z2 x3 y3 z3
 *
 * Example:
 * ./pointcloud_splitter fr_079.ot out1.ot out2.ot 0 4 0 4 4 0 0 4 4
 * It'll split the octree through plane parallel to axis x at y equal to 4
 */

#include <cstdlib>
#include <iostream>

#include <Eigen/Dense>

#include "pointcloud_utils.h"
#include "octree_utils.h"

using namespace octomap_tools;
using namespace Eigen;

void printHelp(const char* progName)
{
    std::cout << "Usage: " << progName
            << " <in_file.ot> <out_file1.ot> <out_file2.ot>"
            << " x1 y1 z1 x2 y2 z2 x3 y3 z3\n"
            << "This program splits octree into two parts.\n"
            << "The plane of split is determined by three points.\n";
    exit(0);
}

int main(int argc, char** argv)
{
    if (argc != 13)
        printHelp(argv[0]);

    const std::string inFilename = argv[1];
    const std::string outFilename1 = argv[2];
    const std::string outFilename2 = argv[3];

    Matrix3f A;
    auto spaceSize = static_cast<size_t>(A.rows());

    for (size_t i = 0, j = 4; i < spaceSize; i++, j += spaceSize)
    {
        A.row(i) = Vector3f{
            std::strtof(argv[j], nullptr),
            std::strtof(argv[j+1], nullptr),
            std::strtof(argv[j+2], nullptr)};
    }

    const auto plane = calculatePlaneFromThreePoints(A);
    auto tree = readOctreeFromFile(inFilename);

    auto cloud = convertOctreeToPointcloud(*tree);
    printPointcloudInfo(cloud, inFilename);

    PointCloud::Ptr out1(new PointCloud);
    PointCloud::Ptr out2(new PointCloud);
    splitPointcloud(plane, cloud, *out1, *out2);

    printPointcloudInfo(*out1, outFilename1);
    printPointcloudInfo(*out2, outFilename2);

    writePointCloudAsOctreeToFile(out1, outFilename1);
    writePointCloudAsOctreeToFile(out2, outFilename2);

    return 0;
}
