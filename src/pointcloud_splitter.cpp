/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

/**
 * This tool splits pointcloud (.pcd) into two parts
 * The plane of split is determined by three points:
 * p1=(x1,y1,z1), p2=(x2,y2,z2), p3=(x3,y3,z3)
 *
 * Usage: <input_file.ot> <output_file.pcd> x1 y1 z1 x2 y2 z2 x3 y3 z3
 */

#include <cstdlib>
#include <iostream>

#include <Eigen/Dense>

#include "pointcloud_utils.h"

using namespace octomap_tools;
using namespace Eigen;

void printHelp(const char* progName)
{
    std::cout << "Usage: " << progName
            << " <in_file.pcd> <out_file1.pcd> <out_file2.pcd>"
            << " x1 y1 z1 x2 y2 z2 x3 y3 z3\n"
            << "This program splits pointcloud into two parts.\n"
            << "The plane of split is determined by three points.\n";
    exit(0);
}

/**
 * Checks if 3D points in matrix (each point in row) are collinear
 */
bool pointsAreCollinear(Matrix3f& points)
{
    Vector3f ab = points.row(1) - points.row(0);
    Vector3f ac = points.row(2) - points.row(0);
    return ab.cross(ac) == Vector3f{0,0,0};
}

void splitPointcloud(const PointCloud& in, PointCloud& out1, PointCloud& out2)
{

    for (const auto& i : in.points)
    {


    }
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

    if (pointsAreCollinear(A))
    {
        std::cerr << "\nPoints are collinear. Please choose other points.\n";
        std::exit(-1);
    }

    Vector3f x = A.colPivHouseholderQr().solve(Vector3f{-1,-1,-1});
    std::cout << "\nThe split plane:\n"
              << x[0] << " x + " << x[1] << " y + " << x[2] << " z + 1 = 0\n";

    auto cloud = readPointCloudFromFile(inFilename);



    return 0;
}
