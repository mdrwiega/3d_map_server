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

#include "utils/PointcloudUtils.h"

using namespace octomap_tools;

void printHelp(const char* progName)
{
    std::cout << "Usage: " << progName << " <file.pcd>"
              << "This program shows information about pointcloud.\n";
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

    printPointcloudInfo(*cloud, inFilename);

    return 0;
}
