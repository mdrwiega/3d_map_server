/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <cstdlib>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace octomap_tools {

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

PointCloud::Ptr readPointCloudFromFile(const std::string fileName)
{
    PointCloud::Ptr cloud (new PointCloud);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file %s \n", fileName.c_str());
        std::exit (-1);
    }
    std::cout << "Loaded " << cloud->width * cloud->height
            << " data points from " << fileName << std::endl;
    return cloud;
}

}
