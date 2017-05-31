/******************************************************************************
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "utils/OctreeUtils.hh"

#include <gtest/gtest.h>
#include <cmath>

using namespace octomap_tools;

TEST(OctreeUtilsTest, CreateTransformationMatrixFromRPY)
{
    constexpr float x = 0.5, y = 1.5, z = -2.0;
    constexpr float ro = 1.0, pi = -1.0, yaw = 4.0;

    auto A = createTransformationMatrix(x, y, z, ro, pi, yaw);

    EXPECT_FLOAT_EQ(A(0,3), x);
    EXPECT_FLOAT_EQ(A(1,3), y);
    EXPECT_FLOAT_EQ(A(2,3), z);

    Eigen::Matrix3f Rx;
    Rx << 1, 0, 0, 0, cos(ro), -sin(ro), 0, sin(ro), cos(ro);

    Eigen::Matrix3f Ry;
    Ry << cos(pi), 0, sin(pi), 0, 1, 0, -sin(pi), 0, cos(pi);

    Eigen::Matrix3f Rz;
    Rz << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;

    auto B = Rx * Ry * Rz; // RPY -> rotations X Y Z

    for (size_t i = 0; i < 3; ++i)
        for (size_t j = 0; j < 3; ++j)
            EXPECT_NEAR(A(i,j), B(i,j), 1e-5);
}
