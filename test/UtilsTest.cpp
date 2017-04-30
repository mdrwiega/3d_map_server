/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "Utils.h"

#include <gtest/gtest.h>

using namespace md_utils;

TEST(Swap, SimplyCase_Int)
{
    int tab[] = {1, 2, 3};
    swap(tab[0], tab[2]);

    EXPECT_EQ(3, tab[0]);
    EXPECT_EQ(1, tab[2]);
}

TEST(Swap, OneElement_Int)
{
    int tab[] = {1};
    swap(tab[0], tab[0]);

    EXPECT_EQ(1, tab[0]);
}

TEST(Partition, SimplyCase_PivotAtEnd_Int)
{
    size_t size = 5;
    int data_init[] = {5,3,1,4,2};
    int expected[] = {1,2,5,4,3};

    EXPECT_EQ(size_t{1}, partition(data_init, 0, size-1, size-1));

    for (std::size_t i = 0; i < size; ++i)
        EXPECT_EQ(expected[i], data_init[i]);
}

TEST(Partition, SimplyCase_PivotInTheMiddle_Int)
{
    size_t size = 5;
    int data_init[] = {5,3,1,4,2};
    int expected[] = {1,3,2,4,5};

    EXPECT_EQ(size_t{0}, partition(data_init, 0, size-1, size/2));

    for (std::size_t i = 0; i < size; ++i)
        EXPECT_EQ(expected[i], data_init[i]);
}

TEST(QuickSort, SimplyCase_Int)
{
    size_t size = 7;
    int tab[] = {7,6,3,8,2,5,1,4};
    quickSort(tab, 0, size);

    for (std::size_t i = 0; i < size; ++i)
        EXPECT_EQ(static_cast<int>(i+1), tab[i]);
}

TEST(QuickSort, OneElement_Int)
{
    int tab[] = {1};
    quickSort(tab, 0, 0);

    EXPECT_EQ(1, tab[0]);
}
