cmake_minimum_required(VERSION 3.1)
project(octomap_tools)

# Turn on tests with: cmake -Doption=ON
option(BUILD_TESTS "Build and run unit tests. " ON)
option(BUILD_BIN "Build binaries of tools." OFF)

#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package( octomap REQUIRED )
find_package( PCL REQUIRED )
find_package( Eigen3 REQUIRED )

include_directories(
    ${OCTOMAP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS}
)

link_libraries(${OCTOMAP_LIBRARIES})
link_libraries(${Eigen3_LIBRARIES})
link_libraries(${PCL_LIBRARIES})

if (BUILD_BIN)
    add_executable(pointcloud2octree src/pointcloud2octree.cpp)
    add_executable(pointcloud_splitter src/pointcloud_splitter.cpp)
    add_executable(pointcloud_info src/pointcloud_info.cpp)

    add_executable(octree2pointcloud
        src/octree2pointcloud.cpp
        src/OctreeUtils.cpp
    )

    add_executable(octree_splitter
        src/octree_splitter.cpp
        src/OctreeUtils.cpp
    )

    add_executable(octomap_merge src/octomap_merge.cpp
        src/OctomapMerger.cpp 
        src/OctreeUtils.cpp
    )
endif()

if (BUILD_TESTS)
    add_subdirectory(test)
endif()