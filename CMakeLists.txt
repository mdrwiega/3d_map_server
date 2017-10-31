cmake_minimum_required(VERSION 3.1)
project(octomap_tools)

# Turn on tests with: cmake -Doption=ON
option(BUILD_TESTS "Build and run unit tests. " ON)
option(BUILD_BIN "Build binaries of tools." OFF)

#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -Wall ${CMAKE_CXX_FLAGS}")

set(SRC_DIR "src")
set(TOOLS_DIR ${SRC_DIR}/tools)

#set(Boost_USE_STATIC_LIBS        ON)

find_package(octomap 1.8.1 REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package( Boost REQUIRED)

# The library can be found on https://github.com/mdrwiega/md_utils
find_library(mdUtils REQUIRED)

include_directories(
    ${SRC_DIR}
    ${OCTOMAP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS}
#    ${Boost_INCLUDE_DIRS}
)

link_libraries(${OCTOMAP_LIBRARIES})
link_libraries(${Eigen3_LIBRARIES})
link_libraries(${PCL_LIBRARIES})
#link_libraries(${Boost_LIBRARIES})
find_package(OpenCV REQUIRED )
link_libraries(${OpenCV_LIBS})

if (BUILD_BIN)
    add_executable(pointcloud2octree
        ${TOOLS_DIR}/pointcloud2octree.cpp
    )
    add_executable(pointcloud_splitter
        ${TOOLS_DIR}/pointcloud_splitter.cpp
    )
    add_executable(pointcloud_info
        ${TOOLS_DIR}/pointcloud_info.cpp
    )
    add_executable(octree2pointcloud
        ${TOOLS_DIR}/octree2pointcloud.cpp
        ${SRC_DIR}/utils/OctreeUtils.cpp
    )
    add_executable(octree_splitter
        ${TOOLS_DIR}/octree_splitter.cpp
        ${SRC_DIR}/utils/OctreeUtils.cpp
    )
    add_executable(find_octrees_tf
        ${TOOLS_DIR}/find_octrees_tf.cpp
        ${SRC_DIR}/OctomapMerger.cpp 
        ${SRC_DIR}/utils/OctreeUtils.cpp
    )
    add_executable(octomap_merge
        ${TOOLS_DIR}/octomap_merge.cpp
        ${SRC_DIR}/OctomapMerger.cpp
        ${SRC_DIR}/utils/OctreeUtils.cpp
        ${SRC_DIR}/utils/octree_transformations.cpp
    )
endif()

if (BUILD_TESTS)
    add_subdirectory(test)
endif()