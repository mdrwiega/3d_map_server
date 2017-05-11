cmake_minimum_required(VERSION 3.1)
project(octomap_tools)

# Turn on tests with: cmake -Doption=ON
option(test "Build and run unit tests." OFF)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package( octomap REQUIRED )
find_package( PCL REQUIRED )
find_package( Eigen3 REQUIRED )

include_directories(include ${catkin_INCLUDE_DIRS} 
    ${OCTOMAP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS}
)

link_libraries(${OCTOMAP_LIBRARIES})
link_libraries(${Eigen3_LIBRARIES})
link_libraries(${PCL_LIBRARIES})

#add_library(ClassNodelet src/class_nodelet.cpp)
#target_link_libraries(ClassNodelet ClassNode ${catkin_LIBRARIES})

add_executable(octomap_tools src/main.cpp)
target_link_libraries(octomap_tools ${catkin_LIBRARIES})

add_executable(octree2pointcloud src/octree2pointcloud.cpp)

add_executable(pointcloud2octree src/pointcloud2octree.cpp)

add_executable(pointcloud_splitter src/pointcloud_splitter.cpp)

add_executable(octree_splitter src/octree_splitter.cpp)

add_executable(pointcloud_info src/pointcloud_info.cpp)

if (test)
#    add_subdirectory(test)
endif()