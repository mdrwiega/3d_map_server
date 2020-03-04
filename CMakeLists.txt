cmake_minimum_required(VERSION 2.8.11)
project(octomap_tools)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)
set(CATKIN_ENABLE_TESTING 1)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  cmake_modules
  sensor_msgs
  std_msgs
  octomap_msgs
  octomap_ros
  pcl_ros
  pcl_conversions
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS OCTOMAP EIGEN3 PCL
)

### FOR PROFILING (need to set after catkin_package)
#add_compile_options(-pg)
#set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OCTOMAP_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
	src/features_matching.cpp
	src/maps_integrator_visualizer.cpp
	src/maps_integrator.cpp
	src/thread_pool.cpp
	src/transformations.cpp
)

#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})

add_executable(octomap_tools_node src/octomap_tools_node.cpp)
target_link_libraries(octomap_tools_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})

# Additional tools
add_executable(octomap_box_filter
	src/tools/octomap_box_filter.cpp
)
target_link_libraries(octomap_box_filter ${PROJECT_NAME} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})

add_executable(octomap_statistical_filter
	src/tools/octomap_statistical_filter.cpp
)
target_link_libraries(octomap_statistical_filter ${PROJECT_NAME} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})

add_executable(octomap_info
	src/tools/octomap_info.cpp
)
target_link_libraries(octomap_info ${PROJECT_NAME} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})

add_executable(radius_outlier_removal
	src/tools/radius_outlier_removal.cpp
)
target_link_libraries(radius_outlier_removal ${PROJECT_NAME} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})


if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(octomaps_integrator_test test/octomaps_integrator.test test/octomaps_integrator_test.cpp test/test_octomap_tools.cpp)
    target_link_libraries(octomaps_integrator_test ${PROJECT_NAME} ${catkin_LIBRARIES})

    set(TESTS_SRC
        test/test_octomap_tools.cpp
        test/transformations_test.cpp
        test/octomaps_integrator_test.cpp
        test/conversions_test.cpp
        test/feature_cloud_test.cpp
        test/features_matching_test.cpp
    )
    catkin_add_gtest(${PROJECT_NAME}_test ${TESTS_SRC})
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
endif()