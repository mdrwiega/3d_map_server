cmake_minimum_required(VERSION 2.8.11)
project(octomap_tools)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_BUILD_TYPE Release)
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
	src/octomap_tools.cpp
	src/transformations.cpp
	src/utils.cpp
	src/maps_integrator.cpp
	src/maps_integrator_visualizer.cpp
	src/thread_pool.cpp
)

#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})

add_executable(octomap_tools_node src/octomap_tools_node.cpp)
target_link_libraries(octomap_tools_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
    set(TESTS_SRC 
        test/main.cpp
#        test/transformations_test.cpp
        test/features_matching_test.cpp
    )
    catkin_add_gtest(${PROJECT_NAME}_test ${TESTS_SRC})
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
endif()