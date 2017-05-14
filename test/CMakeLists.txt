cmake_minimum_required(VERSION 2.8)

find_package(Threads REQUIRED)

include(ExternalProject)
 
ExternalProject_Add(
    gtest
    URL https://github.com/google/googletest/archive/master.zip
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gtest
    INSTALL_COMMAND ""
)
 
ExternalProject_Get_Property(gtest source_dir binary_dir)
 
add_library(libgtest IMPORTED STATIC GLOBAL)
add_dependencies(libgtest gtest)

# Set libgtest properties
set_target_properties(libgtest PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/googlemock/gtest/libgtest.a"
    "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
)

set(SRC_DIR "../src")

include_directories(
    "${source_dir}/googletest/include"
    ${GTEST_INCLUDE_DIRS}
    ${SRC_DIR}
)

enable_testing()

set(TESTED_SRC
    ${SRC_DIR}/OctomapMerger.cpp    
    ${SRC_DIR}/OctreeUtils.cpp
)

file(GLOB TESTS_SRC "*.cpp")

add_executable(octomapToolsTests ${TESTS_SRC} ${TESTED_SRC})

target_link_libraries(octomapToolsTests libgtest)
add_test(NAME octomapToolsTests COMMAND octomapToolsTests)