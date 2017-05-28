cmake_minimum_required(VERSION 3.1)
project(table_printer)

find_package( Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

add_library(table_printer INTERFACE)
target_include_directories(table_printer INTERFACE src/TablePrinter.hh)
target_link_libraries(table_printer INTERFACE ${Boost_LIBRARIES})