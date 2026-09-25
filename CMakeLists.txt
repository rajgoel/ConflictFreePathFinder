cmake_minimum_required(VERSION 3.24)

project(PathFinder
  VERSION 0.1.0
  DESCRIPTION "Collaborative multi-agent path finder with deadlock avoidance"
  LANGUAGES CXX
)

option(PATHFINDER_BUILD_TESTS "Build the PathFinder unit tests" ${PROJECT_IS_TOP_LEVEL})

add_library(PathFinder INTERFACE)
add_library(PathFinder::PathFinder ALIAS PathFinder)

target_include_directories(PathFinder
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(PathFinder INTERFACE cxx_std_20)

if ( PATHFINDER_BUILD_TESTS )
  enable_testing()
  add_subdirectory(tests)
endif()
