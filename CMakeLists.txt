cmake_minimum_required(VERSION 2.8)
project(SolveAXXB)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")



file(GLOB AXXB__SRC
    axxb/*
)

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(  ${EIGEN3_INCLUDE_DIRS} )


################
# AXXB Library #
################
add_library(SolveAXXB SHARED ${AXXB__SRC})



############
# Examples #
############
add_executable(test_utils examples/test_utils.cpp)
target_link_libraries(test_utils SolveAXXB     )

add_executable(demo1 examples/demo1.cpp)
target_link_libraries(demo1 SolveAXXB     )
