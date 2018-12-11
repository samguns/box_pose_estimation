cmake_minimum_required(VERSION 3.8)
project(box_pose_estimation)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.7 REQUIRED REQUIRED COMPONENTS io registration visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(box_pose_estimation main.cpp png2pcd.cpp)
target_link_libraries(box_pose_estimation ${PCL_LIBRARIES})