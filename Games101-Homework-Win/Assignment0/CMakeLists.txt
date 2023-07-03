cmake_minimum_required(VERSION 3.2)
project (Transformation)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)


set(Eigen3_DIR D:/project/myProject/USTC_CG/Homeworks/1_MiniDraw/eigen/install/share/eigen3/cmake)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
# message(STATUS ${EIGEN3_INCLUDE_DIRS})
add_executable (Transformation main.cpp)
