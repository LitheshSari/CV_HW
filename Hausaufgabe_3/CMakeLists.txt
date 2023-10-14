cmake_minimum_required(VERSION 3.15)
project(trace_target)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${Ceres_INCLUDE_DIRS})
include_directories(.)

add_subdirectory(kalman)
link_libraries(${OpenCV_LIBS})

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})