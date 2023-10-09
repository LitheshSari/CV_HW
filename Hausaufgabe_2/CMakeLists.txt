CMAKE_MINIMUM_REQUIRED(VERSION 3.15)
PROJECT(rm_vision)


find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})

link_libraries(${OpenCV_LIBS})

add_executable(camera src/camera.cpp)
add_executable(calibrate src/calibrate.cpp)
add_executable(video src/video.cpp)
