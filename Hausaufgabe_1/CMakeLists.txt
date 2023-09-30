CMAKE_MINIMUM_REQUIRED(VERSION 3.15)
PROJECT(rm_vision)


find_package(OpenCV REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

add_executable(apple src/apple.cpp)
add_executable(plates src/plates.cpp)
