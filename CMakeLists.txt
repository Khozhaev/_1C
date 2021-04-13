cmake_minimum_required(VERSION 3.17)
project(problem)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED )
set(SOURCE_FILES main.cpp)
add_executable(problem ${SOURCE_FILES})
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(problem ${OpenCV_LIBS} )