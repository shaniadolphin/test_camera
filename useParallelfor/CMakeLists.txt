cmake_minimum_required(VERSION 3.9)
set(CMAKE_CXX_STANDARD 11)
add_definitions(-g -o2)
project(testdemo)
find_package(OpenCV REQUIRED)
#include_directories(.)
add_executable(testdemo useParallelfor.cpp)
#add_executable(testcamera capture.c)
target_link_libraries(testdemo ${OpenCV_LIBS})
target_link_libraries(testdemo -lpthread -ltbb)