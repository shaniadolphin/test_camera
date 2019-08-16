cmake_minimum_required(VERSION 2.8)
project(testcamera)
find_package(OpenCV REQUIRED)

add_executable(testcamera capture.cpp)
#add_executable(testcamera capture.c)
target_link_libraries(testcamera ${OpenCV_LIBS})
