cmake_minimum_required(VERSION 2.8)
project(demo)
find_package(OpenCV REQUIRED)

add_executable(demo text_segementation.cpp)

target_link_libraries(demo ${OpenCV_LIBS})
