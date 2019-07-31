cmake_minimum_required(VERSION 2.8)
aux_source_directory(. DIR_CMAKE_SRCS)
add_library(cmake ${DIR_CMAKE_SRCS})