cmake_minimum_required(VERSION 2.8)

aux_source_directory(. DIR_HELLO_SRCS)
add_library(hello ${DIR_HELLO_SRCS})