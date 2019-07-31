cmake_minimum_required(VERSION 3.9)
aux_source_directory(. DIR_PNP_SRCS)
add_library(pnpSolver ${DIR_PNP_SRCS})