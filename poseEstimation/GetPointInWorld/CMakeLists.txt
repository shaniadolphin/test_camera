cmake_minimum_required(VERSION 3.9)
project(GetPointInWorld)
set(CMAKE_CXX_STANDARD 11)
add_definitions(-g -o2)
find_package(OpenCV REQUIRED)
include_directories(./ ../PNPSolver)#添加多目录的路径

# Add block directories
set(PNPSOLVER ${CMAKE_SOURCE_DIR}/../PNPSolver)
if( IS_DIRECTORY ${PNPSOLVER} )
    # 第二个cassdk.out参数用于指定外部文件夹在输出文件夹中的位置
    add_subdirectory( ${PNPSOLVER} pnpSolver.out)
else()
    message(FATAL_ERROR   "INVALID FOLDER 'PNPSOLVER'=${PNPSOLVER}" )
endif()

#aux_source_directory(. source_list)
add_executable(GetPointInWorld main1.cpp)
#add_executable(testcamera capture.c)
target_link_libraries(GetPointInWorld ${OpenCV_LIBS})
target_link_libraries(GetPointInWorld pnpSolver)