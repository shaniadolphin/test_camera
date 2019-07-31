cmake_minimum_required(VERSION 2.8)
project(hellocmake)

# Add the source in project root directory
aux_source_directory(. DIRSRCS)
# Add header file include directories
include_directories(./ ../hello ../cmake)
# Add block directories
set(HELLODIR ${CMAKE_SOURCE_DIR}/../hello)
if( IS_DIRECTORY ${HELLODIR} )
    # 第二个cassdk.out参数用于指定外部文件夹在输出文件夹中的位置
    add_subdirectory( ${HELLODIR} hello.out)
else()
    message(FATAL_ERROR   "INVALID FOLDER 'HELLODIR'=${HELLODIR}" )
endif()

set(CMAKEDIR ${CMAKE_SOURCE_DIR}/../cmake)
if( IS_DIRECTORY ${CMAKEDIR} )
    # 第二个cassdk.out参数用于指定外部文件夹在输出文件夹中的位置
    add_subdirectory( ${CMAKEDIR} cmake.out)
else()
    message(FATAL_ERROR   "INVALID FOLDER 'CMAKEDIR'=${CMAKEDIR}" )
endif()

# Target
add_executable(hellocmake ${DIRSRCS})
target_link_libraries(hellocmake hello cmake)
