# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Define project name
project(demo)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
#set(OpenCV_DIR "/mnt/h/proj/opencv/opencv-4.1.0/release")
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
#message(STATUS "OpenCV library status:")
message(STATUS "OpenCV_DIR config: ${OpenCV_DIR}")
message(STATUS "OpenCV_VERSION: ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

# Declare the executable target built from your sources
add_executable(demo opticalFlowAlgorithm_hs.cpp)

# Link your application with OpenCV libraries
# target_link_libraries(calibrate ${OpenCV_LIBS})
target_link_libraries(demo -lopencv_core -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -lopencv_flann -lopencv_videoio)
target_link_libraries(demo -lpthread -lm  -ljpeg -lstdc++)

# ldd capture查看链接到的库