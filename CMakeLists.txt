cmake_minimum_required(VERSION 2.8.3)
project(ros_gmsl_custom)

# Compiler Setting
find_package(CUDA REQUIRED)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
set(CMAKE_CXX_STANDARD 11)

# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# set(CUDA_PROPAGATE_HOST_FLAGS OFF)
# set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11 -default-stream=per-thread")

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(CUDA_PROPAGATE_HOST_FLAGS OFF)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11 -O3 -default-stream=per-thread")

enable_language("CUDA")

add_definitions(-DVIBRANTE)
add_definitions(-DVIBRANTE_V5L)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
)

configure_file(px2Src/framework/DataPath.hpp.in ${CMAKE_SOURCE_DIR}/px2Src/framework/DataPath.hpp)

set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    src
    px2Src
    px2Src/glfw-3.2.1/include
    /usr/local/driveworks/include
    /usr/local/driveworks/include/dw/sensors/camera
    /usr/local/driveworks/include/dw/image
)

link_directories(
    /usr/local/lib
    /usr/local/cuda/lib64
    /usr/local/driveworks/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/px2Src/glfw-3.2.1/build/src
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
)

# ------------------------
# Node n_cameras_driveNet

file(GLOB n_cameras_driveNet_HEADERS
    px2Src/*.h
    px2Src/*.hpp
    px2Src/framework/*.h
    px2Src/framework/*.hpp
    src/*.h
    src/*.hpp
)

file(GLOB n_cameras_driveNet_SOURCES
    px2Src/*.cpp
    px2Src/*.cu
    px2Src/framework/*.cpp
    px2Src/framework/*.cu
    src/*.cpp
    src/*.cu
)

set(n_cameras_driveNet_LIBS
    nvmedia
    glfw
    EGL
    driveworks
    cudart
    drm
    GLESv2
    nvinfer
    nvcaffe_parser
    pthread
    nppc
    nppial
    nppicc
    nppicom
    nppidei
    nppif
    nppig
    nppim
    nppist
    nppitc
    npps
)

CUDA_ADD_EXECUTABLE(n_cameras_driveNet
    ${n_cameras_driveNet_HEADERS}
    ${n_cameras_driveNet_SOURCES}
)

target_link_libraries(n_cameras_driveNet
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    ${CUDA_LIBS}
    ${n_cameras_driveNet_LIBS}
)

target_compile_options(n_cameras_driveNet PRIVATE)

add_dependencies(n_cameras_driveNet
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

# ------------------------