cmake_minimum_required(VERSION 2.8.3)
project(depthsense_cam)

## Find catkin macros and libraries
## Options
option( DEBUG_ACTIVE "Enable Debug build" ON )

if(DEBUG_ACTIVE)
    MESSAGE( "Debug compilation active" )
    set(ROS_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Debug)
else()
    MESSAGE( "Release compilation active" )
    set(ROS_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Release)
endif()

###
## CUDA
find_package(CUDA)
if(CUDA_FOUND)
    try_run(RUN_RESULT_VAR COMPILE_RESULT_VAR
        ${CMAKE_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/cuda_test/has_cuda_gpu.c
        CMAKE_FLAGS
            -DINCLUDE_DIRECTORIES:STRING=${CUDA_TOOLKIT_INCLUDE}
            -DLINK_LIBRARIES:STRING=${CUDA_CUDART_LIBRARY}
        COMPILE_OUTPUT_VARIABLE COMPILE_OUTPUT_VAR
        RUN_OUTPUT_VARIABLE RUN_OUTPUT_VAR)
    message("${RUN_OUTPUT_VAR}") # Display number of GPUs found
    # COMPILE_RESULT_VAR is TRUE when compile succeeds
    # RUN_RESULT_VAR is zero when a GPU is found
    if(COMPILE_RESULT_VAR AND NOT RUN_RESULT_VAR)
        set(CUDA_HAVE_GPU TRUE CACHE BOOL "Whether CUDA-capable GPU is present")
    else()
        set(CUDA_HAVE_GPU FALSE CACHE BOOL "Whether CUDA-capable GPU is present")
    endif()
endif(CUDA_FOUND)

###
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_transport
  sensor_msgs
  std_msgs
  roscpp
  tf
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    geometry_msgs
    image_transport
)
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

# INCLUDES
set(depthsense_INC_PATH
    src/driver/include/
)

set(depthsense_INC_DIRS
    src/
    ${depthsense_INC_PATH}
    /opt/softkinetic/DepthSenseSDK/include
)

# Specify locations of header files.
include_directories(
	${catkin_INCLUDE_DIRS}
	${depthsense_INC_DIRS}
)
###
# CUDA SOURCES
if(CUDA_HAVE_GPU)
    add_definitions(-DWITH_CUDA)

    set(cuda_SRC_DIR src/driver/cuda)

    set(cuda_SRC ${cuda_SRC_DIR}/)

    SET(CUDA_NVCC_FLAGS "-arch=sm_13" CACHE STRING "nvcc flags" FORCE)

    #HW capability 1.3
    SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)

    SET(LIB_TYPE STATIC) #set the lib type
    CUDA_ADD_LIBRARY(Cuda2Dscan ${LIB_TYPE} ${cuda_SRC} )
endif(CUDA_HAVE_GPU)

###
# SOURCES
set(depthsense_SRC
    src/depthsense_cam_node.cpp
    src/driver/src/depthsense_cam_driver.cpp
)

set(depthsense_HDR
    ${depthsense_INC_PATH}/depthsense_cam_driver.h
)

###
# EXECUTABLE
add_executable(
  depthsense_cam_node
  ${depthsense_HDR}
  ${depthsense_SRC}
)

find_library(DEPTHSENSE_LIBRARY DepthSense DepthSensePlugins turbojpeg /opt/softkinetic/DepthSenseSDK/lib/)

if(CUDA_HAVE_GPU)
    add_dependencies(depthsense_cam_node Cuda2Dscan)
    target_link_libraries(depthsense_cam_node
        ${catkin_LIBRARIES} ${DEPTHSENSE_LIBRARY} Cuda2Dscan)
else()
    target_link_libraries(
        depthsense_cam_node
        ${catkin_LIBRARIES} ${DEPTHSENSE_LIBRARY}
    )
endif(CUDA_HAVE_GPU)
###


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
