cmake_minimum_required(VERSION 2.8.3)
project(vins)

############## Select on Pi or Ubuntu ##############
option (ENABLE_PI  "Enable PI." OFF)
####################################################
if (ENABLE_PI)
    option (ENABLE_MYNT_SDK "Enable SDK." ON)
    option (ENABLE_MYNT_ROS "Enable ROS driver." OFF)
else ()
    option (ENABLE_MYNT_SDK "Enable SDK." OFF)
    option (ENABLE_MYNT_ROS "Enable ROS driver." ON)
endif()


############## Select Release or Debug ################
set(COMPILE_TYPE Release)
if(COMPILE_TYPE STREQUAL "Release")
  set(CMAKE_BUILD_TYPE "Release")
  set(CMAKE_CXX_FLAGS "-std=c++17")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
elseif(COMPILE_TYPE STREQUAL "Debug")
  SET(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "-std=c++17")
  SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
else()
  message(FATAL_ERROR "COMPILE_TYPE is Error!")
endif()
  message(AUTHOR_WARNING "COMPILE_TYPE = ${COMPILE_TYPE}")

if (ENABLE_PI)
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-march=native")
endif()
####################################################
find_package(Threads REQUIRED)

##################### setting ######################
configure_file (
  "${PROJECT_SOURCE_DIR}/src/driver/config.h.in"
  "${PROJECT_BINARY_DIR}/config.h"
  )

# set(CMAKE_INCLUDE_CURRENT_DIR ON) #used to find config.h
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(MINIGLOG_PATH ${PROJECT_SOURCE_DIR}/src/miniglog)
###libminiglog.dylib for Mac and libminiglog.so for Linux in below line.
set(MINIGLOG_LIBS ${MINIGLOG_PATH}/build/lib/libminiglog.so) 
set(MINIGLOG_INCLUDE_DIRS ${MINIGLOG_PATH}/build/include)

if (ENABLE_MYNT_SDK)
  set(CAMERA_MODELS_PATH ${PROJECT_SOURCE_DIR}/../camera_models)
  set(CAMERA_MODELS_LIB ${CAMERA_MODELS_PATH}/build/libcamera_models.so) 
  set(CAMERA_MODELS_INCLUDE_DIRS ${CAMERA_MODELS_PATH}/include)
  include_directories(${CAMERA_MODELS_INCLUDE_DIRS})
endif()

####################################################

############## find relevant packages ##############
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
message(AUTHOR_WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")
####################################################

############## include_directories #################
include_directories(${CERES_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
include_directories(${MINIGLOG_INCLUDE_DIRS})
include_directories(${PROJECT_BINARY_DIR})#used to find config.h
####################################################
set(DRIVER_SOURCES 
    src/driver/adapter.cpp
    src/driver/driver.cpp
    src/driver/cv_painter.cpp
    src/driver/simulator.cpp)

################# set by switch ####################
if (ENABLE_MYNT_SDK)
  set(DRIVER_LIBS mynteye)

  set(DRIVER_SOURCES ${DRIVER_SOURCES} 
    src/driver/mynt_S1030.cpp
    src/utility/visual_sdk.cpp)

    set(BUILD_SHARED SHARED)

elseif(ENABLE_MYNT_ROS)
  catkin_package()
  find_package(catkin REQUIRED COMPONENTS
      roscpp
      std_msgs
      geometry_msgs
      nav_msgs
      tf
      cv_bridge
      camera_models
      image_transport)
  include_directories(${catkin_INCLUDE_DIRS})
  set(DRIVER_LIBS ${catkin_LIBRARIES})

  set(DRIVER_SOURCES ${DRIVER_SOURCES} 
  src/driver/mynt_S1030_ros.cpp
  src/utility/visual_ros.cpp
  src/utility/CameraPoseVisualization.cpp)

endif()
####################################################

############### Generate lib and exe ###############
add_library(vins_lib
    ${BUILD_SHARED}
    src/estimator/parameters.cpp
    src/estimator/estimator.cpp
    src/estimator/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projectionTwoFrameOneCamFactor.cpp
    src/factor/projectionTwoFrameTwoCamFactor.cpp
    src/factor/projectionOneFrameTwoCamFactor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    ${DRIVER_SOURCES}
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    src/featureTracker/feature_tracker.cpp)
target_link_libraries(vins_lib ${CAMERA_MODELS_LIB}  ${MINIGLOG_LIBS} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(vins_node src/main.cpp)
target_link_libraries(vins_node vins_lib ${DRIVER_LIBS}) 
