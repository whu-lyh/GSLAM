cmake_minimum_required(VERSION 2.8.6)
project(GSLAM)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include(PICMake)
include(GSLAMVersion)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)#Debug
ENDIF()

if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNOMINMAX")
endif()

#####################################################################
# Add targets
message("-- Compiling GSLAM Version ${GSLAM_VERSION}.")
message("-- Build type: " ${CMAKE_BUILD_TYPE})

include_directories(.)

pi_add_target(gslam BIN GSLAM/gslam REQUIRED System)
pi_add_target(gslam_qviz SHARED GSLAM/plugins/qviz REQUIRED System Qt OpenGL)
pi_add_target(gslam_rviz SHARED GSLAM/plugins/rviz REQUIRED System ROS)
pi_add_target(gslam_tests SHARED GSLAM/plugins/tests REQUIRED System MODULES OpenCV)
pi_add_target(gslam_evaluate SHARED GSLAM/plugins/evaluate REQUIRED System)

pi_add_target(gslam_optimizer SHARED GSLAM/plugins/optimizer_ceres REQUIRED System Eigen3 Ceres GLOG)
pi_add_target(gslam_estimator SHARED GSLAM/plugins/estimator_opencv REQUIRED OpenCV)

pi_add_target(gslamDB_euroc SHARED GSLAM/plugins/Datasets/DatasetEuroc.cpp REQUIRED OpenCV)
pi_add_target(gslamDB_kitti SHARED GSLAM/plugins/Datasets/DatasetKITTI.cpp REQUIRED OpenCV)
pi_add_target(gslamDB_npudronemap SHARED GSLAM/plugins/Datasets/DatasetNPUDroneMap.cpp)
pi_add_target(gslamDB_npurgbd SHARED GSLAM/plugins/Datasets/DatasetNPURGBD.cpp REQUIRED OpenCV)
pi_add_target(gslamDB_cvmono SHARED GSLAM/plugins/Datasets/DatasetOpenCVMono.cpp REQUIRED OpenCV)
pi_add_target(gslamDB_rtm SHARED GSLAM/plugins/Datasets/DatasetRTMapper.cpp REQUIRED Qt)
pi_add_target(gslamDB_tummono SHARED GSLAM/plugins/Datasets/DatasetTUMMono.cpp REQUIRED OpenCV)
pi_add_target(gslamDB_tumrgbd SHARED GSLAM/plugins/Datasets/DatasetTUMRGBD.cpp REQUIRED OpenCV)

pi_report_target()

#####################################################################
# Now do make install
get_property(TARGETS2COMPILE GLOBAL PROPERTY TARGETS2COMPILE)
pi_install(HEADERS GSLAM/core HEADER_DESTINATION include/GSLAM
           TARGETS ${TARGETS2COMPILE} CMAKE "${PROJECT_SOURCE_DIR}/cmake/FindGSLAM.cmake.in")

