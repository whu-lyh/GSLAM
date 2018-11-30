#ifndef GSLAM_CORE_H
#define GSLAM_CORE_H

#define GSLAM_VERSION_MAJOR 2
#define GSLAM_VERSION_MINOR 4
#define GSLAM_VERSION_PATCH 7
#define GSLAM_VERSION (GSLAM_VERSION_MAJOR<<6|GSLAM_VERSION_MINOR<<4|GSLAM_VERSION_PATCH)
#define GSLAM_COMMAND_STRHELPER(COMMAND) #COMMAND
#define GSLAM_COMMAND_STR(COMMAND) GSLAM_COMMAND_STRHELPER(COMMAND)
#define GSLAM_VERSION_STR (GSLAM_COMMAND_STR(GSLAM_VERSION_MAJOR) "." \
                           GSLAM_COMMAND_STR(GSLAM_VERSION_MINOR) "." \
                           GSLAM_COMMAND_STR(GSLAM_VERSION_PATCH))
// System Basic
#include "Glog.h"
#include "FileResource.h"
#include "ThreadPool.h"
#include "Random.h"
#include "SharedLibrary.h"
#include "Timer.h"
#include "RingBuffer.h"

#include "CPUMetric.h"
#include "MemoryMetric.h"

// Interface
#include "Svar.h"
#include "Messenger.h"
#include "Application.h"

#include "VecParament.h"
#include "GImage.h"
#include "Camera.h"
#include "Undistorter.h"

#include "Point.h"
#include "Matrix.h"
#include "SO3.h"
#include "SE3.h"
#include "SIM3.h"
#include "KeyPoint.h"

#include "Map.h"
#include "VideoFrame.h"
#include "GPS.h"

#include "TileManager.h"
#include "TileProjection.h"
#include "MapFusion.h"

#include "Dataset.h"

#include "SLAM.h"


#endif
