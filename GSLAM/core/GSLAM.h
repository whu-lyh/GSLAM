#ifndef GSLAM_CORE_H
#define GSLAM_CORE_H

#define GSLAM_VERSION_MAJOR 3
#define GSLAM_VERSION_MINOR 0
#define GSLAM_VERSION_PATCH 0
#define GSLAM_VERSION (GSLAM_VERSION_MAJOR<<6|GSLAM_VERSION_MINOR<<4|GSLAM_VERSION_PATCH)
#define GSLAM_COMMAND_STRHELPER(COMMAND) #COMMAND
#define GSLAM_COMMAND_STR(COMMAND) GSLAM_COMMAND_STRHELPER(COMMAND)
#define GSLAM_VERSION_STR (GSLAM_COMMAND_STR(GSLAM_VERSION_MAJOR) "." \
                           GSLAM_COMMAND_STR(GSLAM_VERSION_MINOR) "." \
                           GSLAM_COMMAND_STR(GSLAM_VERSION_PATCH))
// System Basic
#include "Glog.h"
#include "ThreadPool.h"
#include "Random.h"
#include "Timer.h"
#include "RingBuffer.h"
#include "VecParament.h"

// Interface
#include "Svar.h"
#include "Messenger.h"
#include "SharedLibrary.h"
#include "Application.h"
#include "FileResource.h"

#include "Matrix.h"
#include "Point.h"
#include "KeyPoint.h"
#include "SO3.h"
#include "SE3.h"
#include "SIM3.h"

#include "GImage.h"
#include "Camera.h"
#include "Undistorter.h"

#include "GPS.h"
#include "Map.h"
#include "VideoFrame.h"
#include "HashMap.h"
#include "Dataset.h"

#include "TileProjection.h"
#include "TileManager.h"

#include "SLAM.h" // deprecated
#include "MapFusion.h"// deprecated

#endif
