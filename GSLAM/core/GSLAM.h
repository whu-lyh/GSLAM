/**
  This file defines data types shared by all SLAM systems.
  A SLAM system contains one active map for current tracking and some in-active maps after losted or loaded from history.
  A Map are generally constructed with the follow things:
  1. MapFrame : the keyframes (Mono,Stereo,RGBD .etc)
  2. MapPoint : the keypoints (not used in direct methods)
  3. BOWs     : for relocalization and loop closure
  4. Other data structures
 */

#ifndef GSLAM_H
#define GSLAM_H

#include <vector>
#include <map>

#define GSLAM_VERSION_MAJOR 2
#define GSLAM_VERSION_MINOR 4
#define GSLAM_VERSION_PATCH 7
#define GSLAM_VERSION (GSLAM_VERSION_MAJOR<<6|GSLAM_VERSION_MINOR<<4|GSLAM_VERSION_PATCH)
#define GSLAM_COMMAND_STRHELPER(COMMAND) #COMMAND
#define GSLAM_COMMAND_STR(COMMAND) GSLAM_COMMAND_STRHELPER(COMMAND)
#define GSLAM_VERSION_STR (GSLAM_COMMAND_STR(GSLAM_VERSION_MAJOR) "." \
                           GSLAM_COMMAND_STR(GSLAM_VERSION_MINOR) "." \
                           GSLAM_COMMAND_STR(GSLAM_VERSION_PATCH))

#include "SIM3.h"
#include "KeyPoint.h"
#include "GImage.h"
#include "Camera.h"
#include "Mutex.h"
#include "Svar.h"
#include "SharedLibrary.h"
#include "Map.h"
#include "SLAM.h"

#include <GSLAM/core/HashMap.h>
#include <GSLAM/core/Messenger.h>
#include <GSLAM/core/Application.h>
#include <GSLAM/core/Event.h>
#include "Optimizer.h"


#endif
