##################################################
#
# CMakeLists.txt - Root cmake file for BelugaTracker
#
# History:
#   * Original file - Dan Swain, 7/6/10
#   * Simplied significantly - DTS 7/14/10
#   * Modified from SimpleBWTracker - DTS 8/26/10
#   * Modified from KilliTracker - DTS 9/10/10
#   * Remove BelugaServer (using ruby IPC now)
#   
##################################################

# this is the most recent version of CMake
cmake_minimum_required (VERSION 2.8)

# name of the project - this has to come first
project(BelugaTracker)

# MADTRaC information
set(MT_ROOT "../MADTraC/build" CACHE PATH "MADTraC root directory")
# sets variables related to MT
include(${MT_ROOT}/cmake/MT_Config.cmake)

# set the name of the built target
set(APP_NAME BelugaTracker)

# set this to a list of source and header files to be compiled
set(APP_SRC
  src/BelugaTrackerGUI.cpp
  src/BelugaTrackerGUI.h
  src/BelugaTracker.cpp
  src/BelugaTracker.h
  src/BelugaRobot.cpp
  src/BelugaRobot.h
  src/BelugaVideoSetupDialog.h
  src/BelugaVideoSetupDialog.cpp
  src/CalibrationDataFile.h
  src/CalibrationDataFile.cpp
  src/CoordinateTransforms.h
  src/CoordinateTransforms.cpp
  src/TrackerExtras.h
  src/TrackerExtras.cpp
  src/BelugaDynamics.h
  src/BelugaDynamics.cpp
  src/BelugaConstants.h
  src/BelugaControl.cpp
  src/BelugaControl.h
  src/ControlLaw.cpp
  src/ControlLaw.h
  src/Controller.cpp
  src/Controller.h
  src/BelugaIPCClient.cpp
  src/BelugaIPCClient.h )

# adds src to the header search path
include_directories(src)

# sets up the build target
add_executable(${APP_NAME} WIN32 MACOSX_BUNDLE ${APP_SRC})

# sets up the app to build correctly using the right libraries, etc
MT_robot_app(${APP_NAME})
