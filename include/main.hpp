/**
 * @file: main.hpp
 * @brief: Main header file for the project
 * @author: Jun Young Kim
 */

// Standard Libraries
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

// Added Packages
#include <Eigen/Dense>

#include <mujoco/mujoco.h>
#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #if defined(__APPLE__)
    #include <mach-o/dyld.h>
  #endif
    #include <dirent.h>
    #include <dlfcn.h>
    #include <sys/errno.h>
    #include <unistd.h>
#endif
}

namespace {
    namespace mj = ::mujoco;
    namespace mju = ::mujoco::sample_util;

    // using ::mujoco::Glfw;

    // Constants
    const double syncMisalign = 0.1;        // Maximum mis-alignment before re-sync (simulation seconds)
    const double simRefreshFraction = 0.7;  // Fraction of refresh available for simulation
    const int kErrorLength = 1024;          // Load error string length

    // Model and Data
    mjModel* m = nullptr;
    mjData* d = nullptr;

    // Control noise variables
    mjtNum* ctrlnoise = nullptr;

    using Seconds = std::chrono::duration<double>;
}

#include "bruce_controller.hpp"
BRUCEController bruce_controller;