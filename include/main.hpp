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

#include "bruce_controller.hpp"