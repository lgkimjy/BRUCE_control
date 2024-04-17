# Minimum required CMake version
cmake_minimum_required(VERSION 3.16)

# Project name
project(
    BRUCE_control
    VERSION 0.0.0
    DESCRIPTION "CMake Project Template"
    LANGUAGES CXX
)

#################################
# Open Libraries ADD here
#################################
option(MUJOCO_BUILD_EXAMPLES "Build samples for MuJoCo" ON)
option(MUJOCO_BUILD_SIMULATE "Build simulate library for MuJoCo" ON)
option(MUJOCO_BUILD_TESTS "Build tests for MuJoCo" OFF)
option(MUJOCO_TEST_PYTHON_UTIL "Build and test utility libraries for Python bindings" ON)
add_subdirectory(${PROJECT_SOURCE_DIR}/ext/mujoco)                             # Add subdirectory
add_subdirectory(${PROJECT_SOURCE_DIR}/ext/yaml-cpp)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/libraries/mujoco/sample/cmake") # CMake Path append
#################################
# Open Libraries ENDS here
#################################


find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND AND pinocchio_FOUND)
  message(STATUS "[Eigen3] Found Eigen3 library.")
  message(STATUS "[pinocchio] Found pinocchio library.")
  message(STATUS "EIGEN DIR : " ${EIGEN3_INCLUDE_DIR})
else()
  message(STATUS "[Eigen3] Cannot find a package.")
  message(STATUS "[pinocchio] Cannot find a package.")
endif()


# Set Compiler flags & options
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)        # LTO option
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-no-as-needed -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")           # -Wall -Wextra -Wpedantic -Werror")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -march=native")   # -Ofast, -O3, -O2, -O1, -Os


# Add include directories
include_directories(
    include
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    ${EIGEN3_INCLUDE_DIR}
    DEPENDS system_lib
)


# Add source files
set(WBC_SRC
    ${CMAKE_SOURCE_DIR}/src/WBC/wbc_base.cpp
    ${CMAKE_SOURCE_DIR}/src/WBC/weighted_wbc.cpp
    ${CMAKE_SOURCE_DIR}/src/WBC/wbic.cpp
    ${CMAKE_SOURCE_DIR}/src/WBC/hierarchical_wbc.cpp
)
# Add executable target
add_executable(bruce_controller
    src/main.cpp
    src/bruce_controller.cpp
    ${WBC_SRC}
)
# Set include directories
target_include_directories(bruce_controller PUBLIC 
    ${CMAKE_SOURCE_DIR}/include
)
# Link Eigen3 library
target_link_libraries(bruce_controller PRIVATE 
    Eigen3::Eigen
    yaml-cpp
    libsimulate
    mujoco::mujoco
    glfw
    Threads::Threads
    pinocchio::pinocchio
)

############################
######## Unit tests ########
############################
# cmake -DBUILD_TESTING=ON ..
if(BUILD_TESTING)
    find_package(GTest CONFIG REQUIRED) # brew install googletest (macOS)
    include(GoogleTest)
    add_executable(tests 
        tests/test_main.cpp
        tests/test_pino.cpp
        ${WBC_SRC}
    )
    target_link_libraries(tests 
        GTest::gtest
    )
endif()