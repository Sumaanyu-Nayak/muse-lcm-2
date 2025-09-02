cmake_minimum_required(VERSION 3.10)
project(attitude_estimator_lcm)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

# Find LCM
pkg_check_modules(LCM REQUIRED lcm)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/msgs
    ${EIGEN3_INCLUDE_DIR}
    ${LCM_INCLUDE_DIRS}
)

# Add compiler flags
add_compile_options(-Wall -Wextra -O2)

# Link directories
link_directories(${LCM_LIBRARY_DIRS})

# Collect source files for the estimator library
file(GLOB_RECURSE ESTIMATOR_SOURCES
    "src/include/state_estimator/*.hpp"
    "src/src/state_estimator.cpp"
)

# Collect LCM message sources
file(GLOB_RECURSE LCM_MSG_SOURCES
    "src/msgs/state_estimator_msgs/*.hpp"
)

# Create the attitude estimator executable
add_executable(attitude_estimator_lcm 
    attitude_estimator_lcm.cpp
    ${ESTIMATOR_SOURCES}
)

# Create the leg odometry estimator executable
add_executable(leg_odometry_estimator_lcm 
    leg_odometry_estimator_lcm.cpp
)

# Link libraries for attitude estimator
target_link_libraries(attitude_estimator_lcm 
    ${LCM_LIBRARIES}
    Eigen3::Eigen
    m
)

# Link libraries for leg odometry estimator
target_link_libraries(leg_odometry_estimator_lcm 
    ${LCM_LIBRARIES}
    Eigen3::Eigen
    pinocchio::pinocchio
    m
)

# Add compile definitions for both executables
target_compile_definitions(attitude_estimator_lcm PRIVATE
    ${LCM_CFLAGS_OTHER}
)

target_compile_definitions(leg_odometry_estimator_lcm PRIVATE
    ${LCM_CFLAGS_OTHER}
)

# Install targets
install(TARGETS attitude_estimator_lcm leg_odometry_estimator_lcm
    DESTINATION bin
)

# Print configuration info
message(STATUS "LCM found: ${LCM_FOUND}")
message(STATUS "LCM include dirs: ${LCM_INCLUDE_DIRS}")
message(STATUS "LCM libraries: ${LCM_LIBRARIES}")
message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
