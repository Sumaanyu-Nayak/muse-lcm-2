cmake_minimum_required(VERSION 3.10)
project(attitude_node)

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
    ${EIGEN3_INCLUDE_DIR}
    ${LCM_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS}
)

# Add compiler flags - fix the LCM macro issue and reduce warnings
add_compile_options(-Wall -Wextra -O2 -fpermissive -Wno-unused-parameter -Wno-unused-variable -Wno-delete-non-virtual-dtor)

# Link directories
link_directories(${LCM_LIBRARY_DIRS})

# Get implementation files (non-main files) - exclude ALL main files explicitly
file(GLOB_RECURSE ALL_CPP_SOURCES "src/src/*.cpp")

# Remove ALL files with main() functions to avoid conflicts
list(REMOVE_ITEM ALL_CPP_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/src/state_estimator_node.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/src/plugin/attitude_node.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/src/plugin/leg_odometry_lcm.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/src/plugin/contact_detection.cpp"
)

# SHARED_SOURCES now contains only implementation files without main()
set(SHARED_SOURCES ${ALL_CPP_SOURCES})

# Create executables - each with only their own main file
add_executable(attitude_node 
    src/src/plugin/attitude_node.cpp
    ${SHARED_SOURCES}
)

add_executable(leg_odometry_node 
    src/src/plugin/leg_odometry_lcm.cpp
    ${SHARED_SOURCES}
)

add_executable(contact_detection_node 
    src/src/plugin/contact_detection.cpp
    ${SHARED_SOURCES}
)

add_executable(main_node 
    src/src/state_estimator_node.cpp
    ${SHARED_SOURCES}
)

# Link libraries
target_link_libraries(attitude_node 
    ${LCM_LIBRARIES}
    Eigen3::Eigen
)

target_link_libraries(leg_odometry_node 
    ${LCM_LIBRARIES}
    Eigen3::Eigen
    pinocchio::pinocchio # Added to fix URDF undefined reference
)

target_link_libraries(contact_detection_node 
    ${LCM_LIBRARIES}
    Eigen3::Eigen
)

target_link_libraries(main_node 
    ${LCM_LIBRARIES}
    Eigen3::Eigen
)

# Handle LCM compile definitions more carefully
if(LCM_CFLAGS_OTHER)
    # Convert the list to a string and process each flag
    separate_arguments(LCM_FLAGS_LIST UNIX_COMMAND "${LCM_CFLAGS_OTHER}")
    foreach(flag ${LCM_FLAGS_LIST})
        if(flag MATCHES "^-D(.+)")
            # Extract the definition part after -D
            string(REGEX REPLACE "^-D" "" definition "${flag}")
            # Only add if it's a valid identifier (contains letters, numbers, underscores)
            if(definition MATCHES "^[a-zA-Z_][a-zA-Z0-9_]*$" OR definition MATCHES "^[a-zA-Z_][a-zA-Z0-9_]*=.*$")
                target_compile_definitions(attitude_node PRIVATE ${definition})
                target_compile_definitions(leg_odometry_node PRIVATE ${definition})
                target_compile_definitions(contact_detection_node PRIVATE ${definition})
                target_compile_definitions(main_node PRIVATE ${definition})
            endif()
        endif()
    endforeach()
endif()

# Install targets
install(TARGETS attitude_node leg_odometry_node contact_detection_node main_node
    DESTINATION bin
)

# Print configuration info
message(STATUS "LCM found: ${LCM_FOUND}")
message(STATUS "LCM include dirs: ${LCM_INCLUDE_DIRS}")
message(STATUS "LCM libraries: ${LCM_LIBRARIES}")
message(STATUS "LCM CFLAGS: ${LCM_CFLAGS_OTHER}")
message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "Eigen3 version: ${Eigen3_VERSION}")
message(STATUS "Pinocchio found: ${pinocchio_FOUND}")
message(STATUS "Pinocchio include dirs: ${pinocchio_INCLUDE_DIRS}")