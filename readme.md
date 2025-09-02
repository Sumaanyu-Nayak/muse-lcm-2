# Attitude Estimator - LCM Implementation

This repository contains simplified attitude estimation implementations converted from the original ROS2 plugin-based system to standalone LCM-based programs.

## Overview

The original code used a complex plugin architecture with ROS2. This has been simplified into standalone programs that use LCM (Lightweight Communications and Marshalling) for message passing.

## Available Implementations

### 1. Simple Attitude Estimator (`simple_attitude_estimator.cpp`)
- **Dependencies**: None (except standard C++ libraries)
- **Algorithm**: Basic complementary filter
- **Features**: 
  - No external dependencies
  - Simulates IMU data
  - Basic quaternion-based attitude estimation
  - Educational/demonstration purposes

### 2. Simple LCM Attitude Estimator (`simple_attitude_estimator_lcm.cpp`)
- **Dependencies**: LCM library
- **Algorithm**: Basic complementary filter
- **Features**:
  - Can be compiled with or without LCM support
  - Uses LCM for communication when available
  - Falls back to simulation mode without LCM

### 3. Full Attitude Estimator LCM (`attitude_estimator_lcm.cpp`)
- **Dependencies**: LCM, Eigen3, Original estimator library
- **Algorithm**: AttitudeBiasXKF (Extended Kalman Filter with bias estimation)
- **Features**:
  - Full implementation from original plugin
  - Uses the original AttitudeBiasXKF filter
  - LCM communication
  - Production-ready

### 4. Simple Leg Odometry Estimator (`simple_leg_odometry_estimator.cpp`)
- **Dependencies**: None (except standard C++ libraries)
- **Algorithm**: Basic kinematic calculations
- **Features**:
  - No external dependencies
  - Simulates joint states and contact detection
  - Basic 2-DOF leg kinematics
  - Educational/demonstration purposes

### 5. Full Leg Odometry Estimator LCM (`leg_odometry_estimator_lcm.cpp`)
- **Dependencies**: LCM, Eigen3, Pinocchio
- **Algorithm**: Full kinematic computation with Pinocchio
- **Features**:
  - Full implementation from original plugin
  - Uses Pinocchio for accurate kinematics
  - LCM communication
  - Production-ready

## Building

### Method 1: Using Make (Simple)
```bash
# Build simple version (no dependencies)
make

# Build LCM version (requires LCM)
make lcm

# Build full attitude estimator (requires LCM + Eigen + estimator library)
make attitude-lcm

# Build simple leg odometry estimator (no dependencies)
make simple-leg-odom

# Build full leg odometry estimator (requires LCM + Eigen + Pinocchio)
make leg-odom-lcm
```

### Method 2: Using CMake (Recommended for full version)
```bash
make cmake-build
```

### Manual CMake build
```bash
mkdir build
cd build
cmake ..
make
```

## Running

### Simple Version
```bash
make run
# or
./simple_attitude_estimator
```

### LCM Version  
```bash
make run-lcm
# or
./simple_attitude_estimator_lcm
```

### Full Leg Odometry Estimator
```bash
make run-leg-odom-lcm
# or
./leg_odometry_estimator_lcm
```

### Simple Leg Odometry Estimator
```bash
make run-simple-leg-odom
# or
./simple_leg_odometry_estimator
```

## LCM Messages

The estimators use these LCM message types:

### Input: `imu_t`
- Angular velocity (rad/s)
- Linear acceleration (m/s²)
- Header with timestamp

### Input: `joint_state_with_acceleration_t`
- Joint positions, velocities, accelerations
- Joint names and efforts
- Number of joints

### Input: `contact_detection_t`
- Contact states for each foot (LF, RF, LH, RH)
- Boolean flags for stance phases

### Output: `attitude_t`  
- Quaternion (w, x, y, z)
- Euler angles in degrees (roll, pitch, yaw)
- Angular velocity (rad/s)
- Header with timestamp

### Output: `leg_odometry_t`
- Linear velocities for each foot (LF, RF, LH, RH)
- Estimated base velocity
- Header with timestamp

## Key Changes from Original

1. **Removed Plugin Architecture**: No more plugin base classes or dynamic loading
2. **Replaced ROS2 with LCM**: 
   - `rclcpp` → `lcm`
   - ROS2 messages → LCM messages
   - ROS2 subscribers/publishers → LCM subscribe/publish
3. **Simplified Configuration**: Hardcoded parameters instead of ROS2 parameter server
4. **Standalone Executables**: Self-contained programs instead of plugin libraries

## Algorithm Details

### Simple Complementary Filter
- Integrates gyroscope for short-term accuracy
- Uses accelerometer for long-term stability  
- Simple SLERP (spherical linear interpolation) fusion

### AttitudeBiasXKF Filter
- Extended Kalman Filter implementation
- Estimates attitude quaternion and gyroscope bias
- Uses accelerometer and magnetometer measurements
- Handles coordinate frame transformations

### Leg Odometry with Pinocchio
- Uses Pinocchio library for accurate forward kinematics
- Computes foot velocities from joint states
- Estimates base velocity from stance leg information
- Supports arbitrary robot models via URDF

### Simple Leg Odometry
- Basic 2-DOF kinematic calculations per leg
- Simplified hip-knee model
- Educational demonstration of concepts
- No external library dependencies

## Dependencies Installation

### Ubuntu/Debian
```bash
# LCM
sudo apt-get install liblcm-dev

# Eigen3
sudo apt-get install libeigen3-dev

# Pinocchio (for leg odometry)
sudo apt-get install libpinocchio-dev
```

### macOS (using Homebrew)
```bash
# LCM  
brew install lcm

# Eigen
brew install eigen

# Pinocchio (for leg odometry)
brew install pinocchio

# Build tools
brew install cmake
```

## Usage Notes

1. The simple version is good for learning and testing basic concepts
2. The LCM version demonstrates how to integrate with LCM communication
3. The full version provides production-ready attitude estimation
4. All versions can run independently without the original plugin framework
5. Default LCM URL is `udp://239.255.76.67:7667?ttl=1` (can be modified in code)

## Example LCM Usage

To test with LCM, you can use the `lcm-spy` tool to monitor messages:

```bash
# Terminal 1: Run the estimator
./attitude_estimator_lcm

# Terminal 2: Monitor LCM traffic
lcm-spy

# Terminal 3: Send test IMU data (if you have a publisher)
# Or use lcm-gen to create test message publishers
```

---

# Original MUSE Documentation

<h1 align="center"> MUSE: A Real-Time Multi-Sensor State Estimator for Quadruped Robots </h1>
<h3 align="center">Ylenia Nisticò, João Carlos Virgolino Soares, Lorenzo Amatucci, Geoff Fink and Claudio Semini</h3>	

<h4 align="center">This paper has been accepted to IEEE Robotics and Automation Letters, and it is available at https://arxiv.org/abs/2503.12101 </h4>

<h3 align="center"> 
    
![muse_cropped](https://github.com/user-attachments/assets/b212edff-44a4-4e46-acb9-c48e160ae8cd)
    

# :computer: Code

The `muse` package provides a ROS node and utilities for estimating the state of a quadruped robot using sensor data. It includes algorithms for state estimation, sensor fusion, and filtering.

This first version of the code provides a proprioceptive state estimator for quadruped robots. The necessary inputs are 
- **imu measurements**
- **joint states**
- **force exerted on the feet**

    
Additional code to fuse exteroceptive measurements will be available soon!
TODO list at the end of the page
</h2>



## :t-rex: Prerequisites
* [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) (converted from ROS Noetic)
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
* [Pinocchio](https://github.com/stack-of-tasks/pinocchio/tree/master)

⚠️ Don't worry! In this repo, we provide **Dockerization** to avoid dealing with the dependencies!

## :hammer_and_wrench: Building and Running

To install the `muse` package, follow these steps:

1. Clone this repository and build the Docker image:
    ```sh
    git clone https://github.com/iit-DLSLab/muse.git
    cd muse
    docker build -t muse-docker .
    ```

2. Enter the docker and build using `colcon build`:
    ```sh
    cd muse_ros2/muse_ros2_ws
    xhost +local:docker
    docker run -it --rm --name muse -v "$(pwd)":/root/muse_ros2_ws -w  /root/muse_ros2_ws muse-docker
    colcon build
    source install/setup.bash  
    ```
3. To launch the state estimator node:
   ```sh
   ros2 launch state_estimator state_estimator.launch.py
   ```
If you need to read the data from a rosbag, you need to mount the folder where you store your rosbags (`your_path_to_rosbags`), to make it visible inside the image, and then, you can attach a docker image in another terminal, for example:
```sh
docker run -it --rm --name muse \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v your_path_to_rosbags:/root/rosbags \
  -v "$(pwd)":/root/muse_ros2_ws \
  -w /root/muse_ros2_ws \
  muse-docker (terminal 1)
docker exec -it muse bash (terminal 2)
source install/setup.bash
cd ~/rosbags (terminal 2)
ros2 bag play your_rosbag.db3 (terminal 2)
```
To change the name of the topics, check the [config foder](https://github.com/iit-DLSLab/muse/tree/main/muse_ws/src/state_estimator/config).

To visualize your data, you can use [PlotJuggler](https://github.com/facontidavide/PlotJuggler?tab=readme-ov-file) which is already installed in the docker image:
```sh
ros2 run plotjuggler plotjuggler
```

:warning: In this repo we provide an example with the ANYmal B300 robot. If you want to test MUSE with another one, you only need to add the URDF of your robot in [this folder](https://github.com/iit-DLSLab/muse/tree/main/muse_ws/src/state_estimator/urdfs), and change the name of the legs in the [leg odometry plugin, line 249](https://github.com/iit-DLSLab/muse/blob/main/muse_ws/src/state_estimator/src/plugins/leg_odometry_plugin.cpp#L249):

``` sh
std::vector<std::string> feet_frame_names = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};   // Update with your actual link names
```
For real-world experiments, we recommend using this very nice [MPC](https://github.com/iit-DLSLab/Quadruped-PyMPC) to control your robot!
## :scroll: TODO list
- [ ] Extend the code to include exteroception
- [x] Dockerization
- [x] Support for ROS2

## :hugs: Contributing

Contributions to this repository are welcome.

## Citing the paper

If you like this work and would like to cite it (thanks):
```
@ARTICLE{10933515,
  author={Nisticò, Ylenia and Soares, João Carlos Virgolino and Amatucci, Lorenzo and Fink, Geoff and Semini, Claudio},
  journal={IEEE Robotics and Automation Letters}, 
  title={MUSE: A Real-Time Multi-Sensor State Estimator for Quadruped Robots}, 
  year={2025},
  volume={10},
  number={5},
  pages={4620-4627},
  keywords={Robots;Sensors;Robot sensing systems;Legged locomotion;Odometry;Cameras;Laser radar;Robot vision systems;Robot kinematics;Quadrupedal robots;State estimation;localization;sensor fusion;quadruped robots},
  doi={10.1109/LRA.2025.3553047}}
```
This repo is maintained by [Ylenia Nisticò](https://github.com/ylenianistico)