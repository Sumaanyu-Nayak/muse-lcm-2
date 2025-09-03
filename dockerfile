# Use Ubuntu 22.04 as base image
FROM ros:humble

# Set environment variables to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Set the working directory
WORKDIR /root/muse_lcm_ws

# Update package lists and install basic tools
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    git \
    build-essential \
    cmake \
    pkg-config \
    python3 \
    python3-pip \
    python3-dev \
    libeigen3-dev \
    libboost-all-dev \
    openjdk-11-jdk \
    libglib2.0-dev \
    # Pinocchio dependencies \
    liburdfdom-dev \
    liburdfdom-headers-dev \
    libconsole-bridge-dev \
    libassimp-dev \
    libtinyxml2-dev \
    libxml2-dev \
    libxslt1-dev \
    vim \
    nano \
    net-tools \
    liblcm-dev \
    liblcm-java \
    ros-humble-pinocchio \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Create symlink for python
RUN ln -sf /usr/bin/python3 /usr/bin/python



# Build and install Pinocchio from source
# RUN git clone --recursive https://github.com/stack-of-tasks/pinocchio.git /tmp/pinocchio && \
#     cd /tmp/pinocchio && \
#     mkdir build && cd build && \
#     cmake .. \
        # -DCMAKE_BUILD_TYPE=Release \
        # -DCMAKE_INSTALL_PREFIX=/usr/local \
        # -DBUILD_PYTHON_INTERFACE=ON \
        # -DBUILD_WITH_COLLISION_SUPPORT=ON \
        # -DBUILD_TESTING=OFF \
        # -DBUILD_BENCHMARK=OFF \
#         && \
#     make -j$(nproc) && \
#     make install && \
#     ldconfig && \
#     rm -rf /tmp/pinocchio

# Install minimal Python packages
RUN pip3 install numpy scipy lcm

# Set environment variables
ENV LCM_DEFAULT_URL=udp://239.255.76.67:7667?ttl=1
ENV PYTHONPATH=/usr/local/lib/python3.10/site-packages:$PYTHONPATH

# Create test script
# RUN echo '#!/bin/bash\n\
# echo "=== Testing installations ==="\n\
# lcm-spy --help > /dev/null 2>&1 && echo "LCM: OK" || echo "LCM: FAILED"\n\
# python3 -c "import numpy; print(f\"NumPy: {numpy.__version__}\")" || echo "NumPy: FAILED"\n\
# python3 -c "import pinocchio; print(f\"Pinocchio: {pinocchio.__version__}\")" || echo "Pinocchio Python: FAILED"\n\
# pkg-config --exists eigen3 && echo "Eigen3: OK" || echo "Eigen3: FAILED"\n\
# pkg-config --exists pinocchio && echo "Pinocchio C++: OK" || echo "Pinocchio C++: FAILED"\n\
# echo "=== Setup Complete ==="\n\
# ' > /root/test_setup.sh && chmod +x /root/test_setup.sh

# Create build script for simple examples
# RUN echo '#!/bin/bash\n\
# cd /root/muse_lcm_ws\n\
# echo "Building simple examples..."\n\
# g++ -std=c++17 -O2 simple_attitude_estimator.cpp -o simple_attitude_estimator -I/usr/include/eigen3 $(pkg-config --cflags --libs lcm) || echo "simple_attitude_estimator build failed"\n\
# g++ -std=c++17 -O2 simple_leg_odometry_estimator.cpp -o simple_leg_odometry_estimator -I/usr/include/eigen3 $(pkg-config --cflags --libs lcm) || echo "simple_leg_odometry_estimator build failed"\n\
# echo "Building Pinocchio examples..."\n\
# g++ -std=c++17 -O2 leg_odometry_estimator_lcm.cpp -o leg_odometry_estimator_lcm -I/usr/include/eigen3 $(pkg-config --cflags --libs lcm pinocchio) || echo "leg_odometry_estimator_lcm build failed"\n\
# echo "Build complete. Available executables:"\n\
# ls -la simple_* leg_odometry_estimator_lcm\n\
# ' > /root/build_simple.sh && chmod +x /root/build_simple.sh

# Set the default command to bash
CMD ["/bin/bash"]

# Expose LCM default port
EXPOSE 7667/udp