#!/bin/bash

# Build and run the muse-lcm Docker container

echo "=== Building muse-lcm Docker image ==="
docker build -t muse-lcm:latest .

if [ $? -eq 0 ]; then
    echo "=== Docker build successful ==="
    echo "=== Running container with bash ==="
    
    # Allow X11 forwarding (for GUI applications if needed)
    xhost +local:docker
    
    # Run the container with the current directory mounted
    docker run -it --rm \
        --name muse-lcm-container \
        -v "$(pwd)":/root/muse_lcm_ws \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -e DISPLAY=$DISPLAY \
        -e LCM_DEFAULT_URL=udp://239.255.76.67:7667?ttl=1 \
        -p 7667:7667/udp \
        --network host \
        muse-lcm:latest /bin/bash
else
    echo "=== Docker build failed ==="
    exit 1
fi
