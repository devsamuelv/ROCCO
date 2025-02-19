#!/bin/bash
set -e

# Set the default build type
BUILD_TYPE=RelWithDebInfo
colcon build \
        --symlink-install \
        --event-handlers console_cohesion+ \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" -DCMAKE_BUILD_TYPE=Debug "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic
