#!/bin/bash

# Build the package
# colcon build --packages-select sensor_simulator
colcon build
. install/setup.bash
