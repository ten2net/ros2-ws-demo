#!/bin/bash

# Build the package
colcon build --packages-select sensor_simulator

# Source the setup script
. install/setup.bash
