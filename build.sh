#!/bin/bash

# Build the package
colcon build --packages-select sea_battle_sim

# Source the setup script
. install/setup.bash
