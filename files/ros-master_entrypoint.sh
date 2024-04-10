#!/usr/bin/env bash

set -euo pipefail

echo "Staring rosmaster"
source /opt/ros/noetic/setup.bash
stdbuf -o L roscore


