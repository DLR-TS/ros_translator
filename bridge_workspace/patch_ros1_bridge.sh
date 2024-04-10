#!/usr/bin/env bash

set -euo pipefail
#set -euxo pipefail #debug mode

echoerr (){ printf "%s" "$@" >&2;}
exiterr (){ printf "%s\n" "$@" >&2; exit 1;}

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

sed -i "/ros2_suffixes =/c\    ros2_suffixes = ['_msgs', '_interfaces', '_msg']" "${SCRIPT_DIRECTORY}/src/ros1_bridge/ros1_bridge/__init__.py"

