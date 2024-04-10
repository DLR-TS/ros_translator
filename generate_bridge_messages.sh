#!/usr/bin/env bash

set -euo pipefail
#set -euxo pipefail #debug mode

echoerr (){ printf "%s" "$@" >&2;}
exiterr (){ printf "%s\n" "$@" >&2; exit 1;}

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

ROS1_PACKAGE_DIRECTORY="${SCRIPT_DIRECTORY}/ros1_packages"
ROS2_BRIDGE_PACKAGE_DIRECTORY="${SCRIPT_DIRECTORY}/bridge_workspace/src/bridge_packages"
TEMPLATE_PACKAGE_DIRECTORY="${SCRIPT_DIRECTORY}/bridge_workspace/src/template"
BIRDGED_MSGS_TXT_FILE="${SCRIPT_DIRECTORY}/bridge_workspace/bridged_msgs.txt"


get_relative_path() {
    local path="$1"
    local relative_to="$2"

    path=$(realpath "$path")
    relative_to=$(realpath "$relative_to")

    if [[ ! -d "$relative_to" ]]; then
        relative_to=$(dirname "$relative_to")
    fi

    local relative_path
    relative_path=$(realpath --relative-to="$relative_to" "$path")
    echo "$relative_path"
}


copy_msgs() {
    local relative_path="$1"
    local destination_directory="$2"
    bridged_msgs=$(grep "$relative_path" "$BIRDGED_MSGS_TXT_FILE" | grep -v "#" || true)
    if [ -z "$bridged_msgs" ]; then
        echo "  No bridged messages defined in bridged_msgs.txt, skipping."
        return
    fi
    while IFS= read -r path; do
        echo "    Copying message: ${path} to $(get_relative_path "${destination_directory}" "${SCRIPT_DIRECTORY}")"
        mkdir -p "${destination_directory}"
        cp "${ROS1_PACKAGE_DIRECTORY}/${path}" "${destination_directory}"
    done <<< "$bridged_msgs"
}















rm -rf "${ROS2_BRIDGE_PACKAGE_DIRECTORY}"/*

if [ -d "$ROS1_PACKAGE_DIRECTORY" ]; then
     for ros1_package_msg_directory in $(find "$ROS1_PACKAGE_DIRECTORY" -type d -name "msg"); do
        if [ -d "$ros1_package_msg_directory" ]; then
            ros1_package_directory="$(dirname "${ros1_package_msg_directory}")"
            ros1_package=$(basename "$(dirname "$ros1_package_msg_directory")")
            ros1_package_relative_msg_directory="$(get_relative_path "${ros1_package_msg_directory}" "${ROS1_PACKAGE_DIRECTORY}")"
            echo "Generating ROS2 Bridge package for: $ros1_package"
            echo "  ROS1 Package Directory: $ros1_package_directory"
            echo "  ROS1 Package msg Directory: $ros1_package_msg_directory"
            echo "  ROS1 Package relative msg Directory: $ros1_package_relative_msg_directory"
            echo "  ROS2 Package Directory: $ROS2_BRIDGE_PACKAGE_DIRECTORY/$ros1_package"
            echo "  ROS2 Package msg Directory: $ROS2_BRIDGE_PACKAGE_DIRECTORY/$ros1_package/msg"
            rm -rf "${ROS2_BRIDGE_PACKAGE_DIRECTORY}/$ros1_package"
            cp -r "$TEMPLATE_PACKAGE_DIRECTORY" "$ROS2_BRIDGE_PACKAGE_DIRECTORY/$ros1_package" || true
            sed -i "s|PACKAGE|${ros1_package}|g" "$ROS2_BRIDGE_PACKAGE_DIRECTORY/$ros1_package/package.xml"
            sed -i "/depend>${ros1_package}</d" "$ROS2_BRIDGE_PACKAGE_DIRECTORY/$ros1_package/package.xml"
            sed -i "/${ros1_package}/d" "$ROS2_BRIDGE_PACKAGE_DIRECTORY/$ros1_package/requirements.cmake"
            sed -i "/  ${ros1_package}/d" "$ROS2_BRIDGE_PACKAGE_DIRECTORY/$ros1_package/CMakeLists.txt"
            copy_msgs "${ros1_package_relative_msg_directory}" "$ROS2_BRIDGE_PACKAGE_DIRECTORY/$ros1_package/msg/"


            #cp $ros1_package_msg_directory/*.msg "$ROS2_BRIDGE_PACKAGE_DIRECTORY/$ros1_package/msg/"
            rm -f "$ROS2_BRIDGE_PACKAGE_DIRECTORY/$ros1_package/COLCON_IGNORE"
            rm -f "$ROS2_BRIDGE_PACKAGE_DIRECTORY/$ros1_package/CATKIN_IGNORE"
            rm -rf "$ROS2_BRIDGE_PACKAGE_DIRECTORY/$ros1_package/template"
        fi
    done
else
    echo "Error: Directory '$parent_dir' does not exist."
fi

cd bridge_workspace
#bash prune_msgs.sh
bash prune_empty_packages.sh
