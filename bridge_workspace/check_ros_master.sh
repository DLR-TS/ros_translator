#!/usr/bin/env bash

set -euo pipefail

#!/bin/bash

# Get the host and port from the ROS_MASTER_URI environment variable
ROS_MASTER_URI="$ROS_MASTER_URI"
if [ -z "$ROS_MASTER_URI" ]; then
    echo "ROS_MASTER_URI environment variable is not set" >&2
    exit 1
fi

HOST_PORT="${ROS_MASTER_URI#*://}"
HOST="${HOST_PORT%:*}"
PORT="${HOST_PORT#*:}"

nc -zvw3 "$HOST" "$PORT" >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "Host $HOST is listening on port $PORT"
    exit 0
else
    echo "ERROR: No ROS master host found on: ${ROS_MASTER_URI}" >&2
    exit 1
fi

