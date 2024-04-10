#!/usr/bin/env bash

set -e


sleep 2s

cd ${BRIDGE_WORKSPACE}
bash run_ros1_bridge.sh

sleep infinity
