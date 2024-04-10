#!/usr/bin/env bash

set -euo pipefail
#set -euxo pipefail #debug mode

echoerr (){ printf "%s" "$@" >&2;}
exiterr (){ printf "%s\n" "$@" >&2; exit 1;}

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

bridge_packages_directory="${SCRIPT_DIRECTORY}/src/bridge_packages"
bridge_package=

for bridge_package_directory in "$bridge_packages_directory"/*/; do
    if [ -d "$bridge_package_directory" ]; then
        package="$(basename "${bridge_package_directory}")" 
        if [ "$package" == "template" ]; then
            continue;
        fi
        echo "Package: $package"
        if [ -z "$(ls -A "$bridge_package_directory/msg")" ]; then
            rm -rf "${bridge_package_directory}"

        fi
    fi
done
