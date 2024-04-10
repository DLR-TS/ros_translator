#!/usr/bin/env bash

set -euo pipefail

echoerr (){ printf "%s" "$@" >&2;}
exiterr (){ printf "%s\n" "$@" >&2; exit 1;}

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

bridge_packages_dir="${SCRIPT_DIRECTORY}/src/bridge_packages"
bridged_msgs_file="${SCRIPT_DIRECTORY}/bridged_msgs.txt"

delete_files_not_in_list() {
    local directory="$1"
    local file_list="$2"

    find "$directory" -type f | while read -r file; do
        if [[ "${file##*.}" == "msg" ]]; then
            if ! grep -q "$(basename "$file")" <<< "$file_list"; then
                rm "$file"
            fi
        fi
    done
}

file_list=$(grep -v '^\s*#' "$bridged_msgs_file")

delete_files_not_in_list "$bridge_packages_dir" "$file_list"


    
