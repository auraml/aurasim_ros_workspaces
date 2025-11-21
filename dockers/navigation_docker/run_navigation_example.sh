#!/usr/bin/env bash

declare SCRIPT_NAME=$(readlink -f ${BASH_SOURCE[0]})
cd $(dirname $SCRIPT_NAME)

BUILD=""

if [[ ! -z "$1" ]]; then
    if [[ "$1" == "--build" || "$1" == "-b" ]]; then
        BUILD="--build"
    elif [[ "$1" == "--no-cache" || "$1" == "-n" ]]; then
        BUILD="--build --no-cache"
    else
        echo "Unknown argument ${1}"
        echo "Usage: $0 [--build|-b] [--no-cache|-n]"
        exit 1
    fi
fi

# Allow X11 forwarding for GUI applications
xhost +local:docker

docker compose -f docker-compose-nav2.yml run ${BUILD} --rm nav2_scenario
