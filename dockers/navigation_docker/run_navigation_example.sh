#!/usr/bin/env bash

declare SCRIPT_NAME=$(readlink -f ${BASH_SOURCE[0]})
cd $(dirname $SCRIPT_NAME)

BUILD_ARGS=""
NO_CACHE=""

while (( "$#" )); do
  case "$1" in
    -b|--build)
      BUILD_ARGS="--build"
      shift
      ;;
    -n|--no-cache)
      BUILD_ARGS="--build"
      NO_CACHE="--no-cache"
      shift
      ;;
    *)
      echo "Unknown argument $1"
      echo "Usage: $0 [--build|-b] [--no-cache|-n]"
      exit 1
      ;;
  esac
done

# Combine flags
BUILD="$BUILD_ARGS $NO_CACHE"

# Allow X11 forwarding for GUI applications
xhost +local:docker

docker compose -f docker-compose-nav2.yml run ${BUILD} --rm nav2_scenario
