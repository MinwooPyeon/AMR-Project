#!/usr/bin/env bash

echo "trying to stop running amr server container"
docker-compose down amr-server

echo "building spring server"
./gradlew build -x test > /dev/null 2>&1
build_result=$?

if [ $build_result -ne 0 ]; then
  echo "failed to build spring server"
  exit 1
fi

echo "successfully built spring server"

echo "building amr server image"
docker build --tag amr-server . 2>&1
docker_build_result=$?

if [ $docker_build_result -ne 0 ]; then
  echo "failed to build amr server image"
  exit 1
fi

echo "successfully built docker images"

echo "trying to run containers"
docker-compose up -d 2>&1
docker_run_result=$?

if [ $docker_run_result -ne 0 ]; then
  echo "failed to run containers"
  exit 1
fi

echo "successfully running containers"