#!/bin/bash

logfile="/root/ros2_ws/cpu_load.log"

ContainerName="test_task"
ContainerId=$(docker ps -aqf "name=$ContainerName")

if [ -z "$ContainerId" ]; then
  echo "ERROR: 404  $ContainerName Not Found"
  exit 404
fi

command="docker exec $ContainerId tail -f $logfile"
$command

