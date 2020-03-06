#!/bin/bash

if [ -d ${1} ] && [ "${1}"!="" ]
then
	echo "Loading model at ${1}"
	ros2 topic pub /model std_msgs/msg/String "data: ${1}" -1
else
	echo "Path ${1} does not exist"
fi
