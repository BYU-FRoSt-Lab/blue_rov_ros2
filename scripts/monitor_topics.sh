#!/bin/bash

# TODO make it dynamic which container to exec into

docker exec -it bluerov_ros2 bash -c \
  "source ~/ros2_ws/install/setup.bash && \
   ros2 run topic_monitor topic_monitor_node \
   --ros-args --params-file ~/config/time_sync_utils.yaml"