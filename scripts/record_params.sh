#!/bin/bash
# --------------------------------------------------
# Rosbag recording parameters
# This file is sourced by record_bag.sh
# --------------------------------------------------

# -----------------------------
# Container / paths
# -----------------------------
CONTAINER="ouster_lidar_sbg"
BAGS_PATH="/root/bags/ssd"

# -----------------------------
# Rosbag storage settings
# -----------------------------
STORAGE="mcap"
PRESET="fastwrite"

# 0 means “no duration limit” unless overridden
DURATION=120

# Bytes (example: 200 MB)
MAX_CACHE_SIZE=200000000

# -----------------------------
# Topic presets
# -----------------------------
TOPICS_LIO="/imu/data /imu/nav_sat_fix /points /tf /tf_static"

TOPICS_SBG="/imu \
/sbg/gps_pos \
/sbg/ekf_quat \
/sbg/ekf_nav \
/sbg/imu_data \
/sbg/imu_short \
/sbg/utc_time \
/imu/utc_ref \
/imu/mag \
/sbg/mag \
/tf_static"

# -----------------------------
# Default topics (used if no -p or -t)
# -----------------------------
TOPICS="${TOPICS_LIO}"
