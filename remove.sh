#!/bin/bash

# Exit if any command fails
set -e

BASE_DIR="$( cd "$( dirname $0 )" && pwd )"
INSTALL_DIR="$BASE_DIR/install"
MMLIB_DIR="$BASE_DIR/mmlib"
XDFFILEIO_DIR="$BASE_DIR/xdffileio"
EEGDEV_DIR="$BASE_DIR/eegdev"
NEURO_DIR="$BASE_DIR/ros2neuro_quick20_ws"

mv "$INSTALL_DIR/etc" "$BASE_DIR/etc"
mv "$EEGDEV_DIR/src/core/opendev.c" "$BASE_DIR/opendev.c"
mv "$NEURO_DIR/src/ros2neuro_acquisition_eegdev/CMakeLists.txt" "$BASE_DIR/CMakeLists.txt"


rm -r -f install/ xdffileio/ mmlib/ eegdev/
rm -r -f ros2neuro_quick20_ws/install ros2neuro_quick20_ws/build ros2neuro_quick20_ws/log
rm -r -f ros2neuro_quick20_ws/src/ros2neuro_acquisition_eegdev ros2neuro_quick20_ws/src/ros2neuro_acquisition ros2neuro_quick20_ws/src/ros2neuro_data ros2neuro_quick20_ws/src/ros2neuro_msgs

echo "Remove complete"