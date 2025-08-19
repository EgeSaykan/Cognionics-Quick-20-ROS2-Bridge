#!/bin/bash

# Exit if any command fails
set -e

rm -r -f install/ xdffileio/ mmlib/ eegdev/
rm -r -f ros2neuro_quick20_ws/

echo "Remove complete"