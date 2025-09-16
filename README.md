# Cognionics-Quick-20-ROS2-Bridge



## Installiation

Run setup.sh using `sh setup.sh`.
If you're using windows, please run `./setup.sh --os=windows`.

Disclaimer, you may recieve `setup.sh: 114: source: not found`; ignore it, it is meant to be there.

## Usage

ros2 run ros2neuro_acquisition acquisition --ros-args -p plugin:=ros2neuro::EGDDevice -p devarg:=q20 -p samplerate:=500 -p framerate:=500.0
