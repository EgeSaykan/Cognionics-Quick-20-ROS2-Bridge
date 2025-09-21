# Cognionics-Quick-20-ROS2-Bridge

**This is an updated setup file for Cognionics Quick20 2017 edition.**


## Installiation

Firstly, ensure you have [libftd2xx](https://ftdichip.com/drivers/d2xx-drivers/).
This driver lets the usb port for connecting to the headset. To my understanding, windows machines already have this and might not need installing, but linux machines will do so.
You might have to run the following commands to remove unwanted usb drivers, it didn't help me but it is suggested:
```bash
sudo rmmod ftdi_sio
sudo rmmod usbserial
```


Once you have the libftd2xx.so file;
Run setup.sh using `sh setup.sh`.
If you're using windows, please run `./setup.sh --os=windows`.

## Usage

### Setup

There must be a Bluetooth dongle and a driver usb; the Bluetooth dongle is already registered with the headset's Bluetooth so it makes connecting easier, the usb adapter helps with the identification of the device and ports.
Connect both of the devices, make sure the headset is turned on and charged.

### Talker
source the ros2 packages:
```bash
cd ros2neuro_quick20_ws
source install/setup.bash
```

Start neurodata topic by running:
```bash
ros2 run ros2neuro_acquisition acquisition --ros-args -p plugin:=ros2neuro::EGDDevice -p devarg:=q20 -p samplerate:=500 -p framerate:=500.0
```

the acquisition node will start reading transmission of the headset and publishing to neurodata.

You can see confirm by watching the light on the Bluetooth dongle turn into flickering purple, or run 
```bash
ros2 topic list
ros2 topic echo \neurodata
```

## Data

I have some data recorded into ros2neuro_quick20_ws/data.
The data was recorded using a small robot arm game made on unity:
https://github.com/EgeSaykan/Unity-Ros2-RobotArmSimulation
