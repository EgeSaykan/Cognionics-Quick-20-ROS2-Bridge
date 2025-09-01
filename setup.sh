#!/bin/bash

# Exit if any command fails
set -e


# Default os
OS="linux"


# Parse input argument --os=windows or --os=linux
for arg in "$@"; do
    case $arg in
        --os=windows)
            OS="windows"
            shift
            ;;
        --os=linux)
            OS="linux"
            shift
            ;;
    esac
done

if [ "$OS" != "windows" ] && [ "$OS" != "linux" ]; then
    echo "Error: Unsupported OS '$OS'. Only 'windows' and 'linux' are supported." >&2
    exit 1
fi

# Define base directory
BASE_DIR="$( cd "$( dirname $0 )" && pwd )"
INSTALL_DIR="$BASE_DIR/install"
MMLIB_DIR="$BASE_DIR/mmlib"
XDFFILEIO_DIR="$BASE_DIR/xdffileio"
EEGDEV_DIR="$BASE_DIR/eegdev"
NEURO_DIR="$BASE_DIR/ros2neuro_quick20_ws"

echo "[+] Cloning repositories..."
git clone https://github.com/mmlabs-mindmaze/mmlib.git "$MMLIB_DIR"
git clone https://github.com/mmlabs-mindmaze/xdffileio.git "$XDFFILEIO_DIR"
git clone https://github.com/neurorobotics-iaslab/eegdev.git "$EEGDEV_DIR"

echo "[+] Creating install directory..."
mkdir -p "$INSTALL_DIR"

mv "$BASE_DIR/etc" "$INSTALL_DIR"

echo "[+] Building mmlib..."
cd "$MMLIB_DIR"

mkdir -p build && cd build
../autogen.sh
../configure --prefix="$INSTALL_DIR"

echo "[+] Compiling mmlib..."
make
make install
echo "[+] mmlib installed."



echo "[+] Building xdffileio..."
cd "$XDFFILEIO_DIR"

mkdir -p build && cd build
../autogen.sh
../configure --prefix="$INSTALL_DIR" CPPFLAGS="-I$INSTALL_DIR/include" LDFLAGS="-L$INSTALL_DIR/lib -Wl,-rpath,$INSTALL_DIR/lib"

echo "[+] Patching config/config.h for Linux..."
CONFIG_FILE="$XDFFILEIO_DIR/build/config/config.h"

if [ -f "$CONFIG_FILE" ] && [ "$OS" = "linux" ]; then
    sed -i 's/^#define API_EXPORTED __declspec(dllexport)$/\/\/ #define API_EXPORTED __declspec(dllexport) \/\/ WINDOWS\n#define API_EXPORTED \/\/ LINUX/' "$CONFIG_FILE"
    echo "[+] Patched $CONFIG_FILE"
elif [ "$OS" != "windows" ]; then
    echo "[!] Could not find $CONFIG_FILE"
    exit 1
fi

echo "[+] Compiling xdffileio..."
make
make install
echo "[+] xdffileio installed."


echo "[+] Building eegdev..."

rm "$EEGDEV_DIR/src/core/opendev.c"
mv "$BASE_DIR/opendev.c" "$EEGDEV_DIR/src/core"
cd "$EEGDEV_DIR/src/plugins"
sed -i "23a#include <stdio.h>" fileout.c

cd "$EEGDEV_DIR"

mkdir -p build && cd build
../autogen.sh
../configure --prefix="$INSTALL_DIR" CPPFLAGS="-I$INSTALL_DIR/include -I/usr/local/lib" LDFLAGS="-L$INSTALL_DIR/lib -L/usr/local/lib -Wl,-rpath,$INSTALL_DIR/lib -Wl,-rpath,/usr/local/lib" --with-q20

echo "[+] Compiling eegdev..."
make
make install
echo "[+] eegdev installed."


cd "$BASE_DIR"
mkdir "$NEURO_DIR" && cd "$NEURO_DIR"

echo "[+] installing ros2neuro_acquisition_eegdev"
mkdir "$NEURO_DIR/src" && cd "$NEURO_DIR/src"
cd "$NEURO_DIR/src"
echo "[+] Cloning repo"
git clone https://github.com/neurorobotics-iaslab/ros2neuro_acquisition.git
git clone https://github.com/neurorobotics-iaslab/ros2neuro_acquisition_eegdev.git
git clone https://github.com/neurorobotics-iaslab/ros2neuro_msgs.git
git clone https://github.com/neurorobotics-iaslab/ros2neuro_data.git

rm "$NEURO_DIR/src/ros2neuro_acquisition_eegdev/CMakeLists.txt"
mv "$BASE_DIR/CMakeLists.txt" "$NEURO_DIR/src/ros2neuro_acquisition_eegdev/CMakeLists.txt"
cd "$NEURO_DIR"

# Adding library path
grep -qxF "export LD_LIBRARY_PATH=$BASE_DIR/install/lib:\$LD_LIBRARY_PATH" ~/.bashrc || echo "export LD_LIBRARY_PATH=$BASE_DIR/install/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
grep -qxF "export PKG_CONFIG_PATH=$BASE_DIR/install/lib/pkgconfig:\$PKG_CONFIG_PATH" ~/.bashrc || echo "export PKG_CONFIG_PATH=$BASE_DIR/install/lib/pkgconfig:\$PKG_CONFIG_PATH" >> ~/.bashrc
grep -qxF "export CMAKE_PREFIX_PATH=$BASE_DIR/install:\$CMAKE_PREFIX_PATH" ~/.bashrc || echo "export CMAKE_PREFIX_PATH=$BASE_DIR/install:\$CMAKE_PREFIX_PATH" >> ~/.bashrc
grep -qxF "export CMAKE_PREFIX_PATH=\$CMAKE_PREFIX_PATH:$BASE_DIR/ros2neuro_quick20_ws/install" ~/.bashrc || echo "export CMAKE_PREFIX_PATH=\$CMAKE_PREFIX_PATH:$BASE_DIR/ros2neuro_quick20_ws/install" >> ~/.bashrc
grep -qxF "export LD_LIBRARY_PATH=/usr/lib:\$LD_LIBRARY_PATH" ~/.bashrc || echo "export LD_LIBRARY_PATH=/usr/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
grep -qxF "export LD_LIBRARY_PATH=/lib:\$LD_LIBRARY_PATH" ~/.bashrc || echo "export LD_LIBRARY_PATH=/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
grep -qxF "export LD_LIBRARY_PATH=/lib64:\$LD_LIBRARY_PATH" ~/.bashrc || echo "export LD_LIBRARY_PATH=/lib64:\$LD_LIBRARY_PATH" >> ~/.bashrc
grep -qxF "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" ~/.bashrc || echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
grep -qxF "export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\$LD_LIBRARY_PATH" ~/.bashrc || echo "export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\$LD_LIBRARY_PATH" >> ~/.bashrc

source /opt/ros/humble/setup.bash   # Change to your ros distro
source ~/.bashrc

echo "[+] Building dependencies..."
colcon build --packages-select ros2neuro_msgs ros2neuro_data ros2neuro_acquisition
source "$NEURO_DIR/install/setup.bash"
echo "[+] Dependencies built."

echo "[+] Building ros2neuro_acquisition_eegdev..."
colcon build --packages-select ros2neuro_acquisition_eegdev
echo "[+] ros2neuro_acquisition_eegdev built."


echo "Setup completed successfully."
