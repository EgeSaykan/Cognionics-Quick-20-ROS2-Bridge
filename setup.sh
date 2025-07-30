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
BASE_DIR="$HOME/Local"
INSTALL_DIR="$BASE_DIR/install"
MMLIB_DIR="$BASE_DIR/mmlib"
XDFFILEIO_DIR="$BASE_DIR/xdffileio"
EEGDEV_DIR="$BASE_DIR/eegdev"

echo "[+] Cloning repositories..."
git clone https://github.com/mmlabs-mindmaze/mmlib.git "$MMLIB_DIR"
git clone https://github.com/mmlabs-mindmaze/xdffileio.git "$XDFFILEIO_DIR"
git clone https://github.com/neurorobotics-iaslab/eegdev.git "$EEGDEV_DIR"


echo "[+] Creating install directory..."
mkdir -p "$INSTALL_DIR"

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
cd "$EEGDEV_DIR"
mkdir -p build && cd build
../autogen.sh
../configure --prefix="$INSTALL_DIR" --with-q20

echo "[+] Compiling eegdev..."
make
make install
echo "[+] eegdev installed."

echo "Setup completed successfully."