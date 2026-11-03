#!/bin/bash

SERIAL_PORT=/dev/ttyACM0

# build binary
echo "Building the binary ..."
echo
mkdir -p build/gps_compass
cp gps_compass.ino build/gps_compass/
arduino-cli compile --fqbn esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi --export-binaries build/gps_compass
cp build/gps_compass/build/esp32.esp32.esp32s3/gps_compass.ino.bin gps_compass.ino.esp32s3.bin

if [ -f gps_compass.ino.esp32s3.bin ]; then
    echo
    echo "The binary is in the current folder named: $(ls gps_compass.ino.esp32s3.bin)"
    rm -Rf build
else
    echo "Something went wrong. There's no binary here."
    exit 1
fi

echo
read -p "Do you want to install the binary to port $SERIAL_PORT? [Y/n] " resp
resp="${resp,,}"
resp="${resp:-y}"

if [ $resp == y ]; then
    esptool --chip esp32s3 --port /dev/ttyACM0 --baud 921600 write-flash 0x10000 gps_compass.ino.esp32s3.bin
else
    echo "Skipping install."
fi
