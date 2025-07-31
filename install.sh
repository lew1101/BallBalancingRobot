#!/bin/bash

set -e

echo "1. Install System Dependencies"

sudo apt update
sudo apt install -y \
    libcamera-dev \
    python3-picamera2 \
    python3-opencv \
    pigpio \
    libatlas-base-dev

echo "2. Install Project"

pip3 install -e .

echo "Setup Complete"
