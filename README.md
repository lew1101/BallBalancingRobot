# BallBalancingRobot

**Author:** [Kenneth Lew](https://github.com/lew1101)

Software for a Raspberry Pi–based 3D-printed ball-balancing robot built by Kenneth Lew and Lek Man.

## Features

- Real-time ball tracking using OpenCV
- PID control of the platform
- Orientation correction using MPU6050 with Madgwick filter

## Installation

**1. Clone the repository.**

```bash
git clone https://github.com/yourusername/BallBalancingRobot.git
cd BallBalancingRobot
```

**2. Install system depedencies on Pi.**

```bash
sudo apt update
sudo apt install -y pigpio python3-picamera2 libopencv-dev 
```

This will install the `pipgpio` daemon as well as `picamera2` and `OpenCV` as system dependencies. We install them seperately because they are not available on PyPI for the Raspberry Pi architecture.

**3. Install the project as an editable module.**

Make sure the Python version on the Pi is >=3.9. Then run:

```bash
pip install -e .
```

This will install the project into the environment as well as install the rest of the python dependencies.

## Running the Script

Start the `pigpio` daemon:

```bash
sudo pigpiod
```

Run script:

```bash
bbrobot 
```
