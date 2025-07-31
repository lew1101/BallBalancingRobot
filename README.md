# BallBalancingRobot

**Author:** [Kenneth Lew](https://github.com/lew1101)

Software for a Raspberry Pi–based 3D-printed ball-balancing robot built by [Kenneth Lew](https://github.com/lew1101) and [Lek Man](https://github.com/sophondroplet).

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

**2. Install Project**

```bash
sh install.sh
```

## Running the Script

Start the `pigpio` daemon:

```bash
sudo pigpiod
```

Run script:

```bash
bbrobot 
```

## Usage

```bash
bbrobot [-h] [--setpoint X Y] [--debug] [--no-imu]
```
