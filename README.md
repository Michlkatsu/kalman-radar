# Kalman Radar

## Overview

Kalman Radar is a Raspberry Pi–based, fixed-direction distance measurement system using a TF-Nova LiDAR sensor and a one-dimensional Kalman filter.

The program reads distance measurements from the sensor, processes them through a Kalman filter, and displays raw measurements alongside filtered estimates in a live graphical interface.

## Features

- Real-time distance measurement using TF-Nova LiDAR.
- One-dimensional Kalman filtering to estimate distance from successive measurements.
- Live graphical range scope covering distances up to 7 meters.
- Magnified close-range view covering 0–100 cm.
- Rolling history graph comparing raw and filtered measurements.
- Sensor diagnostics, including confidence, signal peak, valid frames, checksum errors, and Kalman gain.

## Hardware

- Raspberry Pi
- TF-Nova LiDAR sensor
- Serial connection at `/dev/ttyAMA0`

## Software

- Python 3
- pyserial
- tkinter
- Python standard libraries for threading, timing, and data handling

## How It Works

The program reads serial data frames from the LiDAR sensor and validates each frame using its checksum.

Valid frames provide the measured distance, which is passed to a one-dimensional Kalman filter. The filter combines the previous estimate with each new measurement to produce a filtered distance estimate.

The interface displays both measurements, allowing their differences and changes over time to be observed.

## Running the Program

Install the required dependencies# kalman-radar
