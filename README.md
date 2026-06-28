# CommandBrick

## Oliver Case – Capacitive Bluetooth Keyboard and Mouse

This repository contains the project files for the **CommandBrick**, an assistive input device based on an **ESP32-S3**.  
The device combines a capacitive touch keyboard, a touchpad for mouse movement, Bluetooth HID communication, and vibration feedback.

The goal of the project is to create an accessible input device that can be used with very little physical force.

## Main Functions

- Bluetooth keyboard and mouse control
- Capacitive touch buttons using the MPR121 sensor
- Mouse movement using a Cirque Pinnacle touchpad
- Vibration feedback when a button is pressed
- Compact custom hardware setup based on ESP32-S3

## Hardware Overview

The device uses the following main components:

- ESP32-S3 microcontroller
- MPR121 capacitive touch controller
- Cirque Pinnacle touchpad
- Vibration motor
- Li-Ion battery and charging/protection module
- Custom 3D printed case

## Repository Structure

```text
Oliver-Case/
│
├── code/
│   └── Source code for the ESP32-S3 device
│
├── programming/
│   └── Description of how the ESP32-S3 was programmed and uploaded
│
├── pin-documentation/
│   └── Pin connections, wiring tables, and hardware connection notes
│
├── photos/
│   └── Photos of the device, electronic parts, sensors, chips, and assembly
│
├── documentation/
│   └── General project documentation, explanations, and technical notes
│
├── presentation/
│   └── Presentation files and project slides
│
└── README.md
