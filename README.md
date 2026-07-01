# CommandBrick

## Oliver Case – Capacitive Bluetooth Keyboard and Mouse

This repository contains the project files for the **CommandBrick**, an assistive input device based on an **ESP32-S3**.  
The device combines a capacitive touch keyboard, a touchpad for mouse movement, Bluetooth HID communication, and vibration feedback.

The goal of the project is to create an accessible input device that can be used with very little physical force.

<img width="500" height="500" alt="photo1" src="https://github.com/user-attachments/assets/12480e32-8642-45a6-93e1-1fc9fdcc148f" />
<img width="500" height="500" alt="photo4" src="https://github.com/user-attachments/assets/29f5d4b2-1ade-4c45-b9f8-0d801bcf1e88" />


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
CapacitiveKeyboard/
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
