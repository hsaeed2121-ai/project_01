# project_01
For project 1 for EDES 301
Hamza's Robot Hand Desk Doohickey

This project controls an InMoov-based robotic hand using a PocketBeagle, an Adafruit PCA9685 Servo Driver, and a set of potentiometers. The system allows for real-time manual control of the robotic fingers, mapping analog inputs (potentiometers) to servo angles, and saves the hand's pose upon powering down.

Project Link: Hamza's Robot Hand Desk Doohickey - Hackster.io

Hardware Architecture

The system is built around the PocketBeagle, which reads analog voltage values from potentiometers and sends PWM signals via I2C to a servo driver.

Components

Controller: PocketBeagle (or BeagleBone Black)

Servo Driver: Adafruit PCA9685 16-Channel 12-bit PWM/Servo Driver

Actuators: 5x MG996R or similar hobby servos (InMoov Hand assembly)

Inputs: 5x 10kΩ Potentiometers

Power: 5V 4A External Power Supply (for servos)

Wiring Diagram Overview

I2C Connection: PocketBeagle I2C pins connected to PCA9685 (SCL/SDA).

Servos: Connected to channels 11-15 on the PCA9685.

Potentiometers: Connected to PocketBeagle Analog Input (AIN) pins.

Software Build Instructions

Prerequisites

PocketBeagle Setup: Ensure your PocketBeagle is running a standard Debian image and is connected to the internet 

Python 3: The software is written in Python 3.

Dependencies

Install the required Adafruit libraries for BeagleBone IO and the PCA9685 driver. Run the following commands on your PocketBeagle:

sudo apt-get update
sudo apt-get install python3-pip
sudo pip3 install Adafruit_BBIO
sudo pip3 install Adafruit_PCA9685


Installation

Clone this repository or copy the servoController.py file to your PocketBeagle.

git clone [https://github.com/your-username/your-repo-name.git]
cd your-repo-name
chmod +x servoController.py


🚀 Software Operation

Configuration

Before running, verify the pin configuration in the __main__ section of servoController.py. The default mapping is:

Finger

Potentiometer Pin (AIN)

Servo Channel (PCA9685)

Thumb

P1_19

11

Index

P1_21

12

Middle

P1_23

13

Ring

P1_25

14

Pinky

P1_27

15

You can also adjust the SERVO_CAL dictionary in the code to tune the minimum and maximum pulse widths for your specific motors.

Running the Controller

Execute the script 

sudo python3 servoController.py


Usage

Startup: The hand will automatically move to its last saved position (loaded from last_pose.json) or a neutral position if no save file exists.

Control: Rotate the potentiometers to move the corresponding fingers. The software has a tolerance filter to prevent jittering

Shutdown: Press Ctrl+C to exit the program. The current hand position will be saved to last_pose.json automatically, so that the hand doesn’t snap next time you boot it.

License and copyright information are included in the python script
