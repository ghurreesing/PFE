# Line-Following Robot with ROS and Mobile Control

## Project Overview

This project is a line-following robot built with the **SunFounder PiCar-S** kit and controlled by a **Raspberry Pi 4** running **ROS Noetic** on **Ubuntu 20.04**. The robot uses infrared (IR) sensors for line tracking, an ultrasonic sensor for obstacle avoidance, and a camera for QR code detection. A **Flutter** mobile app provides remote control, live video feed, functionality management.

## Installation

### 1. Prerequisites

- **Raspberry Pi 4** with **Ubuntu 20.04**.
- **ROS Noetic** installed.
- **Flutter SDK** for the mobile app.

### 2. Setting Up the Project

```bash
# Clone the repository and set up ROS workspace
git clone https://github.com/yourusername/line-following-robot.git
cd ~/catkin_ws/src
cp -r line-following-robot ~/catkin_ws/src/

# Build workspace and source
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. Install Dependencies

Install the required dependencies using the following commands:

```bash
sudo apt-get install ros-noetic-rosbridge-server python3-pip
pip3 install websocket-client 
```

### 4. Running the Project

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
roslaunch app_interface options.launch
```

## Mobile Application
The mobile app is built with Flutter and uses ROSBridge for real-time control and monitoring via WebSocket. To initialize the WebSocket in the code:

```bash
WebSocketChannel channel = WebSocketChannel.connect(Uri.parse('ws://<robot-ip-address>:9090'));
```
### Running the App
Clone the mobile app repository and run:
```bash
flutter run
```
## Features
#### Line Following: IR sensors for path detection.
#### Obstacle Avoidance: Ultrasonic sensor for safe navigation.
#### Mobile Control: Remote control via a Flutter app.
#### Live Video Feed: Real-time camera streaming.
#### QR Code Detection: For interaction with predefined stations.
