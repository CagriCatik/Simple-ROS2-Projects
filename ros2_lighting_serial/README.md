# ROS 2 Arduino LED Control

This project allows a ROS 2 node to communicate with an Arduino to control an RGB LED strip (WS2812) using different lighting modes. The communication between ROS 2 and Arduino is done via serial communication, where the ROS 2 node sends commands to switch the lighting mode of the RGB LED strip.

## Project Overview

- The Arduino controls the RGB LED strip by switching between different lighting modes based on commands sent from a ROS 2 node.
- The ROS 2 node can send commands either by subscribing to a topic (dynamic control).

### Lighting Modes

1. **Mode 1**: Steady white light.
2. **Mode 2**: Flashing amber light.
3. **Mode 0**: Turn off the LEDs.

## Requirements

### Hardware

- Arduino (e.g., Arduino Uno)
- WS2812 RGB LED strip
- Jumper wires and breadboard (optional)
- USB cable to connect Arduino to your computer

### Software

- ROS 2 Humble (or any ROS 2 distribution)
- Python 3.x
- Arduino IDE
- Adafruit NeoPixel library for Arduino

## Project Structure

```sh
ros2_lighting_serial/
├── src/
│   └── arduino_commander/
│       ├── arduino_commander/
│       │   └── arduino_commander.py
│       └── setup.py
└── arduino/
    └── lightning_mods.ino 
```

## Setup Instructions

### 1. Hardware Setup

- Connect the WS2812 LED strip to the Arduino:
  - **LED_PIN**: Connect the `DI` pin of the WS2812 strip to Arduino pin 6.
  - **Power**: Connect the power and ground lines of the strip to the 5V and GND pins on the Arduino.
  
### 2. Arduino Setup

1. Install the [Adafruit NeoPixel library](https://github.com/adafruit/Adafruit_NeoPixel) in the Arduino IDE.
2. Upload the following code to your Arduino:

```cpp
#include <Adafruit_NeoPixel.h>

#define LED_PIN 6           // Pin where WS2812 DI is connected
#define NUM_LEDS 8          // Number of LEDs
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

int current_mode = 0;

void setup() {
  strip.begin();
  strip.show();
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    switchLightMode(command);
  }

  if (current_mode == 1) {
    lightMode1();
  } else if (current_mode == 2) {
    lightMode2();
  }
}

void switchLightMode(char command) {
  if (command == '1') {
    current_mode = 1;
  } else if (command == '2') {
    current_mode = 2;
  } else {
    current_mode = 0;
  }
}

void lightMode1() {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(255, 255, 255));
  }
  strip.show();
}

void lightMode2() {
  static bool state = false;
  state = !state;
  if (state) {
    for (int i = 0; i < NUM_LEDS; i++) {
      strip.setPixelColor(i, strip.Color(255, 165, 0));
    }
  } else {
    strip.clear();
  }
  strip.show();
  delay(500);
}
```

### 3. ROS 2 Setup

1. **Create a ROS 2 workspace:**

   ```bash
   mkdir -p ~/ros2_lighting_serial/src
   cd ~/ros2_lighting_serial
   ```

2. **Clone or create the `arduino_commander` package:**

   ```bash
   cd ~/ros2_lighting_serial/src
   ros2 pkg create --build-type ament_python arduino_commander
   ```

3. **Edit `arduino_commander.py`** to either subscribe to a topic or take user input (see details below).

4. **Build the ROS 2 workspace:**

   ```bash
   cd ~/ros2_lighting_serial
   colcon build
   ```

5. **Source the ROS 2 environment:**

   ```bash
   source install/setup.bash
   ```

## Usage

### Dynamic Control with ROS 2 Topic

1. **Run the ROS 2 node:**

   ```bash
   ros2 run arduino_commander arduino_commander
   ```

2. **Publish commands to the `/led_command` topic:**

   In a new terminal, use the following commands to change the LED modes:

   ```bash
   ros2 topic pub /led_command std_msgs/String "{data: '1'}"
   ros2 topic pub /led_command std_msgs/String "{data: '2'}"
   ros2 topic pub /led_command std_msgs/String "{data: '0'}"
   ```

## Troubleshooting

1. **No Executable Found Error**: Ensure that the `setup.py` file is correctly configured with the entry point.
2. **Serial Connection Issues**: Verify that the correct serial port is set for your system (e.g., `/dev/ttyUSB0` for Linux or `COMX` for Windows).
3. **LED Strip Not Responding**: Check the wiring between the Arduino and LED strip, and ensure the correct library is installed.
