#include <Adafruit_NeoPixel.h>

#define LED_PIN 6           // Pin where WS2812 DI is connected
#define NUM_LEDS 8          // Number of LEDs
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Variables to store current mode
int current_mode = 0;

void setup() {
  // Initialize NeoPixel strip
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  
  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Check for serial input from ROS 2 node
  if (Serial.available() > 0) {
    char command = Serial.read();
    switchLightMode(command);
  }

  // Call the function corresponding to the current mode
  if (current_mode == 1) {
    lightMode1();  // Example mode 1
  } else if (current_mode == 2) {
    lightMode2();  // Example mode 2
  }
}

// Function to switch between light modes based on command received
void switchLightMode(char command) {
  if (command == '1') {
    current_mode = 1;  // Switch to mode 1
  } else if (command == '2') {
    current_mode = 2;  // Switch to mode 2
  } else {
    current_mode = 0;  // Turn off lights
  }
}

// Example light mode 1 (steady white)
void lightMode1() {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(255, 255, 255));  // White light
  }
  strip.show();
}

// Example light mode 2 (flashing amber)
void lightMode2() {
  static bool state = false;
  state = !state;  // Toggle state
  if (state) {
    for (int i = 0; i < NUM_LEDS; i++) {
      strip.setPixelColor(i, strip.Color(255, 165, 0));  // Amber light
    }
  } else {
    strip.clear();  // Turn off
  }
  strip.show();
  delay(500);  // Flashing delay
}
