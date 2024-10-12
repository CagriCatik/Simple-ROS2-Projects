#include <Adafruit_NeoPixel.h>

#define LED_PIN 6           // Pin where WS2812 DI is connected
#define NUM_LEDS 8          // Total number of LEDs in the strip
int front_light_brightness = 1; // Dynamic brightness (starting value)
int brightness_increment = 1;   // Increment for changing brightness

// Create an instance of the Adafruit_NeoPixel class
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  // Initialize NeoPixel strip
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  // Continuously update and display the dynamic front light
  setFrontLight();
  updateBrightness();
  delay(20);  // Delay to control the speed of the brightness change
}

// Function to set the white front light with the current brightness
void setFrontLight() {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(front_light_brightness, front_light_brightness, front_light_brightness)); // White light
  }
  strip.show();
}

// Function to dynamically update the brightness level
void updateBrightness() {
  // Update brightness by incrementing or decrementing
  front_light_brightness += brightness_increment;

  // Reverse direction if brightness is out of bounds
  if (front_light_brightness >= 255 || front_light_brightness <= 0) {
    brightness_increment = -brightness_increment;
  }
}
