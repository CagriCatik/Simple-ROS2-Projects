#include <Adafruit_NeoPixel.h>

#define LED_PIN 6           // Pin where WS2812 DI is connected
#define NUM_LEDS 8          // Total number of LEDs in the strip
int front_light_brightness = 0;  // Dynamic brightness (starting low)
bool is_high_beam = false;       // Track whether high beam is on
#define LOW_BEAM 0               // Low brightness value
#define HIGH_BEAM 255            // High brightness value
#define FLASH_DELAY 500          // Delay between flashes (milliseconds)

// Create an instance of the Adafruit_NeoPixel class
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  // Initialize NeoPixel strip
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  // Continuously flash the front light between low and high brightness
  setFrontLight();
  toggleFlash();   // Toggle between low and high beams
  delay(FLASH_DELAY);  // Control the speed of flashing
}

// Function to set the front light with the current brightness level
void setFrontLight() {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(front_light_brightness, front_light_brightness, front_light_brightness)); // White light
  }
  strip.show();
}

// Function to toggle the brightness between low beam and high beam
void toggleFlash() {
  if (is_high_beam) {
    front_light_brightness = LOW_BEAM;   // Switch to low beam
  } else {
    front_light_brightness = HIGH_BEAM;  // Switch to high beam
  }
  is_high_beam = !is_high_beam;  // Toggle the state for the next loop
}
