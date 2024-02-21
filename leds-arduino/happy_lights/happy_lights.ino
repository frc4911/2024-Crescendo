#include <FastLED.h>

// Number of LEDs in strip
#define NUM_LEDS 27 

// Where the LED signal line is connected
#define DATA_PIN 2

#define DEFAULT 0
#define CYLON 1
#define FLASH 2
#define CONFETTI 3
#define SINELON 4
#define BPM 5
#define JUGGLE 6

// Define the array of leds
CRGB leds[NUM_LEDS];

uint8_t hue = 0;
uint8_t pattern = 0;

void setup() { 
	FastLED.addLeds<WS2812,DATA_PIN,RGB>(leds,NUM_LEDS);
	FastLED.setBrightness(84);
}

void loop() {
  int hueSensorValue = analogRead(A0);
  hue = hueSensorValue * (255.0 / 1023.0);
  
  int patternSensorValue = analogRead(A1);
  pattern = 1023.0 / 40.0 +  patternSensorValue * (20.0 / 1023.0);
  
  switch (patternSensorValue) {
    case CYLON:
      cylon();
      break;
    case FLASH:
      flash();
      break;
    case CONFETTI:
      confetti();
      break;
    case SINELON:
      sinelon();
      break;
    case BPM:
      bpm();
      break;
    case JUGGLE:
      juggle();
      break;
    default:
      sinelon();
      break;
  }
}

void fadeall() {
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i].nscale8(240);
  }
}

void cylon() {
	// First slide the led in one direction
	for(int i = 0; i < NUM_LEDS; i++) {
		// Set the i'th led to red 
		leds[i] = CHSV(hue, 255, 255);
		// Show the leds
		FastLED.show(); 
		// now that we've shown the leds, reset the i'th led to black
		// leds[i] = CRGB::Black;
		fadeall();
		// Wait a little bit before we loop around and do it again
    // NUM_LEDS - NUM_LEDS / 2 - abs(i)
		delay(30);
	}

	// Now go in the other direction.  
	for(int i = (NUM_LEDS)-1; i >= 0; i--) {
		// Set the i'th led to red 
		leds[i] = CHSV(hue, 255, 255);
		// Show the leds
		FastLED.show();
		// now that we've shown the leds, reset the i'th led to black
		// leds[i] = CRGB::Black;
		fadeall();
		// Wait a little bit before we loop around and do it again
		delay(30);
	}
}

void flash() {
	for(int i = 0; i < NUM_LEDS; i++) {
		leds[i] = CHSV(hue, 255, 255);
  }
  FastLED.show();
  delay(30);
  
	for(int i = 0; i < NUM_LEDS; i++) {
		leds[i] = CRGB::Black;
  }
  FastLED.show();
  delay(30);
}
 
void confetti() {
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( hue + random8(64), 200, 255);
}
 
void sinelon() {
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS-1 );
  leds[pos] += CHSV( hue, 255, 192);
}
 
void bpm() {
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, hue+(i*2), beat-hue+(i*10));
  }
}
 
void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  uint8_t dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16( i+7, 0, NUM_LEDS-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}
