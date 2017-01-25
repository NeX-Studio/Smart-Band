#include <Wire.h>
#include <RtcDS3231.h>
#include "FastLED.h"
#include <U8g2lib.h>
#include <U8x8lib.h>
//LED
#define NUM_LEDS 1
#define LED_PIN 1
CRGB leds[NUM_LEDS]; // Initialize the LED array
//LED end
//OLED
U8G2_SSD1306_128X32_UNIVISION_2_HW_I2C u8g2(U8G2_R0); // Initialize the OLED screen
//OLED end
//MAX 30100

//MAX 30100 end
//RTC
RtcDS3231<TwoWire> Rtc(Wire);
//RTC End
void setup() {
	Serial.begin(9600);
	Serial.println("System initializing...");
  	Serial.println("Initializing modules...");
  	u8g2.begin();
  	FastLED.addLeds<WS2811,LED_PIN,RGB>(leds,NUM_LEDS);
  	Serial.println("Initialization completed");
}

void loop() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(0, 24, "Hello World!");
  } while ( u8g2.nextPage() );
}

