#include <Adafruit_NeoPixel.h>
#include <U8g2lib.h>
#include <U8x8lib.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
U8G2_SSD1306_128X32_UNIVISION_2_HW_I2C u8g2(U8G2_R0);
void setup() {
  // put your setup code here, to run once:
u8g2.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(0,24,"Hello World!");
  } while ( u8g2.nextPage() );
}
