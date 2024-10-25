#include <Arduino.h>

// Order of header is important.
#include <IoAbstractionWire.h>
#include <LiquidCrystalIO.h>
#include <Wire.h>

LiquidCrystalI2C_RS_EN(lcd, 0x3F, false);
int x = 0;
int y = 0;
float head = 0.0;


void setup() {
    Serial.begin(115200);
    Wire.begin();                  // This must be called to start I2C.
    lcd.begin(16, 2);              // Number of chars in a row and how many rows.
    lcd.configureBacklightPin(3);  // Backlight brightness (0 is off).
    lcd.backlight();               // Set backlight.
    lcd.clear();                   // Clear display of text.
}

void loop() {
    // COMPLETE THIS.
    lcd.setCursor(0,0);
    lcd.print("[");
    lcd.print(x);
    lcd.print(",");
    lcd.print(y);
    lcd.print(",");
    lcd.print(head);
    lcd.print("]");
   
}
