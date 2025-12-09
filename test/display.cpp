#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// OLED display dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1  // Reset pin (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  // I2C address for 128x32 display (0x3C or 0x3D)

// Create display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  // Initialize serial for debugging
  Serial.begin(9600);
  delay(100);
  Serial.println(F("OLED Display Test Starting..."));

  // Initialize I2C
  Wire.begin();
  
  // Initialize display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    // Halt if display initialization fails
    while (1) {
      delay(1000);
    }
  }

  Serial.println(F("Display initialized successfully!"));
  
  // Clear the display buffer
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  // Display startup message
  display.println(F("NO one noticed"));
  display.println(F(""));
  display.println();
  display.println(F("..."));
  
  display.display();
  delay(2000);
}

void loop() {
  // Synced with song timing - EXTRA SLOW VERSION
  
  // "No one tried"
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15, 12);
  display.println(F("No one tried"));
  display.display();
  delay(4500);
  
  // "To read my eyes"
  display.clearDisplay();
  display.setCursor(10, 12);
  display.println(F("To read my eyes"));
  display.display();
  delay(4500);
  
  // "No one but you"
  display.clearDisplay();
  display.setCursor(10, 12);
  display.println(F("No one but you"));
  display.display();
  delay(4500);
  
  // "Wish it weren't true"
  display.clearDisplay();
  display.setCursor(5, 12);
  display.println(F("Wish it weren't"));
  display.setCursor(25, 22);
  display.println(F("true"));
  display.display();
  delay(5500);
  
  // "Maybe I" + "(I'd kinda like it if you'd call me)"
  display.clearDisplay();
  display.setCursor(20, 4);
  display.println(F("Maybe I"));
  display.setCursor(5, 16);
  display.println(F("Kinda like it if"));
  display.setCursor(10, 24);
  display.println(F("you'd call me"));
  display.display();
  delay(7000);
  
  // "It's not right" + "('cause I'm so over being lonely)"
  display.clearDisplay();
  display.setCursor(12, 4);
  display.println(F("It's not right"));
  display.setCursor(8, 16);
  display.println(F("I'm so over being"));
  display.setCursor(25, 24);
  display.println(F("lonely"));
  display.display();
  delay(7000);
  
  // "Make you mine" + "(I need a virtual connection)"
  display.clearDisplay();
  display.setCursor(10, 4);
  display.println(F("Make you mine"));
  display.setCursor(5, 16);
  display.println(F("I need a virtual"));
  display.setCursor(15, 24);
  display.println(F("connection"));
  display.display();
  delay(7000);
  
  // "Take our time" + "(be my video obsession)"
  display.clearDisplay();
  display.setCursor(10, 4);
  display.println(F("Take our time"));
  display.setCursor(10, 16);
  display.println(F("Be my video"));
  display.setCursor(20, 24);
  display.println(F("obsession"));
  display.display();
  delay(7500);
  
  // Pause before loop
  display.clearDisplay();
  display.display();
  delay(3000);
}
