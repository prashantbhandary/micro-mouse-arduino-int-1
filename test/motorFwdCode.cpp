// Simple motor forward test code
#include <Arduino.h>
#include <SparkFun_TB6612.h>

// Motor pin definitions
#define AIN1 5
#define BIN1 6
#define AIN2 4
#define BIN2 8

#define PWMA 3
#define PWMB 9


const int offsetA = 1;
const int offsetB = 1;

#define LED_PIN 7

// Create motor instances
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, 1); 
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, 1);

void setup()
{
  Serial.begin(9600);
  Serial.println("Motor Forward Test");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}

void loop()
{
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  unsigned long now = millis();
  if (now - lastBlink >= 500) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    lastBlink = now;
  }
  // Run motors forward
//   Serial.println("Motors Running Forward");
//   forward(motor1, motor2, 150);  // Speed 150
// //   delay(3000);  // Run for 3 seconds
  
//   // Stop motors
// //   Serial.println("Motors Stopped");
// //   brake(motor1, motor2);
// //   delay(2000);  // Pause for 2 seconds#
//   delay(3000);
 //  back(motor1, motor2, 255);  // Speed 150
    right(motor1, motor2, 255); 
    delay(5000);
    left(motor1, motor2, 255);
    delay(5000);
  // delay(3000);


 // back(motor1, motor2, 255); 

}
