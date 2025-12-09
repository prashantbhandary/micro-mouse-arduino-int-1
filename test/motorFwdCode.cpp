// Simple motor forward test code
#include <Arduino.h>
#include <SparkFun_TB6612.h>

// Motor pin definitions
#define AIN1 4
#define BIN1 8
#define AIN2 5
#define BIN2 9
#define PWMA 3
#define PWMB 6

const int offsetA = 1;
const int offsetB = 1;

// Create motor instances
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, 1); 
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, 1);

void setup()
{
  Serial.begin(9600);
  Serial.println("Motor Forward Test");
  delay(1000);
}

void loop()
{
  // Run motors forward
  Serial.println("Motors Running Forward");
  forward(motor1, motor2, 150);  // Speed 150
//   delay(3000);  // Run for 3 seconds
  
  // Stop motors
//   Serial.println("Motors Stopped");
//   brake(motor1, motor2);
//   delay(2000);  // Pause for 2 seconds
}
