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

#define BUTTON_PIN 10

const int offsetA = 1;
const int offsetB = 1;

#define LED_PIN 7

// Create motor instances
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, 1); 
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, 1);

// State variables
unsigned long runStartTime = 0;
bool isRunning = false;
bool waitingToStart = false;

void setup()
{
  Serial.begin(9600);
  Serial.println("Motor Test - Press button to start");
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}

void loop()
{
  // Check if button is pressed to start
  if (digitalRead(BUTTON_PIN) == LOW && !isRunning && !waitingToStart)
  {
    delay(20);  // Debounce
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      waitingToStart = true;
      runStartTime = millis();
      digitalWrite(LED_PIN, HIGH);
      Serial.println("Button pressed! Waiting 2 seconds...");
    }
  }

  // Wait 2 seconds after button press before starting motor
  if (waitingToStart && !isRunning)
  {
    if (millis() - runStartTime >= 1000)
    {
      isRunning = true;
      runStartTime = millis();
      Serial.println("Started!");
    }
  }

  // If running, check if 1 second has passed
  if (isRunning)
  {
    if (millis() - runStartTime < 500)
    {
      forward(motor1, motor2, 255);  // Run at full speed
    }
    else
    {
      // Stop after 1 second
      brake(motor1, motor2);
      isRunning = false;
      waitingToStart = false;
      digitalWrite(LED_PIN, LOW);
      Serial.println("Stopped! Press button to run again.");
    }
  }
}
