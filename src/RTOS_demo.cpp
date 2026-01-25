#include <Arduino.h>
#include <MotorControl.h>
#include <QTRSensors.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

// ============================================
// STATE MACHINE + RTOS SIMPLIFIED DEMO
// ============================================

// ========== PIN DEFINITIONS ==========
#define AIN1 5
#define BIN1 6
#define AIN2 4
#define BIN2 8
#define PWMA 3
#define PWMB 9

#define sw1 10
#define sw2 11
#define sw3 12
#define led 7

// ========== SENSOR SETUP ==========
#define NUM_SENSORS 8
QTRSensors qtra;
uint16_t sensors1[NUM_SENSORS];
uint16_t thr[NUM_SENSORS];

// ========== MOTOR SETUP ==========
Motor motor1 = Motor(AIN1, AIN2, PWMA, 1, 1);
Motor motor2 = Motor(BIN1, BIN2, PWMB, 1, 1);

// ========== PID PARAMETERS ==========
#define MaxSpeed 255
#define BaseSpeed 230
int lastError = 0;
float kp = 0.161;
float kd = 0.54;

// ========== PATH STORAGE ==========
char path[100];
int path_length = 0;
int chr = 1;  // Rule selection (1=left, 2=right)

// ========== STATE DEFINITIONS ==========
typedef enum {
  STATE_IDLE = 0,
  STATE_CALIBRATING = 1,
  STATE_EXPLORING = 2,
  STATE_FINISHED = 3,
  STATE_RECOVERY = 4
} RobotState;

// ========== GLOBAL STATE & SYNCHRONIZATION ==========
RobotState currentState = STATE_IDLE;
SemaphoreHandle_t stateMutex;
unsigned long lastSensorReadTime = 0;

// ============================================
// HELPER FUNCTIONS (Keep existing ones)
// ============================================

void follow_segment() {
  while (1) {
    digitalWrite(led, LOW);
    int position = qtra.readLineWhite(sensors1);
    int error = 3500 - position;
    int motorSpeed = kp * error + kd * (error - lastError);
    lastError = error;
    
    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;
    
    if (rightMotorSpeed > MaxSpeed) rightMotorSpeed = MaxSpeed;
    if (leftMotorSpeed > MaxSpeed) leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;
    if (leftMotorSpeed < 0) leftMotorSpeed = 0;
    
    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);
    
    if ((sensors1[0] < thr[0]) || (sensors1[7] < thr[7])) {
      return;  // Found intersection
    }
    else if (sensors1[1] > thr[1] && sensors1[2] > thr[2] && sensors1[3] > thr[3] && 
             sensors1[4] > thr[4] && sensors1[5] > thr[5] && sensors1[6] > thr[6]) {
      return;  // Dead end
    }
    
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void follow_segment_fast() {
  float Kp = 0.2;
  float Kd = 10;
  for (int j = 0; j < 25; j++) {
    int position = qtra.readLineWhite(sensors1);
    int error = 3500 - position;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
    
    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;
    
    if (rightMotorSpeed > MaxSpeed) rightMotorSpeed = MaxSpeed;
    if (leftMotorSpeed > MaxSpeed) leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;
    if (leftMotorSpeed < 0) leftMotorSpeed = 0;
    
    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);
    
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void simplify_path() {
  if (path_length < 3 || path[path_length - 2] != 'B')
    return;
    
  int total_angle = 0;
  int m;
  
  for (m = 1; m <= 3; m++) {
    switch (path[path_length - m]) {
      case 'R':
        total_angle += 90;
        break;
      case 'L':
        total_angle += 270;
        break;
      case 'B':
        total_angle += 180;
        break;
    }
  }
  
  total_angle = total_angle % 360;
  switch (total_angle) {
    case 0:
      path[path_length - 3] = 'S';
      break;
    case 90:
      path[path_length - 3] = 'R';
      break;
    case 180:
      path[path_length - 3] = 'B';
      break;
    case 270:
      path[path_length - 3] = 'L';
      break;
  }
  path_length -= 2;
}

char select_turnL(char found_left, char found_straight, char found_right) {
  if (found_left) return 'L';
  else if (found_straight) return 'S';
  else if (found_right) return 'R';
  else return 'B';
}

char select_turnR(char found_right, char found_straight, char found_left) {
  if (found_right) return 'R';
  else if (found_straight) return 'S';
  else if (found_left) return 'L';
  else return 'B';
}

// ========== TURN WITH TIMEOUT ==========
bool attemptTurn(char dir) {
  unsigned long timeout = 2000;
  unsigned long startTime = millis();
  
  switch (dir) {
    case 'L': {
      left(motor1, motor2, 300);
      qtra.readLineWhite(sensors1);
      
      // Wait for left sensor to go from white to black (with timeout)
      while (sensors1[0] > thr[0] && (millis() - startTime) < timeout) {
        qtra.readLineWhite(sensors1);
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }
      
      if ((millis() - startTime) >= timeout) {
        Serial.println("LEFT-TURN TIMEOUT PHASE 1");
        brake(motor1, motor2);
        return false;
      }
      
      qtra.readLineWhite(sensors1);
      left(motor1, motor2, 200);
      startTime = millis();
      
      // Wait for left sensor to go back to white (with timeout)
      while (sensors1[0] < thr[0] && (millis() - startTime) < timeout) {
        qtra.readLineWhite(sensors1);
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }
      
      if ((millis() - startTime) >= timeout) {
        Serial.println("LEFT-TURN TIMEOUT PHASE 2");
        brake(motor1, motor2);
        return false;
      }
      
      brake(motor1, motor2);
      vTaskDelay(50 / portTICK_PERIOD_MS);
      follow_segment_fast();
      brake(motor1, motor2);
      return true;
    }
    
    case 'R': {
      right(motor1, motor2, 300);
      qtra.readLineWhite(sensors1);
      
      while (sensors1[7] > thr[7] && (millis() - startTime) < timeout) {
        qtra.readLineWhite(sensors1);
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }
      
      if ((millis() - startTime) >= timeout) {
        Serial.println("RIGHT-TURN TIMEOUT PHASE 1");
        brake(motor1, motor2);
        return false;
      }
      
      qtra.readLineWhite(sensors1);
      right(motor1, motor2, 200);
      startTime = millis();
      
      while (sensors1[7] < thr[7] && (millis() - startTime) < timeout) {
        qtra.readLineWhite(sensors1);
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }
      
      if ((millis() - startTime) >= timeout) {
        Serial.println("RIGHT-TURN TIMEOUT PHASE 2");
        brake(motor1, motor2);
        return false;
      }
      
      brake(motor1, motor2);
      vTaskDelay(50 / portTICK_PERIOD_MS);
      follow_segment_fast();
      brake(motor1, motor2);
      return true;
    }
    
    case 'B': {
      right(motor1, motor2, 250);
      qtra.readLineWhite(sensors1);
      
      while (sensors1[7] > thr[7] && (millis() - startTime) < timeout) {
        qtra.readLineWhite(sensors1);
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }
      
      if ((millis() - startTime) >= timeout) {
        Serial.println("BACK-TURN TIMEOUT PHASE 1");
        brake(motor1, motor2);
        return false;
      }
      
      qtra.readLineWhite(sensors1);
      right(motor1, motor2, 150);
      startTime = millis();
      
      while (sensors1[7] < thr[7] && (millis() - startTime) < timeout) {
        qtra.readLineWhite(sensors1);
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }
      
      if ((millis() - startTime) >= timeout) {
        Serial.println("BACK-TURN TIMEOUT PHASE 2");
        brake(motor1, motor2);
        return false;
      }
      
      brake(motor1, motor2);
      vTaskDelay(50 / portTICK_PERIOD_MS);
      follow_segment_fast();
      brake(motor1, motor2);
      return true;
    }
    
    case 'S':
      return true;  // Straight, no turn needed
    
    default:
      return false;
  }
}

// ========== STATE TRANSITION FUNCTION ==========
void transitionToState(RobotState newState) {
  if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
    RobotState oldState = currentState;
    currentState = newState;
    
    Serial.print("STATE TRANSITION: ");
    Serial.print(oldState);
    Serial.print(" -> ");
    Serial.println(newState);
    
    xSemaphoreGive(stateMutex);
  }
}

// ============================================
// STATE HANDLERS
// ============================================

void handleStateIdle() {
  Serial.println("[STATE] IDLE - Press SW1 to calibrate");
  digitalWrite(led, LOW);
  
  int s1 = digitalRead(sw1);
  if (s1 == LOW) {
    Serial.println("SW1 pressed - starting calibration");
    vTaskDelay(800 / portTICK_PERIOD_MS);
    transitionToState(STATE_CALIBRATING);
  }
}

void handleStateCalibrating() {
  static bool calibrationStarted = false;
  
  if (!calibrationStarted) {
    Serial.println("[STATE] CALIBRATING - Swinging left/right");
    digitalWrite(led, HIGH);
    calibrationStarted = true;
    return;
  }
  
  // Calibration cycle - swing left and right
  for (int i = 0; i <= 100; i++) {
    if (i < 25 || i >= 75) {
      left(motor1, motor2, 150);
    } else {
      right(motor1, motor2, 150);
    }
    qtra.calibrate();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  
  brake(motor1, motor2);
  
  // Calculate thresholds
  for (int i = 0; i < NUM_SENSORS; i++) {
    thr[i] = (qtra.calibrationOn.minimum[i] + qtra.calibrationOn.maximum[i]) / 2;
  }
  
  Serial.println("Calibration complete!");
  Serial.print("Threshold values: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(thr[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  digitalWrite(led, LOW);
  
  // Wait for SW2 to select rule
  Serial.println("Press SW2 for left-hand rule, SW3 for right-hand rule");
  
  int s2 = digitalRead(sw2);
  int s3 = digitalRead(sw3);
  
  if (s2 == LOW) {
    chr = 1;
    Serial.println("Left-hand rule selected");
    vTaskDelay(800 / portTICK_PERIOD_MS);
    transitionToState(STATE_EXPLORING);
  } else if (s3 == LOW) {
    chr = 2;
    Serial.println("Right-hand rule selected");
    vTaskDelay(800 / portTICK_PERIOD_MS);
    transitionToState(STATE_EXPLORING);
  }
}

void handleStateExploring() {
  static bool exploringStarted = false;
  
  if (!exploringStarted) {
    Serial.println("[STATE] EXPLORING - Solving maze");
    digitalWrite(led, HIGH);
    exploringStarted = true;
    return;
  }
  
  // Follow line to intersection
  follow_segment();
  
  digitalWrite(led, HIGH);
  brake(motor1, motor2);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  
  // Move forward and check for left/right paths
  forward(motor1, motor2, 255);
  vTaskDelay(5 / portTICK_PERIOD_MS);
  brake(motor1, motor2);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  
  unsigned char found_left = 0;
  unsigned char found_straight = 0;
  unsigned char found_right = 0;
  
  qtra.readLineWhite(sensors1);
  
  if (sensors1[7] < 700) found_right = 1;
  if (sensors1[0] < 700) found_left = 1;
  
  // Move further forward
  forward(motor1, motor2, 255);
  vTaskDelay(60 / portTICK_PERIOD_MS);
  brake(motor1, motor2);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  
  qtra.readLineWhite(sensors1);
  
  // Check for straight path
  if (sensors1[1] < 700 || sensors1[2] < 700 || sensors1[3] < 700 || 
      sensors1[4] < 700 || sensors1[5] < 700 || sensors1[6] < 700) {
    found_straight = 1;
  }
  
  // Check for END POINT (all middle sensors on black)
  if (sensors1[1] < 700 && sensors1[2] < 700 && sensors1[3] < 700 && 
      sensors1[4] < 700 && sensors1[5] < 700 && sensors1[6] < 700) {
    
    Serial.println("*** MAZE SOLVED - END POINT FOUND ***");
    path[path_length] = 'S';
    path_length++;
    
    Serial.print("Path taken: ");
    for (int i = 0; i < path_length; i++) {
      Serial.print(path[i]);
      Serial.print(" ");
    }
    Serial.println();
    
    exploringStarted = false;
    transitionToState(STATE_FINISHED);
    return;
  }
  
  // Select turn direction
  char dir;
  if (chr == 1)
    dir = select_turnL(found_left, found_straight, found_right);
  else
    dir = select_turnR(found_right, found_straight, found_left);
  
  Serial.println(dir);
  
  // Attempt turn
  if (attemptTurn(dir)) {
    path[path_length] = dir;
    path_length++;
    simplify_path();
    
    // Check path bounds
    if (path_length >= 99) {
      Serial.println("Path buffer full!");
      exploringStarted = false;
      transitionToState(STATE_FINISHED);
    }
    // Stay in exploring, will loop again
    
  } else {
    Serial.println("Turn failed - recovery needed");
    exploringStarted = false;
    transitionToState(STATE_RECOVERY);
  }
}

void handleStateRecovery() {
  static bool recoveryStarted = false;
  
  if (!recoveryStarted) {
    Serial.println("[STATE] RECOVERY - Attempting to recover");
    recoveryStarted = true;
    return;
  }
  
  brake(motor1, motor2);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  
  // Try forward movement to find line
  forward(motor1, motor2, 100);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  brake(motor1, motor2);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  
  // Check if line found
  qtra.readLineWhite(sensors1);
  int centerCount = 0;
  for (int i = 1; i < 7; i++) {
    if (sensors1[i] < 700) centerCount++;
  }
  
  if (centerCount >= 3) {
    Serial.println("Line found - resuming exploration");
    recoveryStarted = false;
    transitionToState(STATE_EXPLORING);
  } else {
    Serial.println("Still searching for line...");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // Stay in recovery, try again next cycle
  }
}

void handleStateFinished() {
  static bool finishStarted = false;
  
  if (!finishStarted) {
    Serial.println("[STATE] FINISHED");
    digitalWrite(led, HIGH);
    finishStarted = true;
    return;
  }
  
  Serial.println("Exploration complete! Press SW2 to run optimized path or SW1 to restart");
  
  int s1 = digitalRead(sw1);
  int s2 = digitalRead(sw2);
  
  if (s1 == LOW) {
    Serial.println("Restart selected");
    // Reset everything
    path_length = 0;
    lastError = 0;
    finishStarted = false;
    vTaskDelay(800 / portTICK_PERIOD_MS);
    transitionToState(STATE_IDLE);
  } else if (s2 == LOW) {
    Serial.println("Running optimized path...");
    // For demo, just do a simple forward
    forward(motor1, motor2, 100);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    brake(motor1, motor2);
    finishStarted = false;
  }
}

// ============================================
// RTOS TASKS
// ============================================

// Main state machine task - runs every 50ms
void stateTaskFunction(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
  
  while (1) {
    // Execute state handler
    switch (currentState) {
      case STATE_IDLE:
        handleStateIdle();
        break;
      case STATE_CALIBRATING:
        handleStateCalibrating();
        break;
      case STATE_EXPLORING:
        handleStateExploring();
        break;
      case STATE_RECOVERY:
        handleStateRecovery();
        break;
      case STATE_FINISHED:
        handleStateFinished();
        break;
      default:
        currentState = STATE_IDLE;
        break;
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Watchdog task - monitors for hangs (every 500ms)
void watchdogTaskFunction(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 500 / portTICK_PERIOD_MS;
  unsigned long watchdogTimeout = 0;
  
  while (1) {
    if (currentState == STATE_EXPLORING) {
      watchdogTimeout++;
      
      // If stuck in exploring for more than 30 seconds (60 iterations * 500ms)
      if (watchdogTimeout > 60) {
        Serial.println("WATCHDOG: Timeout in exploring state!");
        brake(motor1, motor2);
        transitionToState(STATE_RECOVERY);
        watchdogTimeout = 0;
      }
    } else {
      watchdogTimeout = 0;
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ============================================
// SETUP AND LOOP
// ============================================

void setup() {
  // Serial communication
  Serial.begin(9600);
  delay(500);
  Serial.println("\n\n=== MICROMOUSE RTOS STATE MACHINE DEMO ===");
  Serial.println("Press SW1 to start calibration");
  
  // Sensor setup
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]) {A7, A6, A5, A4, A3, A2, A1, A0}, NUM_SENSORS);
  
  // Pin setup
  pinMode(sw1, INPUT);
  pinMode(sw2, INPUT);
  pinMode(sw3, INPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  
  // Create mutex for thread-safe state transitions
  stateMutex = xSemaphoreCreateMutex();
  
  // Create RTOS tasks
  // Task name, stack size, parameter, priority, task handle
  xTaskCreate(stateTaskFunction, "StateMachine", 512, NULL, 2, NULL);
  xTaskCreate(watchdogTaskFunction, "Watchdog", 256, NULL, 3, NULL);
  
  Serial.println("RTOS initialized. Starting scheduler...\n");
  
  // RTOS scheduler starts automatically
}

// Loop is not used in RTOS mode - all work happens in tasks
void loop() {
  vTaskDelete(NULL);
}