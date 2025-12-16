const bool REVERSE_STEERING = true; 
const bool INVERT_LEFT_MOTOR = true; 
const bool INVERT_RIGHT_MOTOR = true;

const int ENA = 10;
const int IN1 = 9;
const int IN2 = 8;
const int IN3 = 7;
const int IN4 = 6;
const int ENB = 5;

const int TRIG_PIN = 3;
const int ECHO_PIN = 4;

const int SENSOR_COUNT = 8;
const int sensorPins[SENSOR_COUNT] = {A0, A1, A2, A3, A4, A5, A6, A7};

int sensorMin[SENSOR_COUNT];
int sensorMax[SENSOR_COUNT];

const int HISTORY_SIZE = 10;
int pathHistory[HISTORY_SIZE];
int historyIndex = 0;
unsigned long lastHistoryWrite = 0;

float Kp = 0.08;    
float Ki = 0.0001;
float Kd = 1.2;   

float P, I, D;
float error = 0;
float lastError = 0;
float smoothedDerivative = 0; 

const float integralLimit = 5000;


int currentBaseSpeed = 80;
const int targetBaseSpeed = 130;
const int maxSpeed = 220;       
const int turnSpeed = 130;      
const int minMotorSpeed = 110;   

const int speedRampRate = 2;    
unsigned long lastRampTime = 0;

const int stopDistance = 10; 
unsigned long lastSonarCheck = 0; 
int currentDistance = 999; 

int recoveryAttempts = 0;
unsigned long lostLineTime = 0;
const unsigned long maxRecoveryTime = 3000;


enum RobotState {
  CALIBRATING,
  RUNNING,
  LOST_LINE,
  OBSTACLE_DETECTED,
  STOPPED
};
RobotState currentState = CALIBRATING;

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(13, OUTPUT); // LED

  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(sensorPins[i], INPUT);
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }
  
  for(int i = 0; i < HISTORY_SIZE; i++) pathHistory[i] = 0;

  Serial.println(F("=== LINE FOLLOWER ROBOT ==="));
  Serial.println(F("Calibrating... Move sensors over line and background!"));
  
  digitalWrite(13, HIGH);
  unsigned long startTime = millis();
  
  
  while (millis() - startTime < 5000) {
    calibrateSensors();
    
    
    if ((millis() - startTime) % 500 < 10) {
      Serial.print(F("."));
    }
  }
  
  digitalWrite(13, LOW);
  Serial.println();
  Serial.println(F("Calibration complete!"));
  
  Serial.println(F("Sensor ranges (Min/Max):"));
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(F("S"));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.print(sensorMin[i]);
    Serial.print(F("/"));
    Serial.println(sensorMax[i]);
  }
  
  delay(1000);
  Serial.println(F("Starting in 3 seconds..."));
  delay(3000);
  
  currentState = RUNNING;
  Serial.println(F("GO!"));
}

void loop() {
  // 1. Obstacle Check (non-blocking)
  if (millis() - lastSonarCheck > 60) {
    lastSonarCheck = millis();
    currentDistance = getDistance();
  }
  
  if (currentDistance > 0 && currentDistance < stopDistance) {
    if (currentState != OBSTACLE_DETECTED) {
      Serial.print(F("OBSTACLE at "));
      Serial.print(currentDistance);
      Serial.println(F("cm"));
      currentState = OBSTACLE_DETECTED;
    }
    stopMotors();
    return; 
  } else if (currentState == OBSTACLE_DETECTED) {
    currentState = RUNNING;
    Serial.println(F("Path clear, resuming"));
  }

  // 2. Velocity Ramping (smooth acceleration)
  if (millis() - lastRampTime > 50 && currentBaseSpeed < targetBaseSpeed) {
    currentBaseSpeed += speedRampRate;
    if (currentBaseSpeed > targetBaseSpeed) currentBaseSpeed = targetBaseSpeed;
    lastRampTime = millis();
  }

  // 3. Read Sensors with improved filtering
  int position = readLinePosition();

  // 4. Calculate Error
  error = 3500 - position;

  // 5. Update Path History
  if (millis() - lastHistoryWrite > 50) {
    lastHistoryWrite = millis();
    pathHistory[historyIndex] = (int)error;
    historyIndex++;
    if (historyIndex >= HISTORY_SIZE) historyIndex = 0;
  }

  // 6. Lost Line Detection and Recovery
  if (abs(error) == 3500) {
    if (currentState != LOST_LINE) {
      lostLineTime = millis();
      recoveryAttempts = 0;
      currentState = LOST_LINE;
      Serial.println(F("Line lost! Recovering..."));
    }
    
    // Timeout check
    if (millis() - lostLineTime > maxRecoveryTime) {
      Serial.println(F("Recovery timeout! Stopping."));
      stopMotors();
      currentState = STOPPED;
      return;
    }
    
    recoverLostLine(); 
    return; 
  } else if (currentState == LOST_LINE) {
    currentState = RUNNING;
    Serial.println(F("Line reacquired!"));
    I = 0; // Reset integral on recovery
  }

  // 7. PID Calculation
  P = error;
  
  // Integral with windup prevention
  I += error;
  I = constrain(I, -integralLimit, integralLimit);
  
  // Derivative with smoothing
  float rawDerivative = error - lastError;
  smoothedDerivative = (smoothedDerivative * 0.7) + (rawDerivative * 0.3);
  D = smoothedDerivative;

  float motorSpeedCorrection = (Kp * P) + (Ki * I) + (Kd * D);
  lastError = error;

  // 8. Motor Speed Calculation
  int leftMotorSpeed, rightMotorSpeed;

  if (REVERSE_STEERING) {
    leftMotorSpeed = currentBaseSpeed + motorSpeedCorrection;
    rightMotorSpeed = currentBaseSpeed - motorSpeedCorrection;
  } else {
    leftMotorSpeed = currentBaseSpeed - motorSpeedCorrection;
    rightMotorSpeed = currentBaseSpeed + motorSpeedCorrection;
  }

  leftMotorSpeed = constrain(leftMotorSpeed, -maxSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -maxSpeed, maxSpeed);

  drive(leftMotorSpeed, rightMotorSpeed);

  // 9. Debug output (every 200ms)
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 200) {
    lastDebug = millis();
    Serial.print(F("Pos:"));
    Serial.print(position);
    Serial.print(F(" Err:"));
    Serial.print((int)error);
    Serial.print(F(" L:"));
    Serial.print(leftMotorSpeed);
    Serial.print(F(" R:"));
    Serial.println(rightMotorSpeed);
  }
}

//SENSOR READING
int readLinePosition() {
  int mappedValues[SENSOR_COUNT];
  int maxVal = 0;
  int maxIndex = 0;
  bool lineFound = false;
  int activeSensors = 0;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    int val = analogRead(sensorPins[i]);
    mappedValues[i] = map(val, sensorMin[i], sensorMax[i], 0, 1000);
    mappedValues[i] = constrain(mappedValues[i], 0, 1000);
    
    // Noise filter with slightly higher threshold
    if (mappedValues[i] < 60) mappedValues[i] = 0; 

    // Active sensor detection
    if (mappedValues[i] > 600) activeSensors++;

    // Track maximum
    if (mappedValues[i] > maxVal) {
      maxVal = mappedValues[i];
      maxIndex = i;
    }
    
    // Line detection threshold
    if (mappedValues[i] > 150) lineFound = true;
  }

  // INTERSECTION DETECTION 
  if (activeSensors > 5) {
    return 3500;
  }

  // LINE NOT FOUND
  if (!lineFound) {
    // Return extreme value based on last known direction
    if (REVERSE_STEERING) return (lastError < 0) ? 0 : 7000;
    else return (lastError < 0) ? 7000 : 0;
  }

  // WEIGHTED AVERAGE CALCULATION
  long weightedSum = 0;
  long sum = 0;
  
  
  int startIdx = max(0, maxIndex - 1);
  int endIdx = min(SENSOR_COUNT - 1, maxIndex + 1);
  
  for (int i = startIdx; i <= endIdx; i++) {
    if (mappedValues[i] > 50) { 
      weightedSum += (long)mappedValues[i] * (i * 1000L);
      sum += mappedValues[i];
    }
  }

  if (sum == 0) return 3500; 
  return (int)(weightedSum / sum);
}

// --- SMART RECOVERY WITH EXPONENTIAL SEARCH ---
void recoverLostLine() {
  
  long totalHistory = 0;
  for(int i = 0; i < HISTORY_SIZE; i++) {
    totalHistory += pathHistory[i];
  }
  float averageError = totalHistory / (float)HISTORY_SIZE;

  
  bool turnRight = (averageError < 0);

  
  recoveryAttempts++;
  int searchSpeed = turnSpeed + (recoveryAttempts * 5);
  searchSpeed = constrain(searchSpeed, turnSpeed, maxSpeed);

  if (turnRight) {
    drive(searchSpeed, -searchSpeed); 
  } else {
    drive(-searchSpeed, searchSpeed);
  }
}

// --- CALIBRATION ---
void calibrateSensors() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    int val = analogRead(sensorPins[i]);
    if (val > sensorMax[i]) sensorMax[i] = val;
    if (val < sensorMin[i]) sensorMin[i] = val;
  }
}

// --- ULTRASONIC SENSOR ---
int getDistance() {
  digitalWrite(TRIG_PIN, LOW); 
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); 
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 8000); 
  if (duration == 0) return 999; 
  
  int distance = duration * 0.034 / 2;
  return distance;
}

// --- MOTOR CONTROL ---
int preventStall(int pwm) {
  if (abs(pwm) < 40) return 0;
  
  if (pwm > 0 && pwm < minMotorSpeed) return minMotorSpeed;
  if (pwm < 0 && pwm > -minMotorSpeed) return -minMotorSpeed;
  
  return pwm;
}

void drive(int leftSpeed, int rightSpeed) {
  leftSpeed = preventStall(leftSpeed);
  rightSpeed = preventStall(rightSpeed);

  // Left Motor
  bool leftForward = !INVERT_LEFT_MOTOR; 
  if (leftSpeed < 0) { 
    leftForward = !leftForward; 
    leftSpeed = -leftSpeed; 
  }
  
  digitalWrite(IN1, leftForward ? HIGH : LOW);
  digitalWrite(IN2, leftForward ? LOW : HIGH);

  // Right Motor
  bool rightForward = !INVERT_RIGHT_MOTOR;
  if (rightSpeed < 0) { 
    rightForward = !rightForward; 
    rightSpeed = -rightSpeed; 
  }

  digitalWrite(IN3, rightForward ? HIGH : LOW);
  digitalWrite(IN4, rightForward ? LOW : HIGH);

  // Set speeds
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void stopMotors() {
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); 
  analogWrite(ENB, 0);
  I = 0;
}