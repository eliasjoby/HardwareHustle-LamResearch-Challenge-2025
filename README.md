# Hardware Hustle: Lam Research Challenge 2025 

> Advanced dual-robot coordination system with PID line following, BLE-controlled mecanum platform, 5-DOF robotic arm with physics engine, and FSM-based arena control.

## Technical Architecture

### System Overview
```
ALFR (Arduino Nano)          Arena FSM (ESP32)           SARM (Dual ESP32)
├─ 8-ch Line Sensor      ──▶ ├─ Gate Triggers        ◀── ├─ Base: Mecanum Control
├─ PID Controller            ├─ Pump Control              ├─ Arm: Physics Engine
├─ Ultrasonic Sensor         ├─ Load Cell                 └─ BLE Communication
└─ State Machine             └─ LCD Display
```

## Code Structure
```
polyizer-lam-challenge/
│
├── alfr_line_follower.ino       # Advanced PID line follower
├── sarm_arm_controller.ino      # 5-DOF arm with spring-damper physics
├── sarm_base_mecanum.ino        # Omnidirectional mecanum base
└── arena_gates_control.ino      # FSM game controller
```

## 1. ALFR - Advanced Line Follower Robot

### Core Features
- **PID Control Loop**: Kp=0.08, Ki=0.0001, Kd=1.2
- **8-Channel Weighted Average Positioning**
- **Non-blocking Ultrasonic Obstacle Detection**
- **Exponential Lost-Line Recovery**
- **Velocity Ramping & Anti-Stall Logic**

### Key Algorithms

#### Weighted Average Line Position
```cpp
int readLinePosition() {
  // Map sensor values to 0-1000 range
  // Calculate weighted average: position = Σ(value[i] * position[i]) / Σ(value[i])
  // Returns: 0 (far left) to 7000 (far right), center = 3500
}
```

#### PID Calculation
```cpp
P = error;                                          // Proportional
I += error; I = constrain(I, -5000, 5000);         // Integral with anti-windup
D = (error - lastError) * 0.3 + D * 0.7;           // Smoothed derivative
correction = (Kp * P) + (Ki * I) + (Kd * D);
```

#### Exponential Recovery
```cpp
void recoverLostLine() {
  // Analyze path history to determine turn direction
  // Increase search speed exponentially with each attempt
  // searchSpeed = turnSpeed + (recoveryAttempts * 5)
}
```

### Configuration
```cpp
// Motor pins (L298N)
const int ENA = 10, IN1 = 9, IN2 = 8;
const int IN3 = 7, IN4 = 6, ENB = 5;

// Sensors
const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
const int TRIG_PIN = 3, ECHO_PIN = 4;

// PID tuning
float Kp = 0.08, Ki = 0.0001, Kd = 1.2;
const int targetBaseSpeed = 130;
const int maxSpeed = 220;
const int stopDistance = 10; // cm
```

### State Machine
```cpp
enum RobotState {
  CALIBRATING,        // 5-second sensor calibration
  RUNNING,            // Active line following
  LOST_LINE,          // Exponential search recovery
  OBSTACLE_DETECTED,  // Stopped, waiting for clearance
  STOPPED             // Recovery timeout
};
```

## 2. SARM - Split-Brain Robotic Arm System

### Architecture Philosophy
Two independent ESP32 controllers eliminate I/O conflicts and enable parallel processing:
- **Base ESP32**: Handles 4-wheel mecanum kinematics
- **Arm ESP32**: Executes physics-based servo control

### Arm Controller - Physics Engine

#### Spring-Damper Servo System
```cpp
class SmoothServo {
  // Mass-spring-damper model prevents mechanical stress
  float displacement = targetPos - currentPos;
  float force = displacement * SPRING_TENSION;      // k = 0.08
  float dampingForce = velocity * SPRING_DAMPING;   // d = 0.25
  float acceleration = force - dampingForce;
  
  velocity += acceleration;
  currentPos += velocity;
}
```

**Benefits**:
- Eliminates servo jitter
- Protects 3D-printed gears from shock loads
- Natural "fluid" motion feel
- Limits simultaneous servo movement to 2 for current management

#### BLE Command Protocol
```cpp
// Commands: b<angle>, s<angle>, e<angle>, w<angle>, g<angle>
// Example: "b90\n" → Base servo to 90°
//          "g45\n" → Gripper to 45°
```

#### Configuration
```cpp
// Servo pins
const int PIN_BASE = 13, PIN_SHOULDER = 12;
const int PIN_ELBOW = 14, PIN_WRIST = 27, PIN_GRIPPER = 26;

// Physics constants
const float SPRING_TENSION = 0.08;
const float SPRING_DAMPING = 0.25;
const float STOP_THRESHOLD = 0.5;

// BLE
#define DEVICE_NAME "ESP32_RoboArm_Polyizer"
```

### Base Controller - Mecanum Kinematics

#### Omnidirectional Movement Vectors
```cpp
// Forward: All wheels forward
void moveForward()  { FL+, FR+, RL+, RR+ }

// Backward: All wheels reverse
void moveBackward() { FL-, FR-, RL-, RR- }

// Rotate Left: Left wheels reverse, right forward
void rotateLeft()   { FL-, FR+, RL-, RR+ }

// Rotate Right: Left wheels forward, right reverse
void rotateRight()  { FL+, FR-, RL+, RR- }

// Strafe Left: Diagonal pairs opposite
void strafeLeft()   { FL-, FR+, RL+, RR- }

// Strafe Right: Opposite diagonal
void strafeRight()  { FL+, FR-, RL-, RR+ }
```

#### Motor Inversion Logic
```cpp
// Per-wheel inversion flags (tune during testing)
bool INVERT_FL = false;
bool INVERT_FR = false;
bool INVERT_RL = false;
bool INVERT_RR = false;

// XOR inversion in setMotor()
boolean finalState = reverse ^ invert;
```

#### Configuration
```cpp
// Motor pins
#define FL_ENA 32, FL_IN1 15, FL_IN2 25  // Front Left
#define FR_ENB 14, FR_IN3 26, FR_IN4 27  // Front Right
#define RL_ENA 13, RL_IN1 16, RL_IN2 17  // Rear Left
#define RR_ENB 23, RR_IN3 18, RR_IN4 19  // Rear Right

// PWM configuration
ledcAttach(pin, 30000, 8);  // 30kHz, 8-bit resolution

// BLE
#define DEVICE_NAME "ESP32_Mecanum_Web"
```

## 3. Arena Controller (Placeholder Code Structure)

### FSM States
```cpp
enum GameState {
  IDLE,
  GATE1_TRIGGERED,    // Activate pump → Dispense 125ml
  GATE2_TRIGGERED,    // LAM LED ON
  GATE3_TRIGGERED,    // Load cell check → LCD display
  GAME_COMPLETE
};
```

### Gate Logic
```cpp
// Gate 1: Pump Control
void onGate1Cross() {
  digitalWrite(PUMP_RELAY, HIGH);
  pumpStartTime = millis();
  // Stop after 125ml dispensed (calibrated timing or flow sensor)
}

// Gate 2: LED Activation
void onGate2Cross() {
  digitalWrite(LAM_LED_PIN, HIGH);
}

// Gate 3: Weight Verification
void onGate3Cross() {
  float weight = loadCell.get_units();
  if (abs(weight - TARGET_WEIGHT) < TOLERANCE) {
    displaySuccess();  // Team name/photo/video
  } else {
    displayError();    // "Weight Mismatch"
  }
}
```

### Hardware Pins (Example)
```cpp
const int GATE1_SENSOR = 34;
const int GATE2_SENSOR = 35;
const int GATE3_SENSOR = 36;
const int PUMP_RELAY = 25;
const int LAM_LED_PIN = 26;
const int LOADCELL_DOUT = 16;
const int LOADCELL_SCK = 17;
```

## Hardware Requirements

### ALFR Components
| Component | Spec | Qty |
|-----------|------|-----|
| Arduino Nano R3 | CH340 chip | 1 |
| L298N Motor Driver | 5V-35V, 2A | 1 |
| Line Sensor Array | Smart Elex RLS-08, 8-channel | 1 |
| Ultrasonic Sensor | HC-SR04 | 1 |
| DC Geared Motor | Orange Johnson, 12V, 600rpm | 2 |
| Robot Wheels | 65mm diameter | 2 |

### SARM Components
| Component | Spec | Qty |
|-----------|------|-----|
| ESP32 Dev Board | WROOM-32, 38-pin | 2 |
| Servo Motor | MG996R, 180°, 11kgcm | 2 |
| High Torque Servo | 180°, 20kgcm | 3 |
| Omni Wheels | 58mm | 4 |
| N20 Gear Motor | 600rpm | 2 |
| L298N Motor Driver | 5V-35V | 1 |

### Arena Components
| Component | Spec | Qty |
|-----------|------|-----|
| ESP32 Dev Board | WROOM-32 | 1 |
| Relay Module | 5V, 1-channel | 2 |
| Load Cell | 5kg with HX711 | 1 |
| TFT LCD | 1.8", 128×160 | 1 |
| Water Pump | DC 6-12V, R365 | 1 |

## Installation

### Arduino IDE Setup
```bash
# Add ESP32 board manager URL:
# https://dl.espressif.com/dl/package_esp32_index.json

# Install boards:
# - Arduino AVR Boards (for Nano)
# - esp32 by Espressif Systems

# Install libraries:
# - ESP32Servo (for SARM arm controller)
```

### Upload Procedure
1. **ALFR**: Select Arduino Nano → ATmega328P (Old Bootloader) → Upload
2. **SARM Arm**: Select ESP32 Dev Module → Upload → Connect via BLE
3. **SARM Base**: Select ESP32 Dev Module → Upload → Connect via BLE
4. **Arena**: Select ESP32 Dev Module → Configure pins → Upload

## Calibration

### ALFR Line Sensor
```
1. Power on → LED blinks for 5 seconds
2. Move robot over black line and white surface
3. Serial output shows min/max ranges
4. Wait for "GO!" message
```

### SARM Servo Limits
```cpp
// Test each servo individually
servo.setTarget(0);   // Minimum
delay(2000);
servo.setTarget(90);  // Center
delay(2000);
servo.setTarget(180); // Maximum
```

### Load Cell Zeroing
```cpp
loadCell.tare();  // Remove all weight, call tare()
// Calibration factor determined experimentally
```

## Tuning Guide

### PID Parameters (ALFR)
```cpp
// Increase Kp → Faster response, more oscillation
// Increase Ki → Eliminate steady drift, risk of windup
// Increase Kd → Dampen oscillation, sensitive to noise

// Start conservative, tune empirically:
Kp = 0.05;  // Gradual increase until oscillation appears
Kd = 1.0;   // Increase to dampen oscillation
Ki = 0.0001; // Keep minimal, only if steady-state error exists
```

### Physics Constants (SARM Arm)
```cpp
// Stiffer response (faster, jerkier)
SPRING_TENSION = 0.12;
SPRING_DAMPING = 0.20;

// Softer response (slower, smoother)
SPRING_TENSION = 0.05;
SPRING_DAMPING = 0.30;
```

### Mecanum Motor Speeds
```cpp
// Reduce if wheels slip
int motorSpeed = 200;  // Range: 0-255

// Adjust per-wheel if drifting during straight movement
```

## Communication Protocols

### BLE Command Format
```
Arm Controller:
  - 'b' + angle + '\n'  → Base rotation
  - 's' + angle + '\n'  → Shoulder
  - 'e' + angle + '\n'  → Elbow
  - 'w' + angle + '\n'  → Wrist
  - 'g' + angle + '\n'  → Gripper

Base Controller:
  - 'F' → Forward
  - 'B' → Backward
  - 'L' → Rotate Left
  - 'R' → Rotate Right
  - 'l' → Strafe Left
  - 'r' → Strafe Right
  - 'S' → Stop
```

## Performance Metrics

| Metric | Target | Achieved |
|--------|--------|----------|
| Line Following Accuracy | ±5mm | ±2mm (PID tuned) |
| Obstacle Detection Range | 10cm | 10cm reliable |
| Servo Response Time | <500ms | ~300ms (physics engine) |
| Pump Dispensing Accuracy | 125ml ±5ml | TBD (calibration required) |
| Total Game Time | <10 min | TBD |

## Troubleshooting

### ALFR Issues
```
Problem: Robot oscillates on curves
Solution: Increase Kd, decrease Kp

Problem: Robot loses line on intersections
Solution: Increase HISTORY_SIZE, tune lost-line threshold

Problem: Motors stall at low speeds
Solution: Increase minMotorSpeed (currently 110)
```

### SARM Issues
```
Problem: Servos jitter
Solution: Physics engine should prevent this; check power supply

Problem: Base drifts during forward movement
Solution: Tune INVERT_XX flags for each wheel

Problem: BLE disconnects frequently
Solution: Reduce distance, check antenna orientation
```

### Arena Issues
```
Problem: Pump over-dispenses
Solution: Calibrate timing or add flow sensor feedback

Problem: Load cell readings drift
Solution: Re-tare, check wiring, add thermal compensation
```

## Code Documentation

### ALFR Key Functions
```cpp
int readLinePosition()      // Weighted average from 8 sensors
void recoverLostLine()      // Exponential search algorithm
int getDistance()           // Ultrasonic ranging (non-blocking)
void drive(int L, int R)    // Motor control with inversion logic
```

### SARM Arm Key Functions
```cpp
SmoothServo::update()       // Physics simulation step
void runStartupSequence()   // Soft start prevents current spike
```

### SARM Base Key Functions
```cpp
void setMotor(...)          // PWM + direction with XOR inversion
void strafeLeft/Right()     // Mecanum diagonal movement
```

## Contributing

This code is optimized for the Lam Research Challenge specifications. Modifications should maintain:
- Real-time performance (no blocking delays in loops)
- Calibration robustness (sensor min/max tracking)
- Safety limits (servo angle constraints, motor stall prevention)

## Acknowledgments

- PID implementation based on embedded control theory
- Spring-damper physics adapted from robotics simulation principles
- Mecanum kinematics from omnidirectional platform research

---

**Technical Contact**: Team Polyizer  
**Challenge**: Lam Research Hardware Hustle Round 2  
**Year**: 2025
