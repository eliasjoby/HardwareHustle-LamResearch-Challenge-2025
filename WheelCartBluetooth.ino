#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Unique UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ================= PIN DEFINITIONS =================
// Motor Pins - Front Left
#define FL_ENA 32
#define FL_IN1 15
#define FL_IN2 25

// Motor Pins - Front Right
#define FR_ENB 14
#define FR_IN3 26
#define FR_IN4 27

// Motor Pins - Rear Left
#define RL_ENA 13
#define RL_IN1 16
#define RL_IN2 17

// Motor Pins - Rear Right
#define RR_ENB 23
#define RR_IN3 18
#define RR_IN4 19

// ==================================================
//      --- MOTOR DIRECTION CONFIGURATION ---
// Change these to 'true' if a specific wheel spins 
// backwards when you press Forward.
// ==================================================

bool INVERT_FL = false;   // <--- Changed to TRUE for you
bool INVERT_FR = false;   // <--- Changed to TRUE for you
bool INVERT_RL = false;  // Back was correct
bool INVERT_RR = false;  // Back was correct

// ==================================================

int motorSpeed = 255; 

// Forward Declarations
void moveForward();
void moveBackward();
void rotateLeft();
void rotateRight();
void strafeLeft();
void strafeRight();
void stopRover();

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

// Bluetooth Callback
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue(); 
      if (value.length() > 0) {
        char cmd = value[0]; 
        switch(cmd) {
           case 'F': moveForward(); break;
           case 'B': moveBackward(); break;
           case 'L': rotateLeft(); break;
           case 'R': rotateRight(); break;
           case 'l': strafeLeft(); break;
           case 'r': strafeRight(); break;
           case 'S': stopRover(); break;
        }
      }
    }
};

void setup() {
  Serial.begin(115200);

  // -- Motor Setup --
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN3, OUTPUT); pinMode(FR_IN4, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_IN3, OUTPUT); pinMode(RR_IN4, OUTPUT);

  // ESP32 Board Manager 3.0+ PWM Setup
  ledcAttach(FL_ENA, 30000, 8);
  ledcAttach(FR_ENB, 30000, 8);
  ledcAttach(RL_ENA, 30000, 8);
  ledcAttach(RR_ENB, 30000, 8);

  // -- BLE Setup --
  BLEDevice::init("ESP32_Mecanum_Web"); 
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  
  // Fix for Android/Windows detection (Advertising setup)
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); 
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("Waiting for Web Bluetooth connection...");
}

void loop() {
  delay(10);
}

// ================= HELPER FUNCTION =================
// Now accepts an 'invert' flag to automatically flip logic
void setMotor(int pwmPin, int in1, int in2, int speed, boolean reverse, boolean invert) {
  ledcWrite(pwmPin, speed);
  
  // XOR Logic: If invert is true, it flips the 'reverse' boolean
  boolean finalState = reverse ^ invert; 
  
  if (!finalState) { 
    digitalWrite(in1, HIGH); 
    digitalWrite(in2, LOW); 
  } else { 
    digitalWrite(in1, LOW); 
    digitalWrite(in2, HIGH); 
  }
}

// ================= MOVEMENT LOGIC =================
// Note: We now pass the INVERT_XX variables to every call

void moveForward() {
  setMotor(FL_ENA, FL_IN1, FL_IN2, motorSpeed, false, INVERT_FL);
  setMotor(FR_ENB, FR_IN3, FR_IN4, motorSpeed, false, INVERT_FR);
  setMotor(RL_ENA, RL_IN1, RL_IN2, motorSpeed, false, INVERT_RL);
  setMotor(RR_ENB, RR_IN3, RR_IN4, motorSpeed, false, INVERT_RR);
}

void moveBackward() {
  setMotor(FL_ENA, FL_IN1, FL_IN2, motorSpeed, true, INVERT_FL);
  setMotor(FR_ENB, FR_IN3, FR_IN4, motorSpeed, true, INVERT_FR);
  setMotor(RL_ENA, RL_IN1, RL_IN2, motorSpeed, true, INVERT_RL);
  setMotor(RR_ENB, RR_IN3, RR_IN4, motorSpeed, true, INVERT_RR);
}

void rotateLeft() {
  setMotor(FL_ENA, FL_IN1, FL_IN2, motorSpeed, true, INVERT_FL);
  setMotor(FR_ENB, FR_IN3, FR_IN4, motorSpeed, false, INVERT_FR);
  setMotor(RL_ENA, RL_IN1, RL_IN2, motorSpeed, true, INVERT_RL);
  setMotor(RR_ENB, RR_IN3, RR_IN4, motorSpeed, false, INVERT_RR);
}

void rotateRight() {
  setMotor(FL_ENA, FL_IN1, FL_IN2, motorSpeed, false, INVERT_FL);
  setMotor(FR_ENB, FR_IN3, FR_IN4, motorSpeed, true, INVERT_FR);
  setMotor(RL_ENA, RL_IN1, RL_IN2, motorSpeed, false, INVERT_RL);
  setMotor(RR_ENB, RR_IN3, RR_IN4, motorSpeed, true, INVERT_RR);
}

void strafeLeft() {
  // FL Rev, FR Fwd, RL Fwd, RR Rev
  setMotor(FL_ENA, FL_IN1, FL_IN2, motorSpeed, true, INVERT_FL);
  setMotor(FR_ENB, FR_IN3, FR_IN4, motorSpeed, false, INVERT_FR);
  setMotor(RL_ENA, RL_IN1, RL_IN2, motorSpeed, false, INVERT_RL);
  setMotor(RR_ENB, RR_IN3, RR_IN4, motorSpeed, true, INVERT_RR);
}

void strafeRight() {
  // FL Fwd, FR Rev, RL Rev, RR Fwd
  setMotor(FL_ENA, FL_IN1, FL_IN2, motorSpeed, false, INVERT_FL);
  setMotor(FR_ENB, FR_IN3, FR_IN4, motorSpeed, true, INVERT_FR);
  setMotor(RL_ENA, RL_IN1, RL_IN2, motorSpeed, true, INVERT_RL);
  setMotor(RR_ENB, RR_IN3, RR_IN4, motorSpeed, false, INVERT_RR);
}

void stopRover() {
  ledcWrite(FL_ENA, 0); ledcWrite(FR_ENB, 0);
  ledcWrite(RL_ENA, 0); ledcWrite(RR_ENB, 0);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN3, LOW); digitalWrite(FR_IN4, LOW);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, LOW);
  digitalWrite(RR_IN3, LOW); digitalWrite(RR_IN4, LOW);
}