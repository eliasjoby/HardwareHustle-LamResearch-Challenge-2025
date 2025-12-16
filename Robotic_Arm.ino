#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>

#define DEVICE_NAME            "ESP32_RoboArm_Polyizer" 

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Pins
const int PIN_BASE     = 13;
const int PIN_SHOULDER = 12;
const int PIN_ELBOW    = 14;
const int PIN_WRIST    = 27;
const int PIN_GRIPPER  = 26;

// Physics Constants for "Fluid" feel
const float SPRING_TENSION = 0.08; 
const float SPRING_DAMPING = 0.25; 
const float STOP_THRESHOLD = 0.5;

class SmoothServo {
  private:
    Servo servo;
    int pin;
    float currentPos;    
    float velocity;     
    float targetPos;
    bool _isMoving;

  public:
    String name;

    void init(int p, String n) {
      pin = p;
      name = n;
      // Start logically at 90 (Home)
      currentPos = 90.0;
      targetPos = 90.0;
      velocity = 0.0;
      _isMoving = false;
      
      servo.setPeriodHertz(50);
      servo.attach(pin, 500, 2400);
      servo.write(90); 
    }

    void setTarget(int angle) {
      targetPos = constrain(angle, 0, 180);
    }

    
    bool update() {
      // 1. Calculate physics (Spring-Damper system)
      // Force = (Distance * k) - (Velocity * d)
      float displacement = targetPos - currentPos;
      float force = displacement * SPRING_TENSION;
      float dampingForce = velocity * SPRING_DAMPING;
      float acceleration = force - dampingForce;

      // 2. Apply Physics
      velocity += acceleration;
      currentPos += velocity;

      // 3. Check if settled
      if (abs(displacement) < STOP_THRESHOLD && abs(velocity) < 0.1) {
        currentPos = targetPos;
        velocity = 0;
        _isMoving = false;
      } else {
        _isMoving = true;
      }

      
      servo.write((int)currentPos);
      
      return _isMoving;
    }

    bool isMoving() {
      return abs(targetPos - currentPos) > STOP_THRESHOLD;
    }
    
    // Force set without physics (good for startup)
    void hardSet(int angle) {
      currentPos = angle;
      targetPos = angle;
      velocity = 0;
      servo.write(angle);
    }
    
    bool atAngle(int angle) {
       return abs(currentPos - angle) < 2;
    }
};

// --- Globals ---

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
String receivedData = "";

SmoothServo base, shoulder, elbow, wrist, gripper;
SmoothServo* allServos[] = {&base, &shoulder, &elbow, &wrist, &gripper};

// --- Startup Routine ---
void runStartupSequence() {
  Serial.println("Running Soft Start...");
  
  // 1. Force all logic variables to 0 first (assuming robot starts folded or in random state)
  // Actually, safe start is usually to set them to 90 immediately physically,
  // but to prevent current spike, we attach and move one by one.
  
  for(int i=0; i<5; i++) {
    allServos[i]->hardSet(85);
    delay(200);
    allServos[i]->hardSet(90);
    delay(300);
  }
  Serial.println("Startup Complete. Robot at Home (90).");
}


// --- BLE Callbacks ---

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        for (int i = 0; i < rxValue.length(); i++) {
           char c = rxValue[i];
           if (c == '\n' || c == '\r') {
             if (receivedData.length() > 0) {
                char id = receivedData.charAt(0);
                int val = receivedData.substring(1).toInt();
                
                
                switch (id) {
                  case 'b': base.setTarget(val); break;
                  case 's': shoulder.setTarget(val); break;
                  case 'e': elbow.setTarget(val); break;
                  case 'w': wrist.setTarget(val); break;
                  case 'g': gripper.setTarget(val); break;
                }
                receivedData = "";
             }
           } else {
             receivedData += c;
           }
        }
      }
    }
};

// --- Standard Setup ---

void setup() {
  Serial.begin(115200);


  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  
  base.init(PIN_BASE, "Base");
  shoulder.init(PIN_SHOULDER, "Shoulder");
  elbow.init(PIN_ELBOW, "Elbow");
  wrist.init(PIN_WRIST, "Wrist");
  gripper.init(PIN_GRIPPER, "Gripper");

  runStartupSequence();


  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); 
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.print("Ready! Advertising as: ");
  Serial.println(DEVICE_NAME);
}

// --- Main Loop ---

void loop() {
  
  if (!deviceConnected && oldDeviceConnected) {
      delay(500);
      pServer->startAdvertising();
      oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
  }

  
  int activeCount = 0;
  
  for (int i = 0; i < 5; i++) {
    if (allServos[i]->isMoving()) {
      activeCount++;
    }
  }

  int movingNow = 0;
  for (int i = 0; i < 5; i++) {

    if (allServos[i]->isMoving()) {

      if (movingNow < 2) {
        allServos[i]->update();
        movingNow++;
      } 

    }
  }

  delay(15); 
}