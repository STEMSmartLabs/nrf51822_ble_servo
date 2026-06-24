/*
  nRF51822 BLE Continuous Servo Controller (Software PWM)
  ---------------------------------------------------------
  This sketch sets up a Bluetooth Low Energy (BLE) server on the nRF51822
  using the BLEPeripheral library (by Sandeep Mistry). It emulates the
  Nordic UART Service (NUS) with swapped RX/TX characteristics compatible with
  the STEM Smart Labs BLE Remote (https://github.com/STEMSmartLabs/ble-remote)
  and LOFI Control protocols.
  
  Since the standard Servo library does not support the nRF51 processor, this
  sketch implements a lightweight, non-blocking software PWM routine directly 
  in the loop to drive the servos.
  
  Pins:
  - Servo 1: P0_0 (GP0 / Arduino Pin 0)
  - Servo 2: P0_1 (GP1 / Arduino Pin 1)
  
  Required Board Core:
  - arduino-nRF5 core by Sandeep Mistry (https://github.com/sandeepmistry/arduino-nRF5)
  - Must flash Nordic SoftDevice (usually S110 for nRF51822) to enable BLE stack.
  
  Required Libraries:
  - BLEPeripheral (Install via Sketch -> Include Library -> Manage Libraries...)
  
  PWA Link: https://stemsmartlabs.github.io/ble-remote/ (or local dev server)
*/

#include <SPI.h> // Required by BLEPeripheral library
#include <BLEPeripheral.h>

// BLE UUID definitions (Nordic UART Service - NUS)
#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e"

// NOTE: STEM Smart Labs / LOFI Swaps standard UART characteristics:
// - RX_UUID (Web app writes to this -> nRF51 receives from this) -> 0003
// - TX_UUID (Web app reads/notifies from this -> nRF51 sends to this) -> 0002
#define RX_CHARACTERISTIC_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define TX_CHARACTERISTIC_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

// Create BLE peripheral instance
BLEPeripheral blePeripheral = BLEPeripheral();

// Create BLE Service
BLEService uartService = BLEService(SERVICE_UUID);

// Create Characteristics
// RX characteristic (from central's perspective: central writes -> device receives)
BLECharacteristic rxCharacteristic = BLECharacteristic(RX_CHARACTERISTIC_UUID, BLEWrite | BLEWriteWithoutResponse, 20);

// TX characteristic (from central's perspective: central reads/notifies -> device transmits)
BLECharacteristic txCharacteristic = BLECharacteristic(TX_CHARACTERISTIC_UUID, BLENotify, 20);

// Define Servo Pins (P0_0 and P0_1 on the nRF51822 barebone pinout)
const int SERVO1_PIN = 18; // P0_0 (Left Motor)
const int SERVO2_PIN = 20; // P0_1 (Right Motor)

// Continuous rotation servos limits (microseconds)
const int SERVO_STOP = 1500;
const int SERVO_CW_MAX = 1000;  // Full speed clockwise
const int SERVO_CCW_MAX = 2000; // Full speed counter-clockwise

// Calibration Offsets (in microseconds) to fine-tune speeds
const int SERVO1_CALIBRATION_OFFSET = 0; // Adjust if one motor spins slightly faster
const int SERVO2_CALIBRATION_OFFSET = 0;

// Software PWM tracking variables
unsigned long lastPulseTime = 0;
volatile int servo1PulseUs = SERVO_STOP; 
volatile int servo2PulseUs = SERVO_STOP;

// Command parser variables
String rxBuffer = "";

// Forward declarations of movement control functions
void stopMotors();
void controlDpad(String cmd);
void controlJoystick(int x, int y);
void controlSliderS1(int val);
void controlSliderS2(int val);
void processIncomingCommand(String cmd);

// Forward declarations of BLE event handlers
void blePeripheralConnectHandler(BLECentral& central);
void blePeripheralDisconnectHandler(BLECentral& central);
void rxCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic);

void setup() {
  Serial.begin(115200);
  
  // Initialize Servo Pins as Outputs
  pinMode(SERVO1_PIN, OUTPUT);
  pinMode(SERVO2_PIN, OUTPUT);
  
  stopMotors(); // Initial safety stop

  // Set local name and device name
  // Must start with "micro:bit" to pass the PWA's search filters
  blePeripheral.setLocalName("micro:bit nRF51");
  blePeripheral.setDeviceName("micro:bit nRF51");
  
  // Set advertised service UUID
  blePeripheral.setAdvertisedServiceUuid(uartService.uuid());
  
  // Add service and characteristics attributes to peripheral
  blePeripheral.addAttribute(uartService);
  blePeripheral.addAttribute(rxCharacteristic);
  blePeripheral.addAttribute(txCharacteristic);
  
  // Set callback handlers
  rxCharacteristic.setEventHandler(BLEWritten, rxCharacteristicWritten);
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // Begin BLE advertising
  blePeripheral.begin();
  
  Serial.println("BLE UART Server is running! Ready for STEM Smart Labs pairing...");
}

void loop() {
  // Poll BLE peripheral to check for events and trigger callbacks
  blePeripheral.poll();

  // -------------------------------------------------------------
  // Software PWM Generation (50Hz / 20ms period)
  // -------------------------------------------------------------
  unsigned long now = micros();
  if (now - lastPulseTime >= 20000) { // 20ms elapsed
    lastPulseTime = now;
    
    // Read current volatile values to avoid race conditions
    int p1 = servo1PulseUs;
    int p2 = servo2PulseUs;
    
    // Check active status
    // Continuous rotation servos will stop completely without creep if we cut the pulse signals entirely!
    bool s1_active = (p1 != SERVO_STOP);
    bool s2_active = (p2 != SERVO_STOP);

    if (s1_active && s2_active) {
      // Apply calibration offsets when running
      p1 += SERVO1_CALIBRATION_OFFSET;
      p2 += SERVO2_CALIBRATION_OFFSET;
      
      digitalWrite(SERVO1_PIN, HIGH);
      digitalWrite(SERVO2_PIN, HIGH);
      if (p1 < p2) {
        delayMicroseconds(p1);
        digitalWrite(SERVO1_PIN, LOW);
        delayMicroseconds(p2 - p1);
        digitalWrite(SERVO2_PIN, LOW);
      } else {
        delayMicroseconds(p2);
        digitalWrite(SERVO2_PIN, LOW);
        delayMicroseconds(p1 - p2);
        digitalWrite(SERVO1_PIN, LOW);
      }
    } else if (s1_active) {
      p1 += SERVO1_CALIBRATION_OFFSET;
      digitalWrite(SERVO1_PIN, HIGH);
      delayMicroseconds(p1);
      digitalWrite(SERVO1_PIN, LOW);
    } else if (s2_active) {
      p2 += SERVO2_CALIBRATION_OFFSET;
      digitalWrite(SERVO2_PIN, HIGH);
      delayMicroseconds(p2);
      digitalWrite(SERVO2_PIN, LOW);
    } else {
      // Both stopped: pull low and send NO pulses to eliminate any neutral-creep!
      digitalWrite(SERVO1_PIN, LOW);
      digitalWrite(SERVO2_PIN, LOW);
    }
  }
}

// -------------------------------------------------------------
// BLE CALLBACK HANDLERS
// -------------------------------------------------------------

void blePeripheralConnectHandler(BLECentral& central) {
  Serial.print("BLE Client Connected: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  Serial.print("BLE Client Disconnected: ");
  Serial.println(central.address());
  stopMotors(); // Safety stop when client disconnects
}

void rxCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  const uint8_t* val = characteristic.value();
  int len = characteristic.valueLength();
  
  // Feed incoming characters to parser buffer
  for (int i = 0; i < len; i++) {
    char c = (char)val[i];
    if (c == '\n') {
      rxBuffer.trim();
      if (rxBuffer.length() > 0) {
        processIncomingCommand(rxBuffer);
      }
      rxBuffer = ""; // Reset buffer
    } else if (c != '\r') {
      rxBuffer += c;
    }
  }
}

// -------------------------------------------------------------
// COMMAND ROUTING
// -------------------------------------------------------------

void processIncomingCommand(String cmd) {
  Serial.print("Received BLE Command: ");
  Serial.println(cmd);

  // 1. D-Pad & Emulated Direction Commands
  if (cmd == "UP" || cmd == "DOWN" || cmd == "LEFT" || cmd == "RIGHT" ||
      cmd == "up" || cmd == "down" || cmd == "left" || cmd == "right" ||
      cmd == "STOP") {
    controlDpad(cmd);
  }
  // 2. Joystick & Tilt Mode Coordinates (Format: X+xx,Y+yy)
  else if (cmd.startsWith("X") && cmd.indexOf(",Y") != -1) {
    int yIndex = cmd.indexOf(",Y");
    String xStr = cmd.substring(1, yIndex);
    String yStr = cmd.substring(yIndex + 2);
    
    int xVal = xStr.toInt(); // Range: -90 to 90
    int yVal = yStr.toInt(); // Range: -90 to 90
    
    controlJoystick(xVal, yVal);
  }
  // 3. Slider 1 / Mixer Servo 1 command (Format: c123)
  else if (cmd.startsWith("c")) {
    int val = cmd.substring(1).toInt(); // Range: 0 to 180
    controlSliderS1(val);
  }
  // 4. Slider 2 / Mixer Servo 2 command (Format: x123)
  else if (cmd.startsWith("x")) {
    int val = cmd.substring(1).toInt(); // Range: 0 to 180
    controlSliderS2(val);
  }
}

// -------------------------------------------------------------
// MOTOR CONTROL IMPLEMENTATIONS
// -------------------------------------------------------------

void stopMotors() {
  servo1PulseUs = SERVO_STOP;
  servo2PulseUs = SERVO_STOP;
  Serial.println("Motors: STOPPED");
}

/*
  D-PAD Controller Logic
  Continuous rotation servos need opposite rotations to drive a robot straight
  because the motors are physically mirrored on the left and right sides.
*/
void controlDpad(String cmd) {
  if (cmd == "UP") {
    // Forward: Left CW (1000us), Right CCW (2000us)
    servo1PulseUs = SERVO_CW_MAX;
    servo2PulseUs = SERVO_CCW_MAX;
    Serial.println("Motors: FORWARD");
  } 
  else if (cmd == "DOWN") {
    // Backward: Left CCW (2000us), Right CW (1000us)
    servo1PulseUs = SERVO_CCW_MAX;
    servo2PulseUs = SERVO_CW_MAX;
    Serial.println("Motors: BACKWARD");
  } 
  else if (cmd == "LEFT") {
    // Spin Left: Left CCW (2000us), Right CCW (2000us)
    servo1PulseUs = SERVO_CCW_MAX;
    servo2PulseUs = SERVO_CCW_MAX;
    Serial.println("Motors: SPIN LEFT");
  } 
  else if (cmd == "RIGHT") {
    // Spin Right: Left CW (1000us), Right CW (1000us)
    servo1PulseUs = SERVO_CW_MAX;
    servo2PulseUs = SERVO_CW_MAX;
    Serial.println("Motors: SPIN RIGHT");
  } 
  else if (cmd == "STOP" || cmd == "up" || cmd == "down" || cmd == "left" || cmd == "right") {
    stopMotors();
  }
}

/*
  Joystick / Tilt Mixing Logic
  Transforms Cartesian coordinates (x, y) into differential drive speeds.
  x: steering (-90 left, 90 right)
  y: speed (-90 backward, 90 forward)
*/
void controlJoystick(int x, int y) {
  // 1. Calculate relative speed for each side using differential mixing
  int leftSpeed = y + x;
  int rightSpeed = y - x;

  // 2. Clamp speeds between -90 and 90
  leftSpeed = constrain(leftSpeed, -90, 90);
  rightSpeed = constrain(rightSpeed, -90, 90);

  // 3. Map speed to servo microseconds
  // Left motor (pin 0) forward is CW (1000us), backward is CCW (2000us)
  int leftUs = SERVO_STOP - (leftSpeed * 500 / 90);
  
  // Right motor (pin 1) forward is CCW (2000us), backward is CW (1000us)
  int rightUs = SERVO_STOP + (rightSpeed * 500 / 90);

  servo1PulseUs = leftUs;
  servo2PulseUs = rightUs;

  Serial.print("Joystick output: Left=");
  Serial.print(leftUs);
  Serial.print("us, Right=");
  Serial.print(rightUs);
  Serial.println("us");
}

/*
  Slider Control (Mixer Mode)
  Maps standard servo angles (0-180) to continuous rotation speed/direction.
  cVal controls Servo 1.
*/
void controlSliderS1(int val) {
  // Map angle (0-180) to (1000-2000us)
  int us = map(val, 0, 180, SERVO_CW_MAX, SERVO_CCW_MAX);
  servo1PulseUs = us;
  
  Serial.print("Slider 1 (Servo 1): Value=");
  Serial.print(val);
  Serial.print(" -> ");
  Serial.print(us);
  Serial.println("us");
}

/*
  Slider Control (Mixer Mode)
  xVal controls Servo 2.
*/
void controlSliderS2(int val) {
  // Map angle (0-180) to (1000-2000us)
  int us = map(val, 0, 180, SERVO_CW_MAX, SERVO_CCW_MAX);
  servo2PulseUs = us;

  Serial.print("Slider 2 (Servo 2): Value=");
  Serial.print(val);
  Serial.print(" -> ");
  Serial.print(us);
  Serial.println("us");
}
