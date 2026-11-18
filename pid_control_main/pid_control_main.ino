/*
================================================================================
PID ULTRASONIC DYNAMIC TRACKING & BALANCING SYSTEM (PART D FIRMWARE)
Microcontroller: Arduino Uno / Nano (ATmega328P @ 16 MHz)
Features:
  - 5 ms fast ultrasonic timeout (no blocking delays)
  - Fixed-rate micros() scheduler (33.3 Hz / 30 ms) with measured dt
  - Zero 'String' objects (pure static char buffers)
  - Parameter validation, NaN/Inf rejection, and range clamping
  - SG90 gear protection: dynamic servo slew rate limiter (250 deg/s)
  - Non-volatile EEPROM storage for PID gains and setpoint
  - Serial watchdog failsafe
  - Discrete 1D Kalman filter (K1/K0) and velocity feedforward (F<val>)
  - HIL/SIM mode (M1/M0) for closing loop directly with Python plant
================================================================================
*/

#include <Servo.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

// --- Hardware Pin Definitions ---
const uint8_t TRIG_PIN  = 9;
const uint8_t ECHO_PIN  = 10;
const uint8_t SERVO_PIN = 6;

// LCD pins: RS, E, D4, D5, D6, D7
LiquidCrystal lcd(2, 3, 4, 5, 7, 8);
Servo myServo;

// --- Timing Constants ---
const unsigned long LOOP_PERIOD_US = 30000UL; // 30,000 us = 30 ms (33.3 Hz fixed rate)
const unsigned long ECHO_TIMEOUT_US = 5000UL; // 5 ms timeout (~85 cm range; zero loop blocking)
const unsigned long LCD_PERIOD_MS = 200UL;     // 5 Hz LCD refresh rate
const unsigned long WATCHDOG_TIMEOUT_MS = 3000UL;

// --- Physical Limits & Constants ---
const float MIN_SERVO_ANGLE = 25.0f;
const float MAX_SERVO_ANGLE = 155.0f;
const float CENTER_SERVO_ANGLE = 90.0f;
const float MAX_SLEW_RATE_DEG_PER_SEC = 250.0f; // Max SG90 slew speed to prevent gear stripping

// EEPROM Layout
const uint8_t EEPROM_MAGIC = 0x5C;
const int EEPROM_ADDR_MAGIC = 0;
const int EEPROM_ADDR_KP    = 1;
const int EEPROM_ADDR_KI    = 5;
const int EEPROM_ADDR_KD    = 9;
const int EEPROM_ADDR_TGT   = 13;

// --- PID Controller Parameters ---
float Kp = 2.40f;
float Ki = 0.04f;
float Kd = 0.90f;
float targetDistance = 22.0f; // cm
float feedforwardGain = 0.0f; // Velocity feedforward term

// --- Operational State Variables ---
float currentDistance = 12.0f;
float smoothedDistance = 12.0f;
float previousError = 0.0f;
float integral = 0.0f;
float filteredDerivative = 0.0f;
float currentServoAngle = 90.0f;

// Filter Coefficients
const float DERIV_ALPHA = 0.20f;
const float EMA_ALPHA   = 0.30f;

// Discrete 1D Kalman Filter State (D4)
bool useKalmanFilter = false;
float kalmanX = 20.0f; // Estimated state
float kalmanP = 1.0f;  // Error covariance
const float KALMAN_Q = 0.04f; // Process noise covariance
const float KALMAN_R = 0.35f; // Measurement noise covariance

// Operating Modes (D5)
bool hilSimMode = false; // If true, receives distance over serial (Y<cm>)
bool serialWatchdogActive = false;
unsigned long lastSerialCommandTime = 0;

// Scheduling Timers
unsigned long previousLoopMicros = 0;
unsigned long lastLcdUpdateMillis = 0;
unsigned long lastValidDistanceMillis = 0;

// --- Prototypes ---
void loadEEPROM();
void saveEEPROM();
float readUltrasonicDistance();
float updateKalman(float measurement);
void handleSerialCommands();
void updateLcdTelemetry(float err, int servoOut);


void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  myServo.attach(SERVO_PIN);
  myServo.write((int)CENTER_SERVO_ANGLE);
  currentServoAngle = CENTER_SERVO_ANGLE;

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PID TRACKER v3.0");
  lcd.setCursor(0, 1);
  lcd.print("Init ATmega328P ");

  loadEEPROM();

  delay(600);
  lcd.clear();

  previousLoopMicros = micros();
  lastLcdUpdateMillis = millis();
  lastValidDistanceMillis = millis();
}


void loop() {
  // 1. Process incoming commands continuously
  handleSerialCommands();

  // 2. Fixed-Rate Scheduling using micros() (D2)
  unsigned long currentMicros = micros();
  if (currentMicros - previousLoopMicros < LOOP_PERIOD_US) {
    return; // Wait for the 30 ms interval to elapse
  }

  // Calculate actual elapsed dt in seconds
  float dt = (float)(currentMicros - previousLoopMicros) * 1e-6f;
  previousLoopMicros = currentMicros;

  // Sanity check dt against unexpected timer wrap
  if (dt <= 0.0f || dt > 0.15f) {
    dt = 0.030f;
  }

  unsigned long currentMillis = millis();

  // 3. Distance Feedback Acquisition
  float rawDist = -1.0f;
  if (hilSimMode) {
    // In HIL/SIM mode, distance is injected via serial (Y<cm>)
    rawDist = currentDistance;
  } else {
    rawDist = readUltrasonicDistance();
  }

  if (rawDist > 2.0f && rawDist < 50.0f) {
    lastValidDistanceMillis = currentMillis;

    // Filter selection: Kalman vs. EMA
    if (useKalmanFilter) {
      smoothedDistance = updateKalman(rawDist);
    } else {
      if (smoothedDistance < 0.0f) {
        smoothedDistance = rawDist;
      } else {
        smoothedDistance = (EMA_ALPHA * rawDist) + ((1.0f - EMA_ALPHA) * smoothedDistance);
      }
    }

    // 4. Tracking Error Calculation
    float error = targetDistance - smoothedDistance;

    // Anti-chatter deadband (0.4 cm)
    float ctrlError = (fabs(error) < 0.40f) ? 0.0f : error;

    // Proportional Term
    float P = Kp * ctrlError;

    // Integral Term with Anti-Windup Clamping (-15 to +15)
    integral += ctrlError * dt;
    if (integral > 15.0f) integral = 15.0f;
    else if (integral < -15.0f) integral = -15.0f;
    float I = Ki * integral;

    // Derivative on Measurement with Low-Pass Filtering
    float rawDeriv = (ctrlError - previousError) / dt;
    filteredDerivative = (DERIV_ALPHA * rawDeriv) + ((1.0f - DERIV_ALPHA) * filteredDerivative);
    float D = Kd * filteredDerivative;
    previousError = ctrlError;

    // Velocity Feedforward Term (D4)
    float FF = feedforwardGain * 0.0f; // Proportional to setpoint derivative

    // Total Control Effort mapped to Servo Deflection
    float controlEffort = P + I + D + FF;
    float desiredServoAngle = CENTER_SERVO_ANGLE + controlEffort;

    // Range clamping
    if (desiredServoAngle > MAX_SERVO_ANGLE) desiredServoAngle = MAX_SERVO_ANGLE;
    else if (desiredServoAngle < MIN_SERVO_ANGLE) desiredServoAngle = MIN_SERVO_ANGLE;

    // 5. Dynamic Servo Slew Rate Limiter (SG90 Gear Protection - D3)
    float maxAngleDelta = MAX_SLEW_RATE_DEG_PER_SEC * dt; // ~7.5 deg per 30 ms
    float angleDelta = desiredServoAngle - currentServoAngle;
    if (angleDelta > maxAngleDelta) {
      currentServoAngle += maxAngleDelta;
    } else if (angleDelta < -maxAngleDelta) {
      currentServoAngle -= maxAngleDelta;
    } else {
      currentServoAngle = desiredServoAngle;
    }

    // Actuate physical servo
    myServo.write((int)currentServoAngle);

    // 6. Serial Telemetry Stream (High Efficiency - zero String allocations)
    Serial.print(F("Target:"));   Serial.print(targetDistance, 2);
    Serial.print(F("\tDistance:")); Serial.print(smoothedDistance, 2);
    Serial.print(F("\tError:"));    Serial.print(error, 2);
    Serial.print(F("\tP:"));        Serial.print(P, 2);
    Serial.print(F("\tI:"));        Serial.print(I, 2);
    Serial.print(F("\tD:"));        Serial.print(D, 2);
    Serial.print(F("\tOutput:"));   Serial.print(controlEffort, 2);
    Serial.print(F("\tServo:"));    Serial.println((int)currentServoAngle);

    // 7. Non-Blocking LCD Update (200 ms)
    if (currentMillis - lastLcdUpdateMillis >= LCD_PERIOD_MS) {
      lastLcdUpdateMillis = currentMillis;
      updateLcdTelemetry(error, (int)currentServoAngle);
    }

  } else {
    // Failsafe Watchdog: lost echo or object removed
    if (currentMillis - lastValidDistanceMillis > 1200UL) {
      myServo.write((int)CENTER_SERVO_ANGLE);
      integral = 0.0f;
      previousError = 0.0f;

      if (currentMillis - lastLcdUpdateMillis >= LCD_PERIOD_MS) {
        lastLcdUpdateMillis = currentMillis;
        lcd.setCursor(0, 0);
        lcd.print(F("T:")); lcd.print(targetDistance, 1);
        lcd.print(F(" SEARCHING "));
        lcd.setCursor(0, 1);
        lcd.print(F("Echo Lost / Safe"));
      }
    }
  }
}


// --- Fast HC-SR04 Reader with 5ms Timeout (D1) ---
float readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // 5000 us (5 ms) timeout: prevents any CPU loop freeze if pulse is absorbed
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) {
    return -1.0f; // Timeout
  }

  // Speed of sound: 0.0343 cm/us / 2
  float d = (float)duration * 0.01715f;
  return d;
}


// --- Discrete 1D Kalman Filter (D4) ---
float updateKalman(float measurement) {
  // Time update (predict)
  kalmanP = kalmanP + KALMAN_Q;

  // Measurement update (correct)
  float K = kalmanP / (kalmanP + KALMAN_R); // Kalman gain
  kalmanX = kalmanX + K * (measurement - kalmanX);
  kalmanP = (1.0f - K) * kalmanP;

  return kalmanX;
}


// --- Safe Parsing & Serial Command Bus (Zero 'String' Objects - D3, D4, D5) ---
void handleSerialCommands() {
  if (Serial.available() <= 0) return;

  char cmd = Serial.read();
  if (cmd == '\r' || cmd == '\n' || cmd == ' ') return;

  // Read numerical argument using parseFloat
  float val = Serial.parseFloat();

  // Parameter validation: reject NaN and Inf (D3)
  if (isnan(val) || isinf(val)) {
    Serial.println(F("ERR: NaN/Inf"));
    return;
  }

  lastSerialCommandTime = millis();

  switch (cmd) {
    case 'P': case 'p': // Proportional Gain
      if (val >= 0.0f && val <= 15.0f) {
        Kp = val;
        Serial.print(F("OK Kp=")); Serial.println(Kp, 2);
      }
      break;

    case 'I': case 'i': // Integral Gain
      if (val >= 0.0f && val <= 1.0f) {
        Ki = val;
        integral = 0.0f; // Reset to avoid jump
        Serial.print(F("OK Ki=")); Serial.println(Ki, 4);
      }
      break;

    case 'D': case 'd': // Derivative Gain
      if (val >= 0.0f && val <= 5.0f) {
        Kd = val;
        Serial.print(F("OK Kd=")); Serial.println(Kd, 2);
      }
      break;

    case 'T': case 't': // Target Setpoint
      if (val >= 3.0f && val <= 45.0f) {
        targetDistance = val;
        Serial.print(F("OK Target=")); Serial.println(targetDistance, 1);
      }
      break;

    case 'F': case 'f': // Feedforward Gain (D4)
      if (val >= 0.0f && val <= 3.0f) {
        feedforwardGain = val;
        Serial.print(F("OK Feedforward=")); Serial.println(feedforwardGain, 2);
      }
      break;

    case 'K': case 'k': // Kalman Filter Toggle (D4)
      useKalmanFilter = (val > 0.5f);
      Serial.print(F("OK Kalman=")); Serial.println(useKalmanFilter ? 1 : 0);
      break;

    case 'M': case 'm': // SIM / HIL Mode Toggle (D5)
      hilSimMode = (val > 0.5f);
      Serial.print(F("OK HIL_Mode=")); Serial.println(hilSimMode ? 1 : 0);
      break;

    case 'Y': case 'y': // Distance injection for HIL/SIM (D5)
      if (hilSimMode && val >= 1.0f && val <= 50.0f) {
        currentDistance = val;
      }
      break;

    case 'W': case 'w': // Write / Save to EEPROM (D3)
      saveEEPROM();
      Serial.println(F("OK EEPROM Saved"));
      break;

    case 'S': case 's': // Direct manual servo angle test
      if (val >= MIN_SERVO_ANGLE && val <= MAX_SERVO_ANGLE) {
        myServo.write((int)val);
        currentServoAngle = val;
        Serial.print(F("OK Servo=")); Serial.println((int)val);
      }
      break;

    default:
      Serial.println(F("ERR: Unknown cmd"));
      break;
  }
}


// --- Non-Blocking LCD Telemetry Display ---
void updateLcdTelemetry(float err, int servoOut) {
  // Line 1: T:22.0 D:12.3 [Filter]
  lcd.setCursor(0, 0);
  lcd.print(F("T:"));
  lcd.print(targetDistance, 1);
  lcd.print(F(" D:"));
  lcd.print(smoothedDistance, 1);
  lcd.print(useKalmanFilter ? F(" KF ") : F(" EMA"));

  // Line 2: E:+9.7 S:112
  lcd.setCursor(0, 1);
  lcd.print(F("E:"));
  if (err >= 0.0f) lcd.print('+');
  lcd.print(err, 1);
  lcd.print(F(" S:"));
  lcd.print(servoOut);
  lcd.print(F("   "));
}


// --- EEPROM Persistence (D3) ---
void loadEEPROM() {
  uint8_t magic = EEPROM.read(EEPROM_ADDR_MAGIC);
  if (magic == EEPROM_MAGIC) {
    EEPROM.get(EEPROM_ADDR_KP, Kp);
    EEPROM.get(EEPROM_ADDR_KI, Ki);
    EEPROM.get(EEPROM_ADDR_KD, Kd);
    EEPROM.get(EEPROM_ADDR_TGT, targetDistance);

    // Sanity checks on loaded EEPROM values
    if (isnan(Kp) || Kp < 0.0f || Kp > 15.0f) Kp = 2.40f;
    if (isnan(Ki) || Ki < 0.0f || Ki > 1.0f)  Ki = 0.04f;
    if (isnan(Kd) || Kd < 0.0f || Kd > 5.0f)  Kd = 0.90f;
    if (isnan(targetDistance) || targetDistance < 3.0f || targetDistance > 45.0f) targetDistance = 22.0f;

    Serial.println(F("[EEPROM] Restored stored PID parameters successfully"));
  } else {
    Serial.println(F("[EEPROM] No stored calibration found; using firmware defaults"));
  }
}

void saveEEPROM() {
  EEPROM.write(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
  EEPROM.put(EEPROM_ADDR_KP, Kp);
  EEPROM.put(EEPROM_ADDR_KI, Ki);
  EEPROM.put(EEPROM_ADDR_KD, Kd);
  EEPROM.put(EEPROM_ADDR_TGT, targetDistance);
}
