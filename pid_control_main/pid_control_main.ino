#include <Servo.h>
#include <LiquidCrystal.h>

// --- Pins ---
const int trigPin = 9;
const int echoPin = 10;
const int servoPin = 6;

// LCD pins: RS, E, D4, D5, D6, D7
LiquidCrystal lcd(2, 3, 4, 5, 7, 8);

Servo myServo;

// --- PID Constants ---
float Kp = 2.0;
float Ki = 0.02;
float Kd = 0.1;

float targetDistance = 20.0; // cm

// --- PID Variables ---
float error = 0;
float previousError = 0;
float integral = 0;
float rawDerivative = 0;
float filteredDerivative = 0;
float previousFilteredDerivative = 0;

float alpha = 0.2; // Derivative filter coefficient
float distanceAlpha = 0.3; // Distance EMA coefficient
float smoothedDistance = -1.0;

unsigned long previousTime = 0;
unsigned long lcdPreviousTime = 0;
unsigned long lastValidReadingTime = 0;

// --- Servo Configuration ---
int centerAngle = 90;
int minAngle = 20;
int maxAngle = 160;

void setup() {
  Serial.begin(115200); // Faster baud rate to prevent serial logs from blocking the loop
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  myServo.attach(servoPin);
  myServo.write(centerAngle);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("PID Initializing");
  
  delay(1000);
  previousTime = millis();
  lcdPreviousTime = millis();
}

float getSingleDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // 15ms timeout (approx 250cm range). Reduces blocking time significantly.
  long duration = pulseIn(echoPin, HIGH, 15000); 
  
  if(duration > 0) {
    float dist = duration * 0.034 / 2;
    if(dist > 2.0 && dist < 250.0) { // Valid range filter
      return dist;
    }
  }
  return -1.0; // Return invalid
}

void handleSerialInput() {
  if (Serial.available() > 0) {
    char param = Serial.read();
    float value = Serial.parseFloat();
    
    // Clear buffer of newlines or spaces
    while (Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r' || Serial.peek() == ' ')) {
      Serial.read();
    }
    
    if (param == 'P' || param == 'p') {
      Kp = value;
      Serial.print("Kp updated to: "); Serial.println(Kp);
    } else if (param == 'I' || param == 'i') {
      Ki = value;
      integral = 0; // Reset integral to avoid sudden windup jumps
      Serial.print("Ki updated to: "); Serial.println(Ki);
    } else if (param == 'D' || param == 'd') {
      Kd = value;
      Serial.print("Kd updated to: "); Serial.println(Kd);
    } else if (param == 'T' || param == 't') {
      targetDistance = value;
      Serial.print("Target updated to: "); Serial.println(targetDistance);
    } else {
      Serial.println("Invalid command. Use P<val>, I<val>, D<val>, or T<val> (e.g. P2.5)");
    }
  }
}

void loop() {
  handleSerialInput();
  
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0; // dt in seconds
  
  if (dt <= 0) return; // Prevent division by zero
  
  float rawDistance = getSingleDistance();
  
  if (rawDistance > 0) { 
    lastValidReadingTime = currentTime;
    
    // Exponential Moving Average filter for distance
    if (smoothedDistance < 0) {
      smoothedDistance = rawDistance; // Initialize on first reading
    } else {
      smoothedDistance = (distanceAlpha * rawDistance) + ((1.0 - distanceAlpha) * smoothedDistance);
    }
    
    error = targetDistance - smoothedDistance;
    
    // Deadband: Ignore small errors to prevent servo jitter
    if (abs(error) < 1.0) {
      error = 0;
    }
    
    // Proportional
    float P = Kp * error;
    
    // Integral with anti-windup
    integral += error * dt;
    integral = constrain(integral, -50.0, 50.0);
    float I = Ki * integral;
    
    // Derivative with low-pass filter
    rawDerivative = (error - previousError) / dt;
    filteredDerivative = (alpha * rawDerivative) + ((1.0 - alpha) * previousFilteredDerivative);
    float D = Kd * filteredDerivative;
    
    // Total PID Output
    float pidOutput = P + I + D;
    
    // Servo Actuation
    int servoAngle = centerAngle + (int)pidOutput;
    servoAngle = constrain(servoAngle, minAngle, maxAngle);
    myServo.write(servoAngle);
    
    // Update variables for next loop
    previousError = error;
    previousFilteredDerivative = filteredDerivative;
    previousTime = currentTime;
    
    // --- Serial Logging ---
    Serial.print("Target:"); Serial.print(targetDistance);
    Serial.print("\tDistance:"); Serial.print(smoothedDistance);
    Serial.print("\tError:"); Serial.print(error);
    Serial.print("\tP:"); Serial.print(P);
    Serial.print("\tI:"); Serial.print(I);
    Serial.print("\tD:"); Serial.print(D);
    Serial.print("\tOutput:"); Serial.print(pidOutput);
    Serial.print("\tServo:"); Serial.println(servoAngle);
    
    // --- LCD Update (Non-blocking, updates every 250ms) ---
    if (currentTime - lcdPreviousTime >= 250) {
      lcdPreviousTime = currentTime;
      
      // Line 1: T:20.0 D:10.8
      lcd.setCursor(0, 0);
      lcd.print("T:");
      lcd.print(targetDistance, 1);
      lcd.print(" D:");
      lcd.print(smoothedDistance, 1);
      lcd.print("   "); // Clear trailing characters
      
      // Line 2: E:+9.2 S:109
      lcd.setCursor(0, 1);
      lcd.print("E:");
      if(error > 0) lcd.print("+");
      lcd.print(error, 1);
      lcd.print(" S:");
      lcd.print(servoAngle);
      lcd.print("   "); // Clear trailing characters
    }
  } else {
    // Failsafe Mode: If no valid reading for > 1000ms, return to center
    if (currentTime - lastValidReadingTime > 1000) {
      myServo.write(centerAngle);
      integral = 0; // Reset integral so it doesn't snap back when target returns
      smoothedDistance = -1.0; // Reset smoothing filter
    }
  }
}
