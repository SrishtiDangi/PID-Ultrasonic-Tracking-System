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

unsigned long previousTime = 0;
unsigned long lcdPreviousTime = 0;

// --- Servo Configuration ---
int centerAngle = 90;
int minAngle = 20;
int maxAngle = 160;

void setup() {
  Serial.begin(9600);
  
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

float getDistance() {
  // 5-sample averaging
  float sum = 0;
  int validReadings = 0;
  
  for(int i = 0; i < 5; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
    if(duration > 0) {
      float dist = duration * 0.034 / 2;
      if(dist > 2.0 && dist < 400.0) { // Valid range filter
        sum += dist;
        validReadings++;
      }
    }
    delay(5); // Small delay between pings
  }
  
  if (validReadings > 0) {
    return sum / validReadings;
  } else {
    return -1.0; // Return invalid
  }
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0; // dt in seconds
  
  if (dt <= 0) return; // Prevent division by zero
  
  float distance = getDistance();
  
  if (distance > 0) { // Only calculate PID if we got a valid reading
    error = targetDistance - distance;
    
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
    Serial.print("\tDistance:"); Serial.print(distance);
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
      lcd.print(distance, 1);
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
  }
}
