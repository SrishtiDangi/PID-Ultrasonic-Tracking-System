#include <LiquidCrystal.h>

// RS, E, D4, D5, D6, D7
LiquidCrystal lcd(2, 3, 4, 5, 7, 8);

void setup() {
  // Initialize the 16x2 LCD
  lcd.begin(16, 2);

  // Clear the display
  lcd.clear();

  // Print the test message
  lcd.setCursor(0, 0);
  lcd.print("PID SYSTEM");

  lcd.setCursor(0, 1);
  lcd.print("LCD WORKING!");
}

void loop() {
  // Nothing needed in the loop for this test
}
