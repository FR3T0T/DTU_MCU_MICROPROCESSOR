#include <Arduino.h>
#include <stdint.h>
#include "i2c.h"
#include "ssd1306.h"

#define NUM_OPERATIONS 4

// Pin Definitions
const int switchPin = P2_1;
const int operationSwitchPin = P1_1;
const int dipPin0 = P6_0;
const int dipPin1 = P6_1;
const int dipPin2 = P6_2;
const int dipPin3 = P6_3;
const int dipPin4 = P6_4;
const int dipPin5 = P6_5;
const int dipPin6 = P6_6;
const int dipPin7 = P7_0;

// Global Variables
unsigned char variable1 = 0, variable2 = 0;
int operationCount = 0;
char operations[] = {'+', '-', '*', '/'};
char expressionStr[30]; // Holds the expression being built
char resultStr[30];     // Holds the final result or error message

bool waitingForNextCalculation = false; // Flag to lock the result screen

// Function to read the DIP switches
unsigned char readDipswitch() {
  unsigned char value = 0;
  value |= (digitalRead(dipPin0) == LOW ? 1 : 0) << 0;
  value |= (digitalRead(dipPin1) == LOW ? 1 : 0) << 1;
  value |= (digitalRead(dipPin2) == LOW ? 1 : 0) << 2;
  value |= (digitalRead(dipPin3) == LOW ? 1 : 0) << 3;
  value |= (digitalRead(dipPin4) == LOW ? 1 : 0) << 4;
  value |= (digitalRead(dipPin5) == LOW ? 1 : 0) << 5;
  value |= (digitalRead(dipPin6) == LOW ? 1 : 0) << 6;
  value |= (digitalRead(dipPin7) == LOW ? 1 : 0) << 7;
  return value;
}

// Function to update the display without clearing it entirely
void updateDisplay(const char* line1 = NULL, const char* line2 = NULL, const char* line3 = NULL) {
  if (line1 != NULL) {
    // Clear line 1 by printing spaces
    ssd1306_printText(0, 0, "                    "); // Adjust the number of spaces as needed
    // Print new content
    ssd1306_printText(0, 0, line1);
  }
  if (line2 != NULL) {
    // Clear line 2
    ssd1306_printText(0, 1, "                    ");
    // Print new content
    ssd1306_printText(0, 1, line2);
  }
  if (line3 != NULL) {
    // Clear line 3
    ssd1306_printText(0, 2, "                    ");
    // Print new content
    ssd1306_printText(0, 2, line3);
  }
}

// Function to check if the operation switch is pressed
void checkOperationSwitch() {
  if (!waitingForNextCalculation && digitalRead(operationSwitchPin) == LOW) {
    operationCount = (operationCount + 1) % NUM_OPERATIONS;
    while (digitalRead(operationSwitchPin) == LOW) {
      delay(50); // Debounce
    }
    // Update only the second line with the new operation
    sprintf(expressionStr, "%d %c", variable1, operations[operationCount]);
    updateDisplay(NULL, expressionStr);
    delay(500); // Brief delay
  }
}

void setup() {
  // Initialize pins
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(operationSwitchPin, INPUT_PULLUP);
  pinMode(dipPin0, INPUT_PULLUP);
  pinMode(dipPin1, INPUT_PULLUP);
  pinMode(dipPin2, INPUT_PULLUP);
  pinMode(dipPin3, INPUT_PULLUP);
  pinMode(dipPin4, INPUT_PULLUP);
  pinMode(dipPin5, INPUT_PULLUP);
  pinMode(dipPin6, INPUT_PULLUP);
  pinMode(dipPin7, INPUT_PULLUP);

  // Initialize I2C and OLED display
  i2c_init();
  ssd1306_init();
  ssd1306_clearDisplay();
  ssd1306_printText(0, 0, "Ready");
}

void loop() {
  int result = 0;
  bool error = false;
  waitingForNextCalculation = false; // Reset the flag at the beginning of the loop

  // ======== Step 1: Input First Variable ========
  updateDisplay("Set First Number");
  unsigned char prevVariable1 = 255; // Initialize to a value that won't match the first read
  while (digitalRead(switchPin) == HIGH) {
    checkOperationSwitch(); // Check for operation change
    variable1 = readDipswitch();
    if (variable1 != prevVariable1) {
      prevVariable1 = variable1;
      sprintf(expressionStr, "%d", variable1);
      updateDisplay(NULL, expressionStr); // Update only the second line
    }
    delay(100); // Reduce update frequency to every 100ms
  }
  while (digitalRead(switchPin) == LOW) {
    delay(50); // Debounce
  }

  // ======== Step 2: Select Operation ========
  sprintf(expressionStr, "%d %c", variable1, operations[operationCount]);
  updateDisplay("Select Operation", expressionStr);
  while (digitalRead(switchPin) == HIGH) {
    checkOperationSwitch(); // Allow changing operation
    delay(100); // Wait
  }
  while (digitalRead(switchPin) == LOW) {
    delay(50); // Debounce
  }

  // ======== Step 3: Input Second Variable ========
  updateDisplay("Set Second Number");
  unsigned char prevVariable2 = 255;
  while (digitalRead(switchPin) == HIGH) {
    checkOperationSwitch(); // Check for operation change
    variable2 = readDipswitch();
    if (variable2 != prevVariable2) {
      prevVariable2 = variable2;
      sprintf(expressionStr, "%d %c %d", variable1, operations[operationCount], variable2);
      updateDisplay(NULL, expressionStr); // Update only the second line
    }
    delay(100); // Reduce update frequency to every 100ms
  }
  while (digitalRead(switchPin) == LOW) {
    delay(50); // Debounce
  }

  // ======== Step 4: Display Complete Expression ========
  sprintf(expressionStr, "%d %c %d", variable1, operations[operationCount], variable2);
  updateDisplay("Calculating:", expressionStr);
  delay(1000); // Brief pause before calculation

  // ======== Step 5: Perform Calculation ========
  switch (operations[operationCount]) {
    case '+':
      result = variable1 + variable2;
      break;
    case '-':
      result = variable1 - variable2;
      break;
    case '*':
      result = variable1 * variable2;
      break;
    case '/':
      if (variable2 != 0) {
        result = variable1 / variable2;
      } else {
        sprintf(resultStr, "Error: Div by 0");
        error = true;
      }
      break;
  }

  // ======== Step 6: Display Result ========
  if (!error) {
    sprintf(resultStr, "%d %c %d = %d", variable1, operations[operationCount], variable2, result);
    updateDisplay("Result:", resultStr, "P1.1 to restart");
  } else {
    updateDisplay("Error:", resultStr, "P1.1 to restart");
  }

  // ======== Step 7: Wait for OperationSwitchPin Press to Proceed ========
  waitingForNextCalculation = true; // Set the flag to lock operation changes
  while (digitalRead(operationSwitchPin) == HIGH) {
    delay(50); // Debounce delay
  }
  while (digitalRead(operationSwitchPin) == LOW) {
    delay(50); // Debounce delay
  }
  waitingForNextCalculation = false; // Unlock operation changes

  // Clear display before starting the next calculation
  ssd1306_clearDisplay();
}









/*
#include <Arduino.h>
#include "i2c.h"
#include "ssd1306.h"

// put function declarations here:
int myFunction(int, int);

// Test tryk på MSP-enheden.
const int switchPin = P2_1; 
const int ledPin = P1_0;

// Dipswitch tryk til P_X.
const int dipPin0 = P6_0;
const int dipPin1 = P6_1;
const int dipPin2 = P6_2;
const int dipPin3 = P6_3;
const int dipPin4 = P6_4;
const int dipPin5 = P6_5;
const int dipPin6 = P6_6;
const int dipPin7 = P7_0;

// Funktion til at konvertere et tal til hexadecimal streng
void decimalToHex(unsigned char decimal, char* hex)
{
    sprintf(hex, "%02X", decimal);
}

void setup()
{
// Test tryk på MSP-enheden.
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(switchPin, HIGH);

    //////////////////////////////
   //  Dipswitch tryk til P_X. //
  //////////////////////////////
  // Sæt retningsregistre for DIP-switch pins
  pinMode(dipPin0, INPUT_PULLUP);
  pinMode(dipPin1, INPUT_PULLUP);
  pinMode(dipPin2, INPUT_PULLUP);
  pinMode(dipPin3, INPUT_PULLUP);
  pinMode(dipPin4, INPUT_PULLUP);
  pinMode(dipPin5, INPUT_PULLUP);
  pinMode(dipPin6, INPUT_PULLUP);
  pinMode(dipPin7, INPUT_PULLUP);

  // Aktivér pull-up modstande på DIP-switch pins
  digitalWrite(dipPin0, HIGH);
  digitalWrite(dipPin1, HIGH);
  digitalWrite(dipPin2, HIGH);
  digitalWrite(dipPin3, HIGH);
  digitalWrite(dipPin4, HIGH);
  digitalWrite(dipPin5, HIGH);
  digitalWrite(dipPin6, HIGH);
  digitalWrite(dipPin7, HIGH);

  // Initialiser I2C og OLED-skærm
  i2c_init();             // Åbner lednings nettet
  ssd1306_init();         // OLED Skærm
  reset_diplay();         // Nulstiller skærm
  ssd1306_clearDisplay(); // Renser skærmen

  // ssd1306_printText(0,0,"Peder er GUD!"); // Udskrift til skærm
}

void loop()
{
    // Læs knaptilstand og kontroller LED (uændret)
    int switchState = digitalRead(switchPin);
    digitalWrite(ledPin, switchState == LOW ? HIGH : LOW);

    // Læs bits fra Port 6 (bit 0 til 6) ind i bitsP6
    unsigned char bitsP6 = 0;
    bitsP6 |= (digitalRead(dipPin0) == LOW ? 1 : 0) << 0;
    bitsP6 |= (digitalRead(dipPin1) == LOW ? 1 : 0) << 1;
    bitsP6 |= (digitalRead(dipPin2) == LOW ? 1 : 0) << 2;
    bitsP6 |= (digitalRead(dipPin3) == LOW ? 1 : 0) << 3;
    bitsP6 |= (digitalRead(dipPin4) == LOW ? 1 : 0) << 4;
    bitsP6 |= (digitalRead(dipPin5) == LOW ? 1 : 0) << 5;
    bitsP6 |= (digitalRead(dipPin6) == LOW ? 1 : 0) << 6;

    // Læs mest betydende bit fra Port 7 (bit 0) ind i bitP7_0
    unsigned char bitP7_0 = (digitalRead(dipPin7) == LOW ? 1 : 0);

    // Kombiner bitP7_0 og bitsP6 som anmodet
    bitsP6 = (bitP7_0 << 7) | bitsP6;

    // Konverter den kombinerede værdi til decimal og hexadecimal
    char decimal_buffer[20];
    sprintf(decimal_buffer, "Dec: %d", bitsP6);

    char hex_tal[3];  // Array til at holde hex-cifrene (2 cifre + null-terminator)
    sprintf(hex_tal, "%02X", bitsP6);  // Konverter til hexadecimal ASCII-repræsentation
    
    char hex_buffer[20];
    sprintf(hex_buffer, "Hex: 0x%s", hex_tal);

    // Vis værdierne på OLED-skærmen
    ssd1306_clearDisplay();
    ssd1306_printText(0, 0, decimal_buffer);
    ssd1306_printText(0, 2, hex_buffer);  // Vis på anden linje

    if (switchState == LOW)
  {
    digitalWrite(ledPin, HIGH);
  }
  else
  {
    digitalWrite(ledPin, LOW);
  }

  delay(10);

    delay(500); // Kort forsinkelse før næste læsning
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}

*/