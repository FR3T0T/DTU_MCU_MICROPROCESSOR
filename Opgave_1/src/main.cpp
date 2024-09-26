#include <Arduino.h>
#include <stdint.h>
#include "i2c.h"
#include "ssd1306.h"

#define NUM_OPERATIONS 4

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

// Declare global variables before any functions
unsigned char variable1 = 0, variable2 = 0; // Accessible globally
int operationCount = 0;
char operations[] = {'+', '-', '*', '/'};
char expressionStr[30]; // Holds the expression being built
char resultStr[30];     // Holds the final result or error message

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

void checkOperationSwitch() {
  if (digitalRead(operationSwitchPin) == LOW) {
    operationCount = (operationCount + 1) % NUM_OPERATIONS;
    while (digitalRead(operationSwitchPin) == LOW) {
      delay(50); // Debounce
    }
    // Update display with new operation
    ssd1306_clearDisplay();
    sprintf(expressionStr, "%d %c", variable1, operations[operationCount]);
    ssd1306_printText(0, 0, expressionStr);
    delay(500); // Brief delay to allow the user to see the change
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

  // ======== Step 1: Input First Variable ========
  ssd1306_clearDisplay();
  ssd1306_printText(0, 0, "Set First Number");
  while (digitalRead(switchPin) == HIGH) {
    checkOperationSwitch(); // Check for operation change
    variable1 = readDipswitch();
    sprintf(expressionStr, "%d", variable1);
    ssd1306_printText(0, 1, expressionStr);
    delay(100); // Update every 100ms
  }
  while (digitalRead(switchPin) == LOW) {
    delay(50); // Debounce
  }

  // ======== Step 2: Select Operation ========
  ssd1306_clearDisplay();
  ssd1306_printText(0, 0, "Select Operation");
  // Display current expression with operation
  sprintf(expressionStr, "%d %c", variable1, operations[operationCount]);
  ssd1306_printText(0, 1, expressionStr);
  // Wait for the user to press the switchPin to confirm operation
  while (digitalRead(switchPin) == HIGH) {
    checkOperationSwitch(); // Allow changing operation
    // Update the displayed operation
    sprintf(expressionStr, "%d %c", variable1, operations[operationCount]);
    ssd1306_printText(0, 1, expressionStr);
    delay(100); // Update every 100ms
  }
  while (digitalRead(switchPin) == LOW) {
    delay(50); // Debounce
  }

  // ======== Step 3: Input Second Variable ========
  ssd1306_clearDisplay();
  ssd1306_printText(0, 0, "Set Second Number");
  while (digitalRead(switchPin) == HIGH) {
    checkOperationSwitch(); // Check for operation change
    variable2 = readDipswitch();
    // Update the expression with the second variable
    sprintf(expressionStr, "%d %c %d", variable1, operations[operationCount], variable2);
    ssd1306_printText(0, 1, expressionStr);
    delay(100); // Update every 100ms
  }
  while (digitalRead(switchPin) == LOW) {
    delay(50); // Debounce
  }

  // ======== Step 4: Display Complete Expression ========
  ssd1306_clearDisplay();
  sprintf(expressionStr, "%d %c %d", variable1, operations[operationCount], variable2);
  ssd1306_printText(0, 0, "Calculating:");
  ssd1306_printText(0, 1, expressionStr);
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
  ssd1306_clearDisplay();
  if (!error) {
    sprintf(resultStr, "%d %c %d = %d", variable1, operations[operationCount], variable2, result);
    ssd1306_printText(0, 0, "Result:");
    ssd1306_printText(0, 1, resultStr);
  } else {
    ssd1306_printText(0, 0, resultStr); // Display error message
  }
  // Display prompt to continue
  ssd1306_printText(0, 2, "Press button to continue");

  // ======== Step 7: Wait for OperationSwitchPin Press to Proceed ========
  // Wait until operationSwitchPin is pressed
  while (digitalRead(operationSwitchPin) == HIGH) {
    // Wait for button press
    delay(50); // Debounce delay
  }
  while (digitalRead(operationSwitchPin) == LOW) {
    // Wait for button release
    delay(50); // Debounce delay
  }

  // After button press, proceed to next calculation
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