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
  // put your main code here, to run repeatedly:
  int switchState = digitalRead(switchPin);

  // Læs værdien fra DIP-switch
  int dipValue = 0;
  dipValue |= (digitalRead(dipPin0) == LOW ? 1 : 0) << 0;
  dipValue |= (digitalRead(dipPin1) == LOW ? 1 : 0) << 1;
  dipValue |= (digitalRead(dipPin2) == LOW ? 1 : 0) << 2;
  dipValue |= (digitalRead(dipPin3) == LOW ? 1 : 0) << 3;
  dipValue |= (digitalRead(dipPin4) == LOW ? 1 : 0) << 4;
  dipValue |= (digitalRead(dipPin5) == LOW ? 1 : 0) << 5;
  dipValue |= (digitalRead(dipPin6) == LOW ? 1 : 0) << 6;
  dipValue |= (digitalRead(dipPin7) == LOW ? 1 : 0) << 7;


// Ved tryk, tændes diode.
  if (switchState == LOW)
  {
    digitalWrite(ledPin, HIGH);
  }
  else
  {
    digitalWrite(ledPin, LOW);
  }

  delay(10);


  // Konverter værdien til en streng
  char buffer[16];
  sprintf(buffer, "Vaerdi: %d", dipValue);

  // Vis værdien på OLED-skærmen
  ssd1306_clearDisplay();          // Renser skærmen
  ssd1306_printText(0, 0, buffer); // Udskriver værdien på skærmen

  delay(500); // Kort forsinkelse før næste læsning
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}