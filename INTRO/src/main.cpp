#include <Arduino.h>

const int switchPin = P2_1; 
const int ledPin = P1_0;

void setup()
{
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(switchPin, HIGH);
}

void loop()
{
  int switchState = digitalRead(switchPin);

  if (switchState == LOW) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  delay(10);
}
