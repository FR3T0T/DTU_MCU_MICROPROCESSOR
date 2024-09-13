#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  pinMode(P1_1,INPUT_PULLUP);
  digitalWrite(P1_1,HIGH);
  pinMode(P1_0,OUTPUT);
  /*
  R1REN |=BIT1;    //R1REN=R1REN|BIT1;
  P1OUT |=BIT1;
  */

}

void loop() {

if((digitalRead(P1_1)==0)){
  digitalWrite(P1_0,HIGH);
  }
  else
   digitalWrite(P1_0,LOW);

  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}