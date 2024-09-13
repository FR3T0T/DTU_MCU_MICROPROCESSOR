#include <Arduino.h>
#include "i2c.h"
#include "ssd1306.h"

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
 i2c_init(); // Åbner lednings nettet
 ssd1306_init(); // OLED Skærm
 reset_diplay(); // Nulstiller skærm

 ssd1306_printText(0,0,"hej med dig"); // Udskrift til skærm
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}