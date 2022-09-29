#include "PmodTC1.h"

int x = 3;
//PCB oven shit
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  Myfunc(&x);
  Serial.print(x);
  // put your main code here, to run repeatedly:

}
