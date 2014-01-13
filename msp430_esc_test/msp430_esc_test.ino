#include "esc.h"

void setup()
{
  Serial.begin(9600);
  Serial.println("MSP ESC Tester here");
  initESCs();
  setESCs(900, 900, 900, 900);
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c >= '0' && c <= '9') {
      uint16_t p = (uint16_t)(c-'0')*100L;
      if (c == '9') p += 100;
      setESCs(p, p, p, p);
      Serial.print(c);
    }
  }
}
