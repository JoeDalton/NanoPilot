#include "NP_ServoDriver.h"
NP_ServoDriver sd;
int SERVOMIN = 150;
int SERVOMAX = 600;
int servoNum = 0;

String inString = "";  // string to hold input

void setup() {
  Serial.begin(115200);
  wait_for_serial();
  Serial.println("Initializing sd");
  sd.init();
  Serial.println("Sd initialized");
  Serial.println("<Arduino is ready>");
  

}

void loop() {

    /*Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    sd.pwm.setPWM(servonum, 0, pulselen);
  }

  delay(1000);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    sd.pwm.setPWM(servonum, 0, pulselen);
  }
  delay(1000);
}*/
  
  // Read serial input:
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      Serial.print("Value:");
      Serial.println(inString.toInt());
      Serial.print("String: ");
      Serial.println(inString);
      uint16_t pulsew = inString.toInt();
      //sd.pwm.setPWM(servonum, 0, pulsew);
      sd.set_pwm(3, pulsew);
      //sd.set_pwm(servoNum, pulsew);
      // clear the string for new input:
      inString = "";
    }
  }
}


void wait_for_serial() {
  while (!Serial)
    delay(10); // Pauses the controller until a serial console opens
}
