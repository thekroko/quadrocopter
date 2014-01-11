#include "Servo.h"

// Pinout
#define LED RED_LED
#define MOTOR_TL P1_5
#define MOTOR_TR P1_4
#define MOTOR_BL P2_0
#define MOTOR_BR P2_1

Servo motorTL, motorTR, motorBL, motorBR; 
bool blinkState = false;

// ------------------ Motor helpers
void setMotors(uint8_t tl, uint8_t tr, uint8_t bl, uint8_t br) { // values in permill
  motorTL.writeMicroseconds((uint16_t)tl*3L + 1235L);
  motorTR.writeMicroseconds((uint16_t)tr*3L + 1235L);
  motorBL.writeMicroseconds((uint16_t)bl*3L + 1235L);
  motorBR.writeMicroseconds((uint16_t)br*3L + 1235L);
}

// -----------------------------------
void setup() {
    // Make sure RX has a pullup (we get random interference otherwise)
    pinMode(P1_1, INPUT_PULLUP);
    Serial.begin(115200);
    Serial.println();
    Serial.println("INIT MSP430 starting up..");
    
    // Setup LED
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
    
    // Initialize Servos
    Serial.println("INIT Calibrating ESCs..");
    motorTL.attach(MOTOR_TL); motorTR.attach(MOTOR_TR); motorBL.attach(MOTOR_BL); motorBR.attach(MOTOR_BR);
    setMotors(254, 254, 254, 254);
    delay(5000);
    setMotors(0, 0, 0, 0);
    delay(2000);
    
    // All done
    Serial.println("RDY All ready!");
}

void loop() {
    // Handle input. We expect 4 bytes of motor speeds (0x00 to 0xFE), and a single 0xFF.
    if (Serial.available() < 5) return; // not yet ready
    
    uint8_t speeds[4];
    for (int i = 0; i < 4; i++) {
      uint8_t val = Serial.read();
      if (val == 0xFF) return; // illegal package here
      speeds[i] = val;
    }
    if (Serial.read() != 0xFF) return;
    
    // We have a valid motor speed...
    setMotors(speeds[0], speeds[1], speeds[2], speeds[3]);
    Serial.print('+');
    blinkState = !blinkState;
    digitalWrite(LED, blinkState);
}
