#define LED RED_LED
#define DMP_PACKET_SIZE 42
#define DMP_BUFF_SIZE 16 // only the quaternion
#define RAD2DEG (180.0/M_PI)

#include "Energia.h"
#include "usci_isr_handler.h"
#include "foo.h"

void i2c_state_isr(void) {}
void i2c_txrx_isr(void) {}

#define STACK_MAGIC 0x3F
uint8_t* ramEnd; // this ends up in the far end of the ram, next to the stack. (but before our includes)
inline void checkStack(char c) { // this takes ~63us
  if (*ramEnd != STACK_MAGIC) {
    // memory has been corrupted by something
    Serial.print("!!!STACK OVERFLOW @ "); Serial.print(c); Serial.println("!!!");
    Serial.flush();
    while (1) ; // halt
  }
}

#define RESET_MEASURE { TA1R = 0; }
void MEASURE(char* x) { 
  uint16_t t = TA1R;
  Serial.print(x); Serial.print(": us="); Serial.println((uint32_t)t * 8L / clockCyclesPerMicrosecond()); 
  Serial.flush(); TA1R = 0;
}

// -----------------------------------
void setup() {    
    // Initialize all floats (printing crashes device otherwise)
    ramEnd = (uint8_t*)malloc(1);
    *ramEnd = STACK_MAGIC;

    // Make sure RX has a pullup (we get random interference otherwise)
    pinMode(P1_1, INPUT_PULLUP);
    Serial.begin(9600);
    Serial.println();
    Serial.println("MSP430 Test.");
    
    // Setup a cycle timer to measure performance
    /*TA1CTL = TASSEL_2 + MC_2 + ID_3; // SMCLK/8, count to 
    TA1CCTL0 = 0;
    i2c_state_isr();*/
    pinMode(RED_LED,OUTPUT);
    digitalWrite(RED_LED, LOW);
    delay(500);
    initESCs();
    setESCs(0, 0, 0, 0);
}

void loop() {

  //RESET_MEASURE // reset timer
  P1OUT ^= 1 << 0;
  //digitalWrite(P1_6, HIGH);
  //MEASURE("Loop");

  delay(200);
}
