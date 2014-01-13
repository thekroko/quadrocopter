#include "esc.h"

#include "Energia.h"

uint16_t activeSpeeds[4];
uint16_t nextSpeeds[4];
uint16_t elapsed;
uint8_t toExpire;

#define P_TL P2_3
#define P_TR P2_2
#define P_BL P2_1
#define P_BR P2_0

#define HZ 490 // must be less than 500
#define PERIOD (1000000L / HZ * TICKS_PER_US)
#define TICKS_PER_US 2
#define TRIM_US 0

static void TimerA(void) {
  uint16_t wait;
  do {
    // Timer tick. Operate based upon state.
    if (elapsed >= PERIOD) { // all ESCs have to be off/done by now; start a new tick
      digitalWrite(P_TL, HIGH);
      digitalWrite(P_TR, HIGH);
      digitalWrite(P_BL, HIGH);
      digitalWrite(P_BR, HIGH);
      elapsed = 0;
      for (int i = 0; i < 4; i++)
        activeSpeeds[i] = nextSpeeds[i];
    }
    else {
      // Check for expired timers
      activeSpeeds[toExpire] = 0xFFFF;
      switch (toExpire) {
        default: digitalWrite(P_TL, LOW); break;
        case 1: digitalWrite(P_TR, LOW); break;
        case 2: digitalWrite(P_BL, LOW); break;
        case 3: digitalWrite(P_BR, LOW); break;
      }
    }
    
    // Pick next minimal due time
    wait = PERIOD;
    for (int i = 0; i < 4; i++) {
      uint16_t t = activeSpeeds[i];
      if (t < wait)  { 
        wait = t; 
        toExpire = i;
      }
    }
    wait -= elapsed;
  } while (wait == 0);
  TA0CCR0 = wait;
  elapsed += wait;
}

__attribute__((interrupt(TIMER0_A0_VECTOR))) static void Timer_A_int(void) { TimerA(); }

void dbg() {
  Serial.print(elapsed);
  Serial.print('\t');
  Serial.print(activeSpeeds[0]);  Serial.print('\t');
  Serial.print(activeSpeeds[1]);  Serial.print('\t');
  Serial.print(activeSpeeds[2]);  Serial.print('\t');
  Serial.print(activeSpeeds[3]); Serial.println('\t');
  TimerA();
}

void initESCs() {
  // Enable pins
  pinMode(P_TL, OUTPUT);
  pinMode(P_TR, OUTPUT);
  pinMode(P_BL, OUTPUT);
  pinMode(P_BR, OUTPUT);      
  setESCs(0, 0, 0, 0);
  elapsed = 0xFFFF;
  
  // Enable timers
  TA0CCTL0 = CCIE; // interrupt
  TA0CTL = TASSEL_2 + MC_1 + ID_3; // prescale SMCLK/8, upmode => 2 ticks per us
  TA0CCR0 = 10; // small countdown
}

void setESCs(uint16_t tl, uint16_t tr, uint16_t bl, uint16_t br) {
  nextSpeeds[0] = (tl + 1000 - TRIM_US) * TICKS_PER_US; // 2000-4000
  nextSpeeds[1] = (tr + 1000 - TRIM_US) * TICKS_PER_US;
  nextSpeeds[2] = (bl + 1000 - TRIM_US) * TICKS_PER_US;
  nextSpeeds[3] = (br + 1000 - TRIM_US) * TICKS_PER_US;
}
