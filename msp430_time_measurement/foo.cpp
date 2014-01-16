#include "foo.h"

#include "Energia.h"

#define P_TL P1_5
#define P_TR P1_4
#define P_BL P2_0
#define P_BR P2_1

#define HZ 490 // must be <= 490
#define PERIOD (1000000L / HZ * TICKS_PER_US)
#define TICKS_PER_US 2 // ~required time to toggle a pin
#define TRIM_US 4

volatile uint16_t times[4];

// interrupts for top/middle of the period
__attribute__((interrupt(TIMER0_A1_VECTOR))) static void Timer0_A1_int(void) { 
  switch (TA0IV) {
    case TA0IV_TACCR1: // CCR1 = TL, P1_5
      P1OUT &= ~(1 << 5); // disable
      break;
    case TA0IV_TACCR2: // CCR2 = TR, P1_4
      P1OUT &= ~(1 << 4); // disable
      break;
    case TA0IV_TAIFG: // TAIFG -- bottom: update & enable everything
      P1OUT |= (1 << 5); // make sure everything is enabled here
      P1OUT |= (1 << 4); // enable this one 2us later
      // we have like 1000 us to adjust those two registers ...
      TA0CCR1 = times[0];
      TA0CCR2 = times[1];    
      break;
  }
}

__attribute__((interrupt(TIMER1_A1_VECTOR))) static void Timer1_A1_int(void) { 
  switch (TA1IV) {
    case TA1IV_TACCR1: // CCR1 = BL, P2_0
      P2OUT &= ~(1 << 0); // disable
      break;
    case TA1IV_TACCR2: // CCR2 = BR, P2_1
      P2OUT &= ~(1 << 1); // disable
      break;
    case TA1IV_TAIFG: // TAIFG -- bottom: update & enable everything
      P2OUT |= (1 << 0); // make sure everything is enabled here
      TA1CCR1 = times[2];
      P2OUT |= (1 << 1); // enable this one 2us later
      TA1CCR2 = times[3];    
      break;
  }
}

void initESCs() {
  // Enable pins. Digital write is required to convey the changes
  pinMode(P_TL, OUTPUT);
  pinMode(P_TR, OUTPUT);
  pinMode(P_BL, OUTPUT);
  pinMode(P_BR, OUTPUT);  
  digitalWrite(P_TL, LOW);
  digitalWrite(P_TR, LOW);
  digitalWrite(P_BL, LOW);
  digitalWrite(P_BR, LOW);     
  setESCs(0, 0, 0, 0);
  
  // Setup timer0
  TA0CCR0 = PERIOD-1; // period
  TA0CCR1 = 1000*TICKS_PER_US;  
  TA0CCR2 = 1000*TICKS_PER_US;
  TA0CCTL0 = 0; // no interrupt
  TA0CCTL1 = CCIE; // enable interrupt
  TA0CCTL2 = CCIE; // enable interrupt
  TA0R = 0;
  
  // Setup timer1
  TA1CCR0 = PERIOD-1; // period
  TA1CCR1 = 1000*TICKS_PER_US;  
  TA1CCR2 = 1000*TICKS_PER_US;
  TA1CCTL0 = 0; // no interrupt
  TA1CCTL1 = CCIE; // enable interrupt
  TA1CCTL2 = CCIE; // enable interrupt
  TA1R = PERIOD/4; // slightly offset this one to even CPU load
  
  // Enable both timers
  TA0CTL = TASSEL_2 + MC_1 + ID_3 + TAIE; // prescale SMCLK/8, up mode => 2 ticks per us
  TA1CTL = TASSEL_2 + MC_1 + ID_3 + TAIE; // prescale SMCLK/8, up mode => 2 ticks per us
}

void setESCs(uint16_t tl, uint16_t tr, uint16_t bl, uint16_t br) {
  //[1] and [3] are slightly offset since interrupts will otherwise trigger at the same time
  times[0] = (tl + 1000 - TRIM_US) * TICKS_PER_US - 1; // 2000-4000
  times[1] = (tr + 1000 - TRIM_US) * TICKS_PER_US - 1 + 4;
  times[2] = (bl + 1000 - TRIM_US) * TICKS_PER_US - 1;
  times[3] = (br + 1000 - TRIM_US) * TICKS_PER_US - 1 + 4;
}
