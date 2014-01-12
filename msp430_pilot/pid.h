#include <stdint.h>

// PID magic
struct PID {
  float kP, kI, kD;
  int8_t minI, maxI, minO, maxO;
  float lastInput;
  float errorSum;
};

float updatePID(PID pid, float input, float target);
