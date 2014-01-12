#include <stdint.h>

// PID magic
struct PIDConfig {
  float kP, kI, kD;
  int8_t minI, maxI, minO, maxO;
};

struct PIDData {
  float lastInput;
  float errorSum;
};

float updatePID(PIDConfig &pidc, PIDData &pid, float &input, float &target);
