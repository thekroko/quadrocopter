#include "pid.h"

/*Performs a single PID step */
float updatePID(PIDConfig &pidc, PIDData &pid, float input, float target) {
  float error = target - input;
  
  // Update I (prevents freezing)
  pid.errorSum += error * pidc.kI;
  if (pid.errorSum > pidc.maxI) pid.errorSum = pidc.maxI;
  if (pid.errorSum < pidc.minI) pid.errorSum = pidc.minI;
  
  // Update D (damper)
  float ddt_error = pidc.kD * (input - pid.lastInput);
  pid.lastInput = input;
  
  // Calculate output
  float output = pidc.kP * error + ddt_error + pid.errorSum;
  if (output < pidc.minO) return pidc.minO;
  else if (output > pidc.maxO) return pidc.maxO;
  else return output;
}
