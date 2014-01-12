#include "pid.h"

/*Performs a single PID step */
float updatePID(PID pid, float input, float target) {
  float error = target - input;
  
  // Update I (prevents freezing)
  pid.errorSum += error * pid.kI;
  if (pid.errorSum > pid.maxI) pid.errorSum = pid.maxI;
  if (pid.errorSum < pid.minI) pid.errorSum = pid.minI;
  
  // Update D (damper)
  float ddt_error = -pid.kD * (input - pid.lastInput);
  pid.lastInput = input;
  
  // Calculate output
  float output = pid.kP * error + ddt_error + pid.errorSum;
  if (output < pid.minO) return pid.minO;
  else if (output > pid.maxO) return pid.maxO;
  else return output;
}
