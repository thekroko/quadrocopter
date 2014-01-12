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
  if (output < pid.minOut) return pid.minOut;
  else if (output > pid.maxOut) return pid.maxOut;
  else return output;
}
