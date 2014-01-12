// PID magic
struct PID {
  float kP, kD, kI;
  float minI, maxI;
  float minOut, maxOut;
 
  float errorSum;
  float lastInput;
};

float updatePID(PID pid, float input, float target);
