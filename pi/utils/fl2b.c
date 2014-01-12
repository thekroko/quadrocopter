#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv) {
  char* numStr = argv[1];
  float num = atof(numStr);
  int numI = *(int*)(&num);
  //printf("0x%1X\n", numI);
  for (int i = 0; i < 4; i++) {
    int c = numI & 0xFF;
    printf("0x%02X ", c);
    numI >>= 8;
  }
  printf("\n");
}
