#include "state.h"
#include "stateTree.h"
#include "heap.h"
#include <stdio.h>

#define CAPACITY (16)

int heap[CAPACITY];
struct stateTree* pRawStates;
struct state* pTestState;
int testInt;
void setup();

int main(void){
  setup();
  pTestState->time = 100;
  pTestState->alt = 200;
  pTestState->vel = 300;
  pTestState->accel = 400;


  testInt = getTime(pRawStates, 1);



  setState(pRawStates, 8, pTestState);

  printTree(pRawStates);

  deleteStateTree(pRawStates);
  printf("\n\n----- pRawStates Deleted -----\n");

  printf("testInt = %i", testInt);


  deleteState(pTestState);
  printf("\n\n----- pTestState Deleted -----\n");

  return 1;
}

void setup() {
  pTestState = createState(5);
  pRawStates = createStateTree(CAPACITY);
}

void loop() {
}
