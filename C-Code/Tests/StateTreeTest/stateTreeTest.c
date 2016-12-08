#include "state.h"
#include "stateTree.h"
#include <stdio.h>

#define CAPACITY (16)

int heap[CAPACITY];
struct stateTree* pRawStates;
struct state* pTestState;
void setup();

int main(void){
  setup();
  printf("\n----- pTestState Created -----\n");
  printf("\nPosition = %i", pTestState->position);
  printf("\nTime = %f", pTestState->time);
  printf("\nAltitude = %f", pTestState->alt);
  printf("\nVelocity = %f", pTestState->vel);
  printf("\nAcceleration = %f", pTestState->accel);
  printf("\nleftChild = %p", pTestState->leftChild);
  printf("\nrightChild = %p", pTestState->rightChild);
  deleteState(pTestState);
  printf("\n\n----- pTestState Deleted -----\n");

  printf("\n----- pRawStates Created -----\n");
  printf("\nCapacity = %i", pRawStates->capacity);

  zeroOutHeap(heap, CAPACITY);
  heap[0] = 8;
  if(isNew(heap,1)){
    printf("\n1 is new.");
  } else {
    printf("\n1 is not new.");
  }
  hasRight(heap[0], CAPACITY, heap);
  hasLeft(heap[0], 0, heap);
  printHeap(heap, CAPACITY);


  deleteStateTree(pRawStates);
  printf("\n\n----- pRawStates Deleted -----\n");

  return 1;
}

void setup() {
  pTestState = createState(5);
  pRawStates = createStateTree(CAPACITY);

  //rawStates = createStateTree(CAPACITY);
}

void loop() {
}
