#ifndef __STATE_H
#define __STATE_H

#include <stdlib.h>
#include <stdio.h>

struct state {
  unsigned short key;
  float time;
  float alt;
  float vel;
  float accel;
  struct state* leftChild;
  struct state* rightChild;
};


struct state* createState(int);
void deleteState(struct state*);
void printState(struct state*);

struct state* createState(int key){
  struct state* pState = malloc(sizeof(struct state));
  printf("\n\n-----Creating state with key value: %i-----",key);
  if(pState != NULL){
    pState->key = key;
    pState->time = 0;
    pState->alt = 0;
    pState->vel = 0;
    pState->accel = 0;
    printState(pState);
  } else {
    printf("\n\nERROR WITHIN CREATE STATE!! NULL POINTER");
    free(pState);
  }
  return pState;
}

void deleteState(struct state* pState){
  free(pState);
}

void printState(struct state* pState){
  printf("\n\n----- Printing State -----");
  printf("\nKey = %i", pState->key);
  printf("\nTime = %i", pState->time);
  printf("\nAltitude = %i", pState->alt);
  printf("\nVelocity = %i", pState->vel);
  printf("\nAcceleration = %i", pState->accel);
  printf("\nleftChild = %p", pState->leftChild);
  printf("\nrightChilde = %p", pState->rightChild);
}


#endif
