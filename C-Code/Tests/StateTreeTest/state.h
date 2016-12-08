#ifndef __STATE_H
#define __STATE_H

#include <stdlib.h>

struct state {
  unsigned short position;
  float time;
  float alt;
  float vel;
  float accel;
  struct state* leftChild;
  struct state* rightChild;
};


struct state* createState(int pos){
  struct state* pState = malloc(sizeof(struct state));
  if(pState != NULL){
    pState->position = pos;
    pState->alt = 0;
    pState->vel = 0;
    pState->accel = 0;
  } else {
    free(pState);
  }
  return pState;
}

void deleteState(struct state* pState){
  free(pState);
}


#endif
