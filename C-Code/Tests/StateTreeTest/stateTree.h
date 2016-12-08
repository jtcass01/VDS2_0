#ifndef __STATE_H__
#define __STATE_H__

#include <math.h>

typedef struct {
  state* root;
  unsigned short capacity;
} stateTree;

bool hasLeft(int, int, int*);
bool hasRight(int current, int max, int* heap);

stateTree* createStateTree(int cap){
  stateTree* pStateTree = malloc(sizeof(short) + (cap*sizeof(state)));
  if(pStateTree != NULL){
    pStateTree->capacity = cap;
//    initializeTree(pStateTree);
  } else {
    free(pStateTree);
  }
}

void initializeTree(stateTree* pStateTree){
  int mid = 0, max = 0, i = 1, left = 0, right = 0;
  int order[pStateTree->capacity];
  
  mid = (pStateTree->capacity/2);
  order[0] = mid;
//  if(hasLeft(mid,0){
    
//  }
}

int* getBinaryHeap(int capacity){
  int heap[capacity];
  int mid = capacity / 2, i = 1;

  heap[0] = mid;

  if(hasLeft(mid,0,heap)){
    i++;
  }
  if(hasRight(mid,capacity,heap)){
    i++;
  }

  for(;i<capacity;){
  }
}

bool hasLeft(int current, int min, int* heap){
  bool result = false;

  if((current-min)/2 == 0){
    return false;
  } else {
    return true;
  }
}

bool hasRight(int current, int max, int* heap){
  bool result = false;
  int possibility = 0;

  possibility = ceil(((float)max-(float)current)/2);

  
  if((max-current)/2 == 0){
    return false;
  } else {
    return true;
  }
}

bool isNew(int* heap, int possibility){
  for(unsigned short i = 0; heap[i]!='\0';i++){
    if(heap[i] == possibility){
      return false;
    }
  }
  return true;
}

void zeroOutHeap(int* heap, int size){
  for(unsigned short i = 0; i < size; i++){
    heap[i] = 0;
  }
}

#endif
