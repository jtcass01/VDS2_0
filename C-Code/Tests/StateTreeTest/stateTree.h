#include "state.h"
#include <stdlib.h>
#include <stdio.h>

#define CEILING_POS(X) ((X-(int)(X)) > 0 ? (int)(X+1) : (int)(X))
#define CEILING_NEG(X) ((X-(int)(X)) < 0 ? (int)(X-1) : (int)(X))
#define CEILING(X) ( ((X) > 0) ? CEILING_POS(X) : CEILING_NEG(X) )

struct stateTree {
  struct state* root;
  unsigned short capacity;
};


short hasLeft(int, int, int*);
short hasRight(int, int, int*);
short isNew(int*, int);

struct stateTree* createStateTree(int cap){
  struct stateTree* pStateTree = malloc(sizeof(short) + (cap*sizeof(struct state)));
  if(pStateTree != NULL){
    pStateTree->capacity = cap;
//    initializeTree(pStateTree);
  } else {
    free(pStateTree);
  }
}

void deleteStateTree(struct stateTree* pStateTree){
  deleteState(pStateTree->root);
  free(pStateTree);
}

/*
void initializeTree(struct stateTree* pStateTree){
  int mid = 0, max = 0, i = 1, left = 0, right = 0;
  int order[pStateTree->capacity];

  mid = (pStateTree->capacity/2);
  order[0] = mid;
  printf("\nmid = %i\n",mid);
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
*/

short hasLeft(int current, int min, int* heap){
  int possibility = 0;

  possibility = (int)((current - min) / 2);
  possibility = current - possibility;
  printf("\npossibility %i\n", possibility);

  if(possibility == 0){
    return 0;
  } else {
    if(isNew(heap,possibility)){
      printf("\nLeft node possibilty confirmed.\n");
      return 1;
    } else {
      return 0;
    }
  }
}


short hasRight(int current, int max, int* heap){
  double temp = 0;
  int possibility = 0;

  temp = ((double)max-(double)current)/2;
  printf("\nTEMP %f\n", temp);

  possibility = (int)CEILING(temp);
  possibility += current;
  printf("\npossibility %i\n", possibility);

  if(possibility == 0){
    return 0;
  } else {
    if(isNew(heap,possibility)){
      printf("\nRight node possibilty confirmed.\n");
      return 1;
    } else {
      return 0;
    }
  }
}

short isNew(int* heap, int possibility){
  for(unsigned short i = 0; heap[i]!=0;i++){
    if(heap[i] == possibility){
      return 0;
    }
  }
  return 1;
}

void printHeap(int* heap, short size){
  printf("\n\n----- PRINTING HEAP ------\n");
  for(unsigned short i = 0; i < size; i++){
    printf("\ni = %i     heap[i] = %i", i, heap[i]);
  }
}

void zeroOutHeap(int* heap, short size){
  for(unsigned short i = 0; i < size; i++){
    heap[i] = 0;
  }
}
