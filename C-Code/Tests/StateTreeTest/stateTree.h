#include "state.h"
#include "heap.h"
#include <stdlib.h>
#include <stdio.h>

struct stateTree {
  struct state* root;
  unsigned short capacity;
};


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
*/
