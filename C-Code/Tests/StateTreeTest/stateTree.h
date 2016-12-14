#include "state.h"
#include "heap.h"
#include <stdlib.h>
#include <stdio.h>

struct stateTree {
  struct state* root;
  unsigned short capacity;
};

struct stateTree* createStateTree(int);
void deleteStateTree(struct stateTree*);
void initializeTree(struct stateTree*);
void insert(const int, struct stateTree*);
void insert(const int, struct state*);
void printTree(struct stateTree*);
void printTree(struct state*);


struct stateTree* createStateTree(int capacity){
  struct stateTree* pStateTree = malloc(sizeof(short) + (capacity*sizeof(struct state)));
  if(pStateTree != NULL){
    pStateTree->capacity = capacity;
//    initializeTree(pStateTree);
  } else {
    free(pStateTree);
  }
}


void deleteStateTree(struct stateTree* pStateTree){
  deleteState(pStateTree->root);
  free(pStateTree);
}


void initializeTree(struct stateTree* pStateTree){
  int binaryHeap[pStateTree->capacity];

  getBinaryHeap(binaryHeap, pStateTree->capacity);

  for(unsigned short binaryHeap_i = 0; binaryHeap_i < pStateTree->capacity; binaryHeap_i++){
    insert(binaryHeap[binaryHeap_i], pStateTree);
  }
}


void insert(const int pos, struct stateTree* pStateTree){
  insert(pos, pStateTree->root);
}

void insert(const int pos, struct state* pState){
  if(pState == NULL){
    pState = createState(pos);
  } else if (pos < pState->position){
    insert(pos, pState->leftChild);
  } else if (pos > pState->position){
    insert(pos, pState->rightChild);
  }
}


int isEmpty(struct stateTree* pStateTree){
  return ((pStateTree->root == NULL) ? 1 : 0);
}


void printTree(struct stateTree* pStateTree){
  printf("\n\nPRINTING BINARY TREE");


}


void printTree(struct state* pState){

}
