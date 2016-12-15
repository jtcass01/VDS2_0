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
void insertState(const int, struct state**);
void printTree(struct stateTree*);
void printBranch(struct state*);


struct stateTree* createStateTree(int capacity){
  struct stateTree* pStateTree = malloc(sizeof(short) + sizeof(struct state));
  if(pStateTree != NULL){
    pStateTree->capacity = capacity;
    initializeTree(pStateTree);
    return pStateTree;
  } else {
    free(pStateTree);
    return (struct stateTree*)NULL;
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

  printf("\n\nBinary State Tree initialized... printing tree for confirmation..");
  printTree(pStateTree);
}


void insert(const int pos, struct stateTree* pStateTree){
  struct state** pRoot = &(pStateTree->root);

  printf("\n\nInserting into binary tree...");
  insertState(pos, pRoot);
  printTree(pStateTree);
}

void insertState(const int pos, struct state** pState){
  if(*pState == NULL){
    (*pState) = createState(pos);
  } else if (pos < (*pState)->position){
    insertState(pos, &(*pState)->leftChild);
  } else if (pos > (*pState)->position){
    insertState(pos, &(*pState)->rightChild);
  }
  printState((*pState));
}


int isEmpty(struct stateTree* pStateTree){
  return ((pStateTree->root == NULL) ? 1 : 0);
}


void printTree(struct stateTree* pStateTree){
  printf("\n\nPRINTING BINARY TREE");

  if(isEmpty(pStateTree)){
    printf("\nTree is empty.");
  } else {
    printBranch(pStateTree->root);
  }
}


void printBranch(struct state* pState){
  if(pState != NULL) {
    printBranch(pState->leftChild);
    printf("\n\n\n----- Printing Tree Node -----");
    printf("\nPosition = %i", pState->position);
    printf("\nTime = %f", pState->time);
    printf("\nAltitude = %f", pState->alt);
    printf("\nVelocity = %f", pState->vel);
    printf("\nAcceleration = %f", pState->accel);
    printf("\nleftChild = %p", pState->leftChild);
    printf("\nrightChild = %p", pState->rightChild);
    printBranch(pState->rightChild);
  }
}
