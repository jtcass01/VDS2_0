#include "state.h"
#include "heap.h"
#include <stdlib.h>
#include <stdio.h>

struct stateTree {
  struct state* root;
  unsigned short capacity;
};

//Initialization of Tree
struct stateTree* createStateTree(int);
void initializeTree(struct stateTree*);
void insert(const short, struct stateTree*);
void insertState(const short, struct state**);


//Getters
struct state* getState(struct stateTree*, const short);
struct state* findState(struct state*, const short);
float getTime(struct stateTree*, const short);
float getAltitude(struct stateTree*, const short);
float getVelocity(struct stateTree*, const short);
float getAcceleration(struct stateTree*, const short);


//Setters
void setState(struct stateTree*, const short, struct state*);
void setTime(struct stateTree*, const short, float);
void setAltitude(struct stateTree*, const short, float);
void setVelocity(struct stateTree*, const short, float);
void setAcceleration(struct stateTree*, const short, float);


//Deletion
void deleteStateTree(struct stateTree*);
void deleteBranch(struct state*);


//Display
void printTree(struct stateTree*);
void printBranch(struct state*);


//Misc
int isEmpty(struct stateTree*);


/***************     INITIALIZATION     ***************/
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


void initializeTree(struct stateTree* pStateTree){
  int binaryHeap[pStateTree->capacity];

  getBinaryHeap(binaryHeap, pStateTree->capacity);

  for(unsigned short binaryHeap_i = 0; binaryHeap_i < pStateTree->capacity; binaryHeap_i++){
    insert(binaryHeap[binaryHeap_i], pStateTree);
  }

  printf("\n\nBinary State Tree initialized... printing tree for confirmation..");
  printTree(pStateTree);
}


void insert(const short key, struct stateTree* pStateTree){
  struct state** pRoot = &(pStateTree->root);

  printf("\n\nInserting into binary tree...");
  insertState(key, pRoot);
  printTree(pStateTree);
}

void insertState(const short key, struct state** pState){
  if(*pState == NULL){
    (*pState) = createState(key);
  } else if (key < (*pState)->key){
    insertState(key, &(*pState)->leftChild);
  } else if (key > (*pState)->key){
    insertState(key, &(*pState)->rightChild);
  }
  printState((*pState));
}


/***************     GETTERS     ***************/
struct state* getState(struct stateTree* pStateTree, const short key){
  if(pStateTree->root == NULL){
    printf("\n\nState Tree is empty");
  } else {
    return findState(pStateTree->root, key);
  }
}

struct state* findState(struct state* pState, const short key){
  if(pState->key == key){
    return pState;
  } else if (pState->key < key){
    findState(pState->rightChild, key);
  } else if (pState->key > key){
    findState(pState->leftChild, key);
  }
}


float getTime(struct stateTree* pStateTree, const short key){
  struct state* pState = getState(pStateTree, key);
  return pState->time;
}


float getAltitude(struct stateTree* pStateTree, const short key){
  struct state* pState = getState(pStateTree, key);
  return pState->alt;
}


float getVelocity(struct stateTree* pStateTree, const short key){
  struct state* pState = getState(pStateTree, key);
  return pState->vel;
}


float getAcceleration(struct stateTree* pStateTree, const short key){
  struct state* pState = getState(pStateTree, key);
  return pState->accel;
}


/***************     SETTERS     ***************/
void setState(struct stateTree* pStateTree, const short key, struct state* pNewState){
    struct state* pOldState = getState(pStateTree, key);
    pOldState->time = pNewState->time;
    pOldState->alt = pNewState->alt;
    pOldState->vel = pNewState->vel;
    pOldState->accel = pNewState->accel;
}


void setTime(struct stateTree* pStateTree, const short key, float newTime){
  struct state* pState = getState(pStateTree, key);
  pState->time = newTime;
}


void setAltitude(struct stateTree* pStateTree, const short key, float newAlt){
  struct state* pState = getState(pStateTree, key);
  pState->alt = newAlt;
}


void setVelocity(struct stateTree* pStateTree, const short key, float newVel){
  struct state* pState = getState(pStateTree, key);
  pState->vel = newVel;
}


void setAcceleration(struct stateTree* pStateTree, const short key, float newAccel){
  struct state* pState = getState(pStateTree, key);
  pState->accel = newAccel;
}


/***************     DELETION     ***************/
void deleteStateTree(struct stateTree* pStateTree){
  deleteBranch(pStateTree->root);
  free(pStateTree);
}

void deleteBranch(struct state* pState){
  if(pState == NULL){

  } else {
    deleteBranch(pState->leftChild);
    deleteState(pState);
    deleteBranch(pState->rightChild);
  }
}


/***************     DISPLAY     ***************/
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
    printf("\nPosition = %i", pState->key);
    printf("\nTime = %f", pState->time);
    printf("\nAltitude = %f", pState->alt);
    printf("\nVelocity = %f", pState->vel);
    printf("\nAcceleration = %f", pState->accel);
    printf("\nleftChild = %p", pState->leftChild);
    printf("\nrightChild = %p", pState->rightChild);
    printBranch(pState->rightChild);
  }
}


/***************     MISC     ***************/
int isEmpty(struct stateTree* pStateTree){
  return ((pStateTree->root == NULL) ? 1 : 0);
}
