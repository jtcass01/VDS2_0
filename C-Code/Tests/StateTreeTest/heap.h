#ifndef __HEAP_H
#define __HEAP_H
#include <stdlib.h>
#include <stdio.h>

#define CEILING_POS(X) ((X-(int)(X)) > 0 ? (int)(X+1) : (int)(X))
#define CEILING_NEG(X) ((X-(int)(X)) < 0 ? (int)(X-1) : (int)(X))
#define CEILING(X) ( ((X) > 0) ? CEILING_POS(X) : CEILING_NEG(X) )

short hasLeft(int, int, int*);
int getLeft(int, int, int*);
short hasRight(int, int, int*);
int getRight(int, int, int*);
short isNew(int*, int);


void getBinaryHeap(int* heap, int capacity){
  int mid = capacity / 2, i = 1, j=0;

  heap[0] = mid;

  for(;i<capacity;){
    printf("\n\nNode being tested: %i     j = %i", heap[j], j);
    if(heap[j] < mid){
      if(hasLeft(heap[j],0,heap)){
        printf("\nInserting %i at leftNode.     i = %i     j = %i     heapLocation = %i", getLeft(heap[j],0,heap), i, j, (2*j+1));
        heap[(2*j+1)] = getLeft(heap[j],0,heap);
        i++;
      }
      if(hasRight(heap[j],mid,heap)){
        printf("\nInserting %i at rightNode.     i = %i     j = %i     heapLocation = %i", getRight(heap[j],mid,heap), i, j, (2*j+2));
        heap[(2*j+2)] = getRight(heap[j],mid,heap);
        i++;
      }
    } else if (heap[j] > mid){
      if(hasLeft(heap[j],mid,heap)){
        printf("\nInserting %i at leftNode.     i = %i     j = %i     heapLocation = %i", getLeft(heap[j],mid,heap), i, j, (2*j+1));
        heap[(2*j+1)] = getLeft(heap[j],mid,heap);
        i++;
      }
      if(hasRight(heap[j],capacity,heap)){
        printf("\nInserting %i at rightNode.     i = %i     j = %i     heapLocation = %i", getRight(heap[j],capacity,heap), i, j, (2*j+2));
        heap[(2*j+2)] = getRight(heap[j],capacity,heap);
        i++;
      }
    } else {
      if(hasLeft(mid,0,heap)){
        heap[i] = getLeft(mid,0,heap);
        i++;
      }
      if(hasRight(mid,capacity,heap)){
        heap[i] = getRight(mid, capacity, heap);
        i++;
      }
    }
    j++;
  }

/*  if(heap[capacity-1] == 0){
    heap[capacity-1] = capacity;
  }

  heap[capacity] = 0;*/
}

short hasLeft(int current, int min, int* heap){
  int possibility = 0;

  possibility = (int)((current - min) / 2);
  possibility = current - possibility;

  if(possibility == 0){
    return 0;
  } else {
    if(isNew(heap,possibility)){
      printf("\nLeft node possibilty confirmed! New left node = %i", possibility);
      return 1;
    } else {
      printf("\nLeft node possibilty denied! Attempted left node = %i", possibility);
      return 0;
    }
  }
}

int getLeft(int current, int min, int* heap){
  int possibility = 0;

  possibility = (int)((current - min) / 2);
  possibility = current - possibility;

  return possibility;
}


short hasRight(int current, int max, int* heap){
  double temp = 0;
  int possibility = 0;

  temp = ((double)max-(double)current)/2;
//  printf("\nTEMP %f\n", temp);

  possibility = (int)CEILING(temp);
  possibility += current;
//  printf("\npossibility %i\n", possibility);

  if(current == (max-1)){
    if(isNew(heap,max)){
      return 1;
    } else {
      return 0;
    }
  }

  if(possibility == 0){
    return 0;
  } else {
    if(isNew(heap,possibility)){
      printf("\nRight node possibilty confirmed! New right node = %i", possibility);
      return 1;
    } else {
      printf("\nRight node possibilty denied! Attempted right node = %i", possibility);
      return 0;
    }
  }
}

int getRight(int current, int max, int* heap){
  double temp = 0;
  int possibility = 0;

  temp = ((double)max-(double)current)/2;
  possibility = (int)CEILING(temp);
  possibility += current;

  if(current == max-1){
    if(isNew(heap,max)){
      return max;
    }
  }

  return possibility;
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
  for(unsigned short i = 0; heap[i] != 0; i++){
    printf("\ni = %i       heap[i] = %i", i, heap[i]);
  }
}

void zeroOutHeap(int* heap, short size){
  for(unsigned short i = 0; i < size; i++){
    heap[i] = 0;
  }
}


#endif
