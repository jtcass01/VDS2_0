#ifndef __HEAP_H
#define __HEAP_H
#include <stdlib.h>
#include <stdio.h>

#define CEILING_POS(X) ((X-(int)(X)) > 0 ? (int)(X+1) : (int)(X))
#define CEILING_NEG(X) ((X-(int)(X)) < 0 ? (int)(X-1) : (int)(X))
#define CEILING(X) ( ((X) > 0) ? CEILING_POS(X) : CEILING_NEG(X) )

void zeroOutHeap(int*,short);
void getBinaryHeap(int*, short);
short hasLeft(int*, int, int);
int getLeft(int*, int, int);
short hasRight(int*, int, int);
int getRight(int*, int, int);
short isNew(int*, int);
int getNextLower(int*, int, int);
int getNextUpper(int*, short, int, int);
int checkRepresentation(int*, short);


void zeroOutHeap(int* heap, short size){
  for(unsigned short i = 0; i <= size; i++){
    heap[i] = 0;
  }
}


void getBinaryHeap(int* heap, short capacity){
  int mid = capacity / 2, i = 1, j=0, nextLower = 0, nextUpper = 0;
  zeroOutHeap(heap,capacity);

  heap[0] = mid;

  for(;i<capacity;){
    printf("\n\nNode being tested: %i     j = %i", heap[j], j);

    nextLower = getNextLower(heap, heap[j], j);
    nextUpper = getNextUpper(heap, capacity, heap[j], j);

    if(heap[j] < mid){
      if(hasLeft(heap, heap[j], nextLower)){
        printf("\nInserting %i at leftNode.     i = %i     j = %i", getLeft(heap, heap[j], nextLower), i, j);
        //2*j+1
        heap[i] = getLeft(heap, heap[j], nextLower);
        i++;
      }
      if(hasRight(heap, heap[j], nextUpper)){
        printf("\nInserting %i at rightNode.     i = %i     j = %i", getRight(heap, heap[j], nextUpper), i, j);
        //2*j+2
        heap[i] = getRight(heap, heap[j], nextUpper);
        i++;
      }
    } else if (heap[j] > mid){
      if(hasLeft(heap, heap[j], nextLower)){
        printf("\nInserting %i at leftNode.     i = %i     j = %i", getLeft(heap, heap[j], nextLower), i, j);
        heap[i] = getLeft(heap, heap[j], nextLower);
        i++;
      }
      if(hasRight(heap, heap[j],nextUpper)){
        printf("\nInserting %i at rightNode.     i = %i     j = %i", getRight(heap, heap[j],nextUpper), i, j);
        heap[i] = getRight(heap, heap[j],nextUpper);
        i++;
      }
    } else {
      if(hasLeft(heap, mid, nextLower)){
        heap[i] = getLeft(heap, mid, nextLower);
        i++;
      }
      if(hasRight(heap, mid, nextUpper)){
        heap[i] = getRight(heap, mid, nextUpper);
        i++;
      }
    }
    j++;
  }

  if(checkRepresentation(heap, capacity)){
    printf("\nRepresentation checked.  All position values are included in binary heap.");
  } else {
    printf("\nRepresentation check failed.  Find programming error!");
  }

/*  if(heap[capacity-1] == 0){
    heap[capacity-1] = capacity;
  }

  heap[capacity] = 0;*/
}

short hasLeft(int* heap, int current, int min){
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

int getLeft(int* heap, int current, int min){
  int possibility = 0;

  possibility = (int)((current - min) / 2);
  possibility = current - possibility;

  return possibility;
}


short hasRight(int* heap, int current, int max){
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

int getRight(int* heap, int current, int max){
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

void printHeap(int* heap){
  printf("\n\n----- PRINTING HEAP ------\n");
  for(unsigned short i = 0; heap[i] != 0; i++){
    printf("\nheap[%i] = %i", i, heap[i]);
  }
}


int getNextLower(int* heap, int current, int location){
  int spot = 0;

  printf("\n\ngetNextLower()\nCurrent: %i     Location: %i", heap[location], location);

  spot = location;

  while(spot > 0){
    spot -= 1;
    spot /= 2;

    printf("\nNew location being tested: %i      value: %i", spot, heap[spot]);

    if(heap[spot] < current){
      printf("\nLower being returned: %i\n", heap[spot]);
      return heap[spot];
    }
  }

  printf("\nLower being returned: %i\n", 0);
  return 0;
}


int getNextUpper(int* heap, short capacity, int current, int location){
  int spot = 0;

  printf("\n\ngetNextUpper()\nCapacity: %i      Current: %i     Location: %i", capacity, heap[location], location);

  spot = location;

  while(spot > 0){
    spot -= 1;
    spot /= 2;

    printf("\nNew location: %i      value: %i", spot, heap[spot]);

    if(heap[spot] > current){
      printf("\nUpper being returned: %i\n", heap[spot]);
      return heap[spot];
    }
  }

  printf("\nUpper being returned: %i\n", capacity);
  return capacity;
}


int checkRepresentation(int* heap, short capacity){
  int temp[capacity];
  unsigned short heap_i = 0;

  for(unsigned short temp_i =1; temp_i <= capacity; temp_i++)
    temp[temp_i-1] = temp_i;

  for(; heap_i < capacity; ){
    for(unsigned short heap_j = 0; heap_j < capacity; ){
      if(heap[heap_i] == temp[heap_j]){
        heap_i++;
        break;
      } else {
        heap_j++;
      }

      if(heap_j == (capacity - 1) && heap[heap_j] != temp[heap_j]){
        return 0;
      }
    }
  }
  return 1;
}

#endif
