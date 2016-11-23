#include <math.h>



/********************BEGIN FUNCTION PROTOTYPES********************/
void handShake(void);
float calulateVelocity(struct stateStruct);         //Calculates velocity using alt from bmp180 and accel from BNO055

void copyState(struct stateStruct* original, struct stateStruct* destination);
void initializePastStates(void);
void printState(struct stateStruct, int);
void printPastStates(struct stateStruct*);
/*********************END FUNCTION PROTOTYPES*********************/

/******************** VARIABLES ********************/
const int BUFF_N = 10;

struct stateStruct {
  float alt;
  float vel;
  float accel;
  float time;
};

struct stateStruct pastRawStates[BUFF_N];

struct stateStruct additionalStruct;
/****************** END VARIABLES ******************/


/********************BEGIN SETUP FUNCTION********************/
void setup(void) {
  Serial.begin(9600);

  additionalStruct.alt = 1;
  additionalStruct.vel = 1;
  additionalStruct.accel = 1;
  additionalStruct.time = 1;

  handShake();

  initializePastStates();
  
  printPastStates(pastRawStates);

  //copyState(&pastRawStates[0],&pastRawStates[1]);

  //printPastStates(pastRawStates);

  Serial.print(calculateVelocity(additionalStruct));
  Serial.println(" Vel;");
}
/********************END SETUP FUNCTION********************/



/********************BEGIN LOOP FUNCTION********************/
void loop(void){
//  Serial.println(calulateVelocity());
}
/*********************END LOOP FUNCTION*********************/



/********************BEGIN FUNCTION DEFINITIONS********************/
void handShake() {
  while (Serial.available() <= 0) {
    Serial.write('~');   // send a capital A
    delay(300);
  }
} //END handShake()

//float calulateVelocity(void){
//  //VARIABLES NEEDED FOR CALULATION
//  long sumBMPTimes = 0;
//  long sumBMPTimes2 = 0;
//  long sumAlt = 0;
//  long sumAltTimes = 0;
//  float leftSide = 0;
//  float rightSide = 0;
//  
//  //FIND SUMS FOR BMP
//  for(unsigned int i = 0; i < altN; i++){
//    sumBMPTimes += altTimes[i];
//    sumBMPTimes2 += pow(altTimes[i],2);
//    sumAlt += alts[i];
//    sumAltTimes += (alts[i] * altTimes[i]);
//  }
//
//  //CALCULATE LEFT SIDE OF EQUATION
//  leftSide = (((float)sumBMPTimes * (float)sumAlt) - ((float)altN * (float)sumAltTimes)) / ((float)(pow(sumBMPTimes,2)) - ((float)altN * (float)sumBMPTimes2));  
//
//  //CALCULATE RIGHT SIDE OF EQUATION
//  for(unsigned int i = (accelN/2); i < (accelN - 1); i++){
//    rightSide += (.5*(accel[i] + accel[(i+1)])*((float)accelTimes[(i+1)] - (float)accelTimes[i]));
//  }
//
//  return (leftSide + rightSide);
//}

float calculateVelocity(struct stateStruct rawState)   { //VARIABLES NEEDED FOR CALULATION
  //static struct stateStruct pastRawStates[BUFF_N];
  float sumBMPTimes = 0, sumBMPTimes2 = 0, sumAlt = 0, sumAltTimes = 0, leftSide = 0;
  float rightSide = 0, numer = 0, denom=0;

  //shift new readings into arrays   
  for (uint8_t i = (BUFF_N-1); i > 0; i--) {
    copyState(&pastRawStates[i],&pastRawStates[i-1]);
//    printState(pastRawStates[i], i);
//    Serial.println(i);
  }
  printPastStates(pastRawStates);

  copyState(&pastRawStates[0],&rawState);

  printPastStates(pastRawStates);

  //FIND SUMS FOR BMP
  for (unsigned int i = 0; i < BUFF_N; i++) {
    sumBMPTimes += pastRawStates[i].time;
    sumBMPTimes2 += pastRawStates[i].time * pastRawStates[i].time;
    sumAlt += pastRawStates[i].alt;
    sumAltTimes += (pastRawStates[i].alt * pastRawStates[i].alt);
  }

  //CALCULATE LEFT SIDE OF EQUATION
  numer = ((sumBMPTimes * sumAlt) - (BUFF_N * sumAltTimes));
  denom = (sumBMPTimes*sumBMPTimes - (BUFF_N * sumBMPTimes2));
  leftSide = numer / denom;

  //CALCULATE RIGHT SIDE OF EQUATION
  for (unsigned int i = (BUFF_N / 2); i < (BUFF_N - 1); i++) {
    rightSide += (.5*(pastRawStates[i].accel + pastRawStates[i + 1].accel)*((float)pastRawStates[i + 1].time - (float)pastRawStates[i].time));
  }

  return leftSide+rightSide;
}

void initializePastStates(void){
  for(unsigned int i = 0; i<BUFF_N; i++){
    pastRawStates[i].alt = (float)(0);
    pastRawStates[i].vel = (float)(0);
    pastRawStates[i].accel = (float)(0);
    pastRawStates[i].time = (float)(0);
  }
}

void copyState(struct stateStruct* destination, struct stateStruct* original){
  destination->alt = original->alt;
  destination->vel = original->vel;
  destination->accel = original->accel;
  destination->time = original->time;
}


void printPastStates(struct stateStruct* pastStates) {
  for(int i = 0; i < BUFF_N; i++){
    printState(pastStates[i], i);
  }
}


void printState(struct stateStruct state, int label) {
  Serial.print(label);
  Serial.print(") alt = ");
  Serial.print(state.alt);
  Serial.print(", vel = ");
  Serial.print(state.vel);
  Serial.print(", accel = ");
  Serial.print(state.accel);
  Serial.print(", time = ");
  Serial.print(state.time);
  Serial.println(");");
} //End printState()

/*********************END FUNCTION DEFINITIONS*********************/
