#include <math.h>

/********************BEGIN FUNCTION PROTOTYPES********************/
float calulateVelocity(void);         //Calculates velocity using alt from bmp180 and accel from BNO055
/*********************END FUNCTION PROTOTYPES*********************/



/********************BEGIN SETUP FUNCTION********************/
void setup(void) {
  Serial.begin(9600);
  
  for(unsigned int i = 0; i<altN; i++){
    altTimes[i] = (long)(i+1);
    alts[i] = (long)(i+1);
    accelTimes[i] = (long)(i+1);
    accel[i] = (float)(i+1);
  }
}
/********************END SETUP FUNCTION********************/



/********************BEGIN LOOP FUNCTION********************/
void loop(void){
  Serial.println(calulateVelocity());
}
/*********************END LOOP FUNCTION*********************/



/********************BEGIN FUNCTION DEFINITIONS********************/
float calulateVelocity(void){
  //VARIABLES NEEDED FOR CALULATION
  long sumBMPTimes = 0;
  long sumBMPTimes2 = 0;
  long sumAlt = 0;
  long sumAltTimes = 0;
  float leftSide = 0;
  float rightSide = 0;
  
  //FIND SUMS FOR BMP
  for(unsigned int i = 0; i < altN; i++){
    sumBMPTimes += altTimes[i];
    sumBMPTimes2 += pow(altTimes[i],2);
    sumAlt += alts[i];
    sumAltTimes += (alts[i] * altTimes[i]);
  }

  //CALCULATE LEFT SIDE OF EQUATION
  leftSide = (((float)sumBMPTimes * (float)sumAlt) - ((float)altN * (float)sumAltTimes)) / ((float)(pow(sumBMPTimes,2)) - ((float)altN * (float)sumBMPTimes2));  

  //CALCULATE RIGHT SIDE OF EQUATION
  for(unsigned int i = (accelN/2); i < (accelN - 1); i++){
    rightSide += (.5*(accel[i] + accel[(i+1)])*((float)accelTimes[(i+1)] - (float)accelTimes[i]));
  }

  return (leftSide + rightSide);
}
/*********************END FUNCTION DEFINITIONS*********************/
