#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <math.h>

/********************BEGIN GLOBAL VARIABLES********************/
#define TESTMODE (0)
/*General Variables*/
float v;                            //Most recent velocity (m/s).

/*BMP180 Variables*/
long padAlt;                        //The sea level (SL) altitude of the launchpad. (mm)
long y;                             //Most recent altitude (mm).
const int altN = 10;                //Number of Data Points per alt array.
unsigned long altTimes[altN];       //The n most recent times (ms).
long alts[altN];                    //The n most recent altitudesAGL (mm).

/*BNO055 Variables*/
const int accelN = 10;              //Number of Data Points per accel array.
unsigned long accelTimes[accelN];   //The n most recent times (ms).
float accel[accelN];                //The n most recent acceleration values.

/*GUI Variables*/
char response;                      //Holds the most recent char response from Serial
/*********************END GLOBAL VARIABLES*********************/



/********************CREATE BMP180 OBJECTS********************/
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
sensors_event_t event;
/*********************END BMP180 OBJECTS*********************/



/********************CREATE BMP180 OBJECTS********************/
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();
/*********************END BMP180 OBJECTS*********************/



/********************BEGIN FUNCTION PROTOTYPES********************/
/*General Functions*/
void newFlight(void);                 //Initiates files and variables for a new flight.
void initializeArrays(void);          //Fills arrays with zeros at setup.
void flightMode(void);                //Begins flightMode sequence.  Dependent on TESTMODE
void getData(void);                   //Attempts to retrieve data from sensors.
float calculateVelocity(void);        //Calculates velocity using alt from bmp180 and accel from BNO055.

/*GUI Functions*/
void handShake(void);                 //Initiates pairing with Java program.
void returnResponse(char);            //Returns received response from Java program with message stating what was received.

/*BMP180 Functions*/
long getAltitude(void);               //Finds current altitude using bmp180 sensor.
long getPadAlt(void);                 //Finds pad altitude using bmp180 sensor.
void updateTimesAlts(void);           //Updates time and altitude data from bmp180.

/*BNO055 Functions*/
float getAcceleration(void);          //TODO----FINISH THIS FUNCTION
void updateTimesAccel(void);          //Updates time and acceleration data from BNO055.
/*********************END FUNCTION PROTOTYPES*********************/



/********************BEGIN SETUP FUNCTION********************/
void setup(void) {  
  response = '\0';

  // start serial port at 9600 bps:
  Serial.begin(9600);

  //Confirm connection with Java program
  handShake();  // send a byte to establish contact until receiver responds


  //Initialize BMP180
  if(!bmp.begin()){
    Serial.print("Ooops, no BMP085 detected.... Check your wiring or I2C ADDR");
    while(1);
  }

  //Initialize BNO055
  if(!bno.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or 12C ADDR");
    while(1);
  }

  bno.setExtCrystalUse(true);
  newFlight();
}
/********************END SETUP FUNCTION********************/



/********************BEGIN LOOP FUNCTION********************/
void loop(void){
  if(Serial.available() >= 4){
    response = Serial.read();

    returnResponse(response);
    switch(response){
    case 'F':
      Serial.print("Entering flight mode...;");
      flightMode();
      break;
    case 'E':
      Serial.print("Case E;");
      break;
    case 'P':
      Serial.print(getAltitude());   
      Serial.flush();
      break;
    case 'S':
      break;
    default:
      break;
    }
  }
}
/*********************END LOOP FUNCTION*********************/



/********************BEGIN FUNCTION DEFINITIONS********************/
/*General Functions*/
void newFlight(void){
  initializeArrays();
  padAlt = getPadAlt();
} //END newFlight()


void initializeArrays(void){
  for(unsigned int i = 0; i < altN; i++){
    alts[i] = 0;
    altTimes[i] = 0;
  }

  for(unsigned int i = 0; i < accelN; i++){
    accel[i] = 0;
    accelTimes[i] = 0;
  }
} //END initializeArrays()


void flightMode(void){
  while(Serial.available()<=0) {
    getData();
    v = calculateVelocity();
    //kalmanFilter(alts[0],accel[0],v);
    //writeToFile(altTimes[0], alts[0])
  }
} //END flightMode(void)


void getData(void){
  #if TESTMODE
    //testMode code
//    getTimeAltAccel(); //TO DO: <----this function
  #else
    //FlightMode code
    //if(bmp180Ready){
      updateTimesAlts();
    //} else {
      //requestBMP180();
    //}
    updateTimesAccel();
  #endif
}


float calculateVelocity(void){
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
}// END calculateVelocity()


/*GUI Functions*/
void handShake() {
  while (Serial.available() <= 0) {
    Serial.write('~');   // send a capital A
    delay(300);
  }
} //END handShake()


void returnResponse(char response) {
  if(response == '~'){
      ;
  } else {
    Serial.print(response);
    Serial.print(" RECEIVED;");
    Serial.flush();
  }
} //END returnResponse()


/*BMP180 Functions*/
long getAltitude(void){  //USED TO CALCULATE CURRENT ALTITUDE
  bmp.getEvent(&event);
  if (event.pressure){
    return (long)1000 * (bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, event.pressure));
  } else {
    return NULL;
  }
}//END getAltitude()


long getPadAlt(void){  //USED TO CALCULATE PAD ALTITUDE
  long returnVal = getAltitude();
  delay(500);
  returnVal += getAltitude();
  delay(500);
  returnVal += getAltitude();
  delay(500);
  returnVal += getAltitude();
  delay(500);
  returnVal += getAltitude();
  returnVal /= 5;
  return returnVal;
}//END getPadAlt()


void updateTimesAlts(void){
  for (unsigned i = altN-1; i>0; i--){
    alts[i] = alts[i-1];
    altTimes[i] = altTimes[i-1];
  }
  alts[0] = getAltitude() - padAlt;
  altTimes[0] = millis();
} //END updateTimesAlts()


/*BNO055 Functions*/
void updateTimesAccel(void){
  for (unsigned i = accelN; i>0; i--){
    accel[i] = accel[i-1];
    accelTimes[i] = accelTimes[i-1];
  }
  //accel[0] = Reading from BNO055
  accelTimes[0] = millis();
} //END updateTimesAccel()

float getAcceleration(void){
  float result = 0;
  return result;
}
//TO DO:::: getAcceleration()
/*********************END FUNCTION DEFINITIONS*********************/
