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
void newFlight(void);                 //Initiates files and variables for a new flight
void initializeArrays(void);          //Fills arrays with zeros at setup.
void flightMode(void);                //Begins flightMode sequence.  Dependent on TESTMODE
void getData(void);                   //Attempts to retrieve data from sensors.
float calculateVelocity(void);         //Calculates velocity using alt from bmp180 and accel from BNO055

/*GUI Functions*/
void handShake(void);                 //Initiates pairing with Java program
void returnResponse(char);            //Returns received response from Java program with message stating what was received.

/*BMP180 Functions*/
long getAltitude(void);               //Finds current altitude using bmp180 sensor
long getPadAlt(void);                 //Finds pad altitude using bmp180 sensor
void updateTimesAlts(void);           //Updates time and altitude data from bmp180

/*BNO055 Functions*/
float getAcceleration(void);           //TODO----FINISH THIS FUNCTION
void updateTimesAccel(void);          //Updates time and acceleration data from BNO055
/*********************END FUNCTION PROTOTYPES*********************/




/* _____      _               
  / ____|    | |              
 | (___   ___| |_ _   _ _ __  
  \___ \ / _ \ __| | | | '_ \ 
  ____) |  __/ |_| |_| | |_) |
 |_____/ \___|\__|\__,_| .__/ 
                       | |    
                       |_|*/
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




/*__  __       _         _                       
 |  \/  |     (_)       | |                      
 | \  / | __ _ _ _ __   | |     ___   ___  _ __  
 | |\/| |/ _` | | '_ \  | |    / _ \ / _ \| '_ \ 
 | |  | | (_| | | | | | | |___| (_) | (_) | |_) |
 |_|  |_|\__,_|_|_| |_| |______\___/ \___/| .__/ 
                                          | |    
                                          |_| */
void loop(void){
  if(Serial.available() >= 4){ //why >= 4?
    response = Serial.read();

    returnResponse(response);
    switch(response){
    case 'B':
      Serial.print("Case B;");
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
/* _____                           _   ______                _   _                 
  / ____|                         | | |  ____|              | | (_)                
 | |  __  ___ _ __   ___ _ __ __ _| | | |__ _   _ _ __   ___| |_ _  ___  _ __  ___ 
 | | |_ |/ _ \ '_ \ / _ \ '__/ _` | | |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
 | |__| |  __/ | | |  __/ | | (_| | | | |  | |_| | | | | (__| |_| | (_) | | | \__ \
  \_____|\___|_| |_|\___|_|  \__,_|_| |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/*/

/**************************************************************************/
/*!
@brief  Prepares varaibles for new launch
Author: Jake
*/
/**************************************************************************/
void newFlight(void){
  initializeArrays();
  padAlt = getPadAlt();
} //END newFlight()


/**************************************************************************/
/*!
@brief  Initializes arrays to have values of 0 for a new flight.
Author: Jake
*/
/**************************************************************************/
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


/**************************************************************************/
/*!
@brief  Launch and test sequence.
Author: Jake
*/
/**************************************************************************/
void flightMode(void){
  while(Serial.available()<=0) {
    getData();
    v = calculateVelocity();
    //kalmanFilter(alts[0],accel[0],v);
    //writeToFile(altTimes[0], alts[0])
  }
} //END flightMode(void)


/**************************************************************************/
/*!
@brief  Gathers data from the desired source (Sensors or file).  Dependent on TESTMODE
Author: Jake
*/
/**************************************************************************/
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


/**************************************************************************/
/*!
@brief  Calculates a velocity value using altitude data from BMP180 and acceleration data fromm BNO055.
Author: Jake & Ben
  - Algorithm developed by Ben Stringer, function written by Jacob Cassady
*/
/**************************************************************************/
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




/*/$$$$$$  /$$   /$$ /$$$$$$       /$$$$$$$$                              /$$     /$$                              
 /$$__  $$| $$  | $$|_  $$_/      | $$_____/                             | $$    |__/                              
| $$  \__/| $$  | $$  | $$        | $$    /$$   /$$ /$$$$$$$   /$$$$$$$ /$$$$$$   /$$  /$$$$$$  /$$$$$$$   /$$$$$$$
| $$ /$$$$| $$  | $$  | $$        | $$$$$| $$  | $$| $$__  $$ /$$_____/|_  $$_/  | $$ /$$__  $$| $$__  $$ /$$_____/
| $$|_  $$| $$  | $$  | $$        | $$__/| $$  | $$| $$  \ $$| $$        | $$    | $$| $$  \ $$| $$  \ $$|  $$$$$$ 
| $$  \ $$| $$  | $$  | $$        | $$   | $$  | $$| $$  | $$| $$        | $$ /$$| $$| $$  | $$| $$  | $$ \____  $$
|  $$$$$$/|  $$$$$$/ /$$$$$$      | $$   |  $$$$$$/| $$  | $$|  $$$$$$$  |  $$$$/| $$|  $$$$$$/| $$  | $$ /$$$$$$$/
 \______/  \______/ |______/      |__/    \______/ |__/  |__/ \_______/   \___/  |__/ \______/ |__/  |__/|_______/ */
/**************************************************************************/
/*!
@brief  Initializes and confirms connection with Java program.
Author: Jake
*/
/**************************************************************************/
void handShake() {
  while (Serial.available() <= 0) {
    Serial.write('~');   // send a capital A
    delay(300);
  }
} //END handShake()

/**************************************************************************/
/*!
@brief  Returns a received response from the Java program to ensure successful delivery
Author: Jake
*/
/**************************************************************************/
void returnResponse(char response) {
  if(response == '~'){
      ;
  } else {
    Serial.print(response);
    Serial.print(" RECEIVED;");
    Serial.flush();
  }
} //END returnResponse()




/*____                 __  ___   ___    ______                _   _                 
 |  _ \               /_ |/ _ \ / _ \  |  ____|              | | (_)                
 | |_) |_ __ ___  _ __ | | (_) | | | | | |__ _   _ _ __   ___| |_ _  ___  _ __  ___ 
 |  _ <| '_ ` _ \| '_ \| |> _ <| | | | |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
 | |_) | | | | | | |_) | | (_) | |_| | | |  | |_| | | | | (__| |_| | (_) | | | \__ \
 |____/|_| |_| |_| .__/|_|\___/ \___/  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
                 | |                                                                
                 |_|                                                                
*/
/**************************************************************************/
/*!
@brief  calculates current altitude.
Will become legacy soon (11/10/16) as this function is slow. Will be replaced 
with a faster function
Author: Ben
*/
/**************************************************************************/
long getAltitude(void){  //USED TO CALCULATE CURRENT ALTITUDE
  bmp.getEvent(&event);
  if (event.pressure){
    return (long)1000 * (bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, event.pressure));
  } else {
    return NULL;
  }
}//END getAltitude()

/**************************************************************************/
/*!
@brief  calculates the ASL altitude of the launch pad
Author: Ben
*/
/**************************************************************************/
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

/**************************************************************************/
/*!
@brief  updates the array of altitude readings and the corresponding time readings
Author: Ben
*/
/**************************************************************************/
void updateTimesAlts(void){
  for (unsigned i = altN-1; i>0; i--){
    alts[i] = alts[i-1];
    altTimes[i] = altTimes[i-1];
  }
  alts[0] = getAltitude() - padAlt;
  altTimes[0] = millis();
} //END updateTimesAlts()




/*____               ___  _____ _____   ______                _   _                 
 |  _ \             / _ \| ____| ____| |  ____|              | | (_)                
 | |_) |_ __   ___ | | | | |__ | |__   | |__ _   _ _ __   ___| |_ _  ___  _ __  ___ 
 |  _ <| '_ \ / _ \| | | |___ \|___ \  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
 | |_) | | | | (_) | |_| |___) |___) | | |  | |_| | | | | (__| |_| | (_) | | | \__ \
 |____/|_| |_|\___/ \___/|____/|____/  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/*/
/**************************************************************************/
/*!
@brief  updates the array of acceleration readings and the corresponding time readings
Author: Jake
*/
/**************************************************************************/
void updateTimesAccel(void){
  for (unsigned i = accelN; i>0; i--){
    accel[i] = accel[i-1];
    accelTimes[i] = accelTimes[i-1];
  }
  //accel[0] = Reading from BNO055
  accelTimes[0] = millis();
} //END updateTimesAccel()

float getAcceleration(void){
  
}
//TO DO:::: getAcceleration()
/*********************END FUNCTION DEFINITIONS*********************/
