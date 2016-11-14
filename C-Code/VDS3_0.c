#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <math.h>

#include "hashTagDefines.h" //All the VDS settings and constants are here
#include "RCR_Bmp180.h"     //Our own version of the pressure sensor library

/********************BEGIN GLOBAL VARIABLES********************/

/*BMP180 Variables*/
long padAlt;                         //The sea level (SL) altitude of the launchpad. (mm)
bool bmp180_init = false;			       //used to inform user that the bmp180 was not initialized succesfully
unsigned long altTimes[ALT_N];       //The n most recent times (ms).
long alts[ALT_N];                    //The n most recent altitudesAGL (mm).

/*BNO055 Variables*/
unsigned long accelTimes[ACCEL_N];   //The n most recent times (ms).
float accel[ACCEL_N];                //The n most recent acceleration values.
bool bno055_init = false;			       //used to inform user that the bno055 was not initialized succesfully

/*GUI Variables*/
char response;                       //Holds the most recent char response from Serial
/*********************END GLOBAL VARIABLES*********************/



/********************CREATE BMP180 OBJECTS********************/
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
/*********************END BMP180 OBJECTS*********************/



/********************CREATE BMP180 OBJECTS********************/
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();
/*********************END BMP180 OBJECTS*********************/



/********************CREATE FILE IO OBJECTS********************/
File data;                           //Holds file location
SdFatSdio sd;                        //Micro SD card object
/**********************END FILE IO OBJECTS*********************/



/********************BEGIN FUNCTION PROTOTYPES********************/
/*General Functions*/
void newFlight(void);                 //Initiates files and variables for a new flight
void initializeArrays(void);          //Fills arrays with zeros at setup.
void flightMode(void);                //Begins flightMode sequence.  Dependent on TESTMODE
void getData(void);                   //Attempts to retrieve data from sensors.
float calculateVelocity(void);        //Calculates velocity using alt from bmp180 and accel from BNO055

/*GUI Functions*/
void handShake(void);                 //Initiates pairing with Java program
void returnResponse(char);            //Returns received response from Java program with message stating what was received.

/*BMP180 Functions*/
long getAltitude(void);               //Finds current altitude using bmp180 sensor
long getPadAlt(void);                 //Finds pad altitude using bmp180 sensor
void updateTimesAlts(void);           //Updates time and altitude data from bmp180

/*BNO055 Functions*/
float getAcceleration(void);          //TODO----FINISH THIS FUNCTION
void updateTimesAccel(void);          //Updates time and acceleration data from BNO055

/*File IO Functions*/
void writeToFile(unsigned long, long, float, float, long, float, float, int);  //Writes data to file
                /*time, altitude,velocity,accel,kalman alt, kalman vel, kAccel, Error*/
void readFromFile(void);              //Retrieves past flight data for tests.  Replaces sensor functions
void resetNumber(char*);              //Resets (char)number array to NULL values.
float charToFloat(char);              //Converts a char number to a floating point value
float numToFloat(char*);              //Converts a char array representing a number into a floating point value.
                                      //Handles certain forms of scientific notation.
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


  // start serial port at 115200 bps:
  Serial.begin(115200);

  //Confirm connection with Java program
  //handShake();  // send a byte to establish contact until receiver responds

  //Initialize BMP180
  if(!bmp.begin()){
	  Serial.println("NO Bmp180 DETECTED!");
    //while(1);
  }
  else {
	  bmp180_init = true;
  }

  //Initialize BNO055
  if(!bno.begin()){
    Serial.println("NO Bno055 DETECTED!");
    //while(1);
  }
  else {
	  bno055_init = true;
  }

  bno.setExtCrystalUse(true);

  //Initialize SD card
  if(!sd.begin()){
    Serial.println("SD card initialization failed!");
  }
  
  newFlight();

  //State what debug levels are activated
#if DEBUG_NORMAL
  Serial.println("DEBUG_NORMAL");
#endif
#if DEBUG_ALPHA
  Serial.println("DEBUG_ALPHA");
#endif
#if DEBUG_BRAVO
  Serial.println("DEBUG_BRAVO");
#endif
#if TEST_MODE
  Serial.println("TEST_MODE!");
#endif
  delay(2500);
  Serial.println("now entering menu");
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
  if(Serial.available()){ 
	// response = Serial.read();

    //returnResponse(response);
    switch(Serial.read()){
    case 'B':
		Serial.println("Case B;");
		eatYourBreakfast();
		break;
    case 'e':
		 Serial.println("Case e;");
		eatYourBreakfast();
		break;
    case 'P':
		//Serial.print(getAltitude());   //change
		eatYourBreakfast();
		break;
    case 'S':
		break;
	case 'F':
		Serial.println("Entering Flight Mode");
		eatYourBreakfast();
		flightMode();
		break;
    default:
		Serial.println("Unkown code received");
		Serial.println(response);
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
  @brief  Launch and test sequence.
  Author: Jake
  */
  /**************************************************************************/
void flightMode(void) {
	float alt = 0;				//most recent altitude reading
	float vel = 0;                   //Most recent velocity (m/s).
	long timer = 0;

	while (!Serial.available()) {

#if TEST_MODE
		Serial.println("");
		Serial.println("TEST_MODE!");
		Serial.println("");
#endif
		if (!bmp180_init) {
			Serial.println("WARNING: bmp180 not initialized!");
		}
		if (!bno055_init) {
			Serial.println("WARNING: bno055 not initialized!");
		}

		Serial.print("dt=");
		timer = millis();
		altitude_plz(&alt); //gets the altitude if it's ready and puts it in alt
		Serial.println(millis() - timer);

#if DEBUG_NORMAL
		Serial.print(alt);
		Serial.print(" at t = ");
		Serial.print(millis());
		Serial.println(" ");
#endif

		//getData();
		//v = calculateVelocity();
		//kalmanFilter(alts[0],accel[0],v);
		//writeToFile(altTimes[0], alts[0])
	}
	//if some serial input ~= to the standdown code or 1 second passes, call flightmode again...  need to discuss
} //END flightMode(void)

/**************************************************************************/
/*!
@brief  Prepares varaibles for new launch
Author: Jake
*/
/**************************************************************************/
void newFlight(void){
  initializeArrays();

  sd.remove("VDS2_0.dat");

  File data = sd.open("VDS2_0.dat", FILE_WRITE);
  if(!data){
    Serial.println("Data file unable to initiated."); 
  } else {
    data.println("t(ms), vel(m/s), accel(m/s^2), kalman altitude(m), kalman velocity(m/s), kalman acceleration(m/s^2), Error Code");
    data.close();
  }
//  padAlt = getPadAlt(); //change

} //END newFlight()


/**************************************************************************/
/*!
@brief  Initializes arrays to have values of 0 for a new flight.
Author: Jake
*/
/**************************************************************************/
void initializeArrays(void){
  for(unsigned int i = 0; i < ALT_N; i++){
    alts[i] = 0;
    altTimes[i] = 0;
  }

  for(unsigned int i = 0; i < ACCEL_N; i++){
    accel[i] = 0;
    accelTimes[i] = 0;
  }
} //END initializeArrays()





/**************************************************************************/
/*!
@brief  Gathers data from the desired source (Sensors or file).  Dependent on TESTMODE
Author: Jake
*/
/**************************************************************************/
void getData(void){
  #if TEST_MODE
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
  for(unsigned int i = 0; i < ALT_N; i++){
    sumBMPTimes += altTimes[i];
    sumBMPTimes2 += pow(altTimes[i],2);
    sumAlt += alts[i];
    sumAltTimes += (alts[i] * altTimes[i]);
  }

  //CALCULATE LEFT SIDE OF EQUATION
  leftSide = (((float)sumBMPTimes * (float)sumAlt) - ((float)ALT_N * (float)sumAltTimes)) / ((float)(pow(sumBMPTimes,2)) - ((float)ALT_N * (float)sumBMPTimes2));  

  //CALCULATE RIGHT SIDE OF EQUATION
  for(unsigned int i = (ACCEL_N /2); i < (ACCEL_N - 1); i++){
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
  @brief  Clears the serial buffer.. This 
  is helpful for carriage returns and things of that sort that 
  hang around after you got what you wanted.
  Author: Ben
  */
  /**************************************************************************/
void eatYourBreakfast() {
	while (Serial.available() > 0) {
		delay(2);
		Serial.read();
	}
}

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
@brief  Checks if Bmp180 has a reading ready, retrieves reading and requests a new readings if yes, returns false if not ready yet
Pronounced "altitude please".
Author: Ben
*/
/**************************************************************************/
bool altitude_plz(float *altitude) {
	float pressure_kPa;
	float pressure_; //units are Pa*10?

	if (bmp.RCR_readyYet()) {
		bmp.RCR_getPressure(&pressure_kPa);  //picks up the pressure reading from the Bmp180, then puts in a request for a new one
#if DEBUG_ALPHA
		Serial.print("RCR_getPressure returned: ");
		Serial.print(pressure_kPa);
		Serial.print("  kPa at t = ");
		Serial.println(millis());
#endif
		pressure_ = pressure_kPa / 100.0F;
		*altitude = (bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, pressure_));
		return true;
	}
	else {
		return false;
	}
}


/**************************************************************************/
/*!
@brief  updates the array of altitude readings and the corresponding time readings
Author: Ben
*/
/**************************************************************************/
void updateTimesAlts(void){
  for (unsigned i = ALT_N-1; i>0; i--){
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
  for (unsigned i = ACCEL_N; i>0; i--){
    accel[i] = accel[i-1];
    accelTimes[i] = accelTimes[i-1];
  }
  //accel[0] = Reading from BNO055
  accelTimes[0] = millis();
} //END updateTimesAccel()

float getAcceleration(void){
  
}
//TO DO:::: getAcceleration()



/*           ,,    ,,                                                                                     ,,                             
`7MM"""YMM db  `7MM              `7MMF' .g8""8q.       `7MM"""YMM                                mm     db                             
  MM    `7       MM                MM .dP'    `YM.       MM    `7                                MM                                    
  MM   d `7MM    MM  .gP"Ya        MM dM'      `MM       MM   d `7MM  `7MM  `7MMpMMMb.  ,p6"bo mmMMmm `7MM  ,pW"Wq.`7MMpMMMb.  ,pP"Ybd 
  MM""MM   MM    MM ,M'   Yb       MM MM        MM       MM""MM   MM    MM    MM    MM 6M'  OO   MM     MM 6W'   `Wb MM    MM  8I   `" 
  MM   Y   MM    MM 8M""""""       MM MM.      ,MP       MM   Y   MM    MM    MM    MM 8M        MM     MM 8M     M8 MM    MM  `YMMMa. 
  MM       MM    MM YM.    ,       MM `Mb.    ,dP'       MM       MM    MM    MM    MM YM.    ,  MM     MM YA.   ,A9 MM    MM  L.   I8 
.JMML.   .JMML..JMML.`Mbmmd'     .JMML. `"bmmd"'       .JMML.     `Mbod"YML..JMML  JMML.YMbmd'   `Mbmo.JMML.`Ybmd9'.JMML  JMML.M9mmmP' */
/**************************************************************************/
/*!
@brief  Writes data to a file
Author: Jake
*/
/**************************************************************************/
void writeToFile(unsigned long t, long alt, float vel, float accel, long kAlt, float kVel, float kAccel, int error){
  File myFile = sd.open("test.dat", FILE_WRITE);

  if(myFile) {
    Serial.print("Writing data to test.dat;");
  } else {
    Serial.print("Unable to open test.dat;");
  }
    myFile.print(t);
    myFile.print(",");
    myFile.print(alt);
    myFile.print(",");
    myFile.print(vel);
    myFile.print(",");
    myFile.print(accel);
    myFile.print(",");
    myFile.print(kAlt);
    myFile.print(",");
    myFile.print(kVel);
    myFile.print(",");
    myFile.print(kAccel);
    myFile.print(",");
    myFile.print(error);
    myFile.println("");
    myFile.close();
} //END writeToFile()


/**************************************************************************/
/*!
@brief  Retrieves past flight data for tests.  Replaces sensor functions
Author: Jake
*/
/**************************************************************************/
void readFromFile(void){
  File myFile = sd.open("8_6_16_launch.dat", FILE_READ);
  char place = '\n';
  char number[20] = {NULL};
  short numPlace = 0, numCount = 0;
  float value = 0;
  int lineCount = 0;
  static int linePlaceHolder = 0;

  if(myFile){
    while(myFile.available()) {
      place = myFile.read();

      if(isdigit(place)){
        number[numPlace] = place;
      } else if (place == '.') {
        number[numPlace] = place;
      } else if (place == '-'){
        number[numPlace] = place;
      } else if (place == 'e' || place == '+'){
        number[numPlace] = place;
      } else if (place == ','){
        value = numToFloat(number);
        numCount++;
        numPlace = -1;
        switch(numCount-(lineCount*3)){
        case 1:
          vel = value;
          break;
          
        case 2:
          alt = value;
          break;
        }
        resetNumber(number);
      } else {
        value = numToFloat(number);
        numCount++;
        numPlace = -1;
        resetNumber(number);
        lineCount++;
        accel = value;
      }

      if(numCount == 0) {
        
      } else {
        if((numCount % ((linePlaceHolder+1)*3)) == 0){
          Serial.print("Velocity: ");
          Serial.print(vel);
          Serial.print(";");
          Serial.print("Altitude: ");
          Serial.print(alt);
          Serial.print(";");
          Serial.print("Acceleration: ");
          Serial.print(accel);
          Serial.print(";");
          linePlaceHolder++;
          break;
        }
      }
      numPlace++;
      
    }

    myFile.close();
  } else {
    Serial.print("error opening the text file!;");
  }
} //END readFromFile();


/**************************************************************************/
/*!
@brief  Resets (char)number array to NULL values.
Author: Jake
*/
/**************************************************************************/
void resetNumber(char* number){
  for(short i = 0; i<20; i++){
    number[i] = NULL;
  }
} //END resetNumber();


/**************************************************************************/
/*!
@brief  Converts a char number to a floating point value
Author: Jake
*/
/**************************************************************************/
float charToFloat(char input){
  int temp = input - '\0';
  temp -= 48;
  return float(temp);
} //END charToFloat();


/**************************************************************************/
/*!
@brief  Converts a char array representing a number into a floating point value.
        Handles certain forms of scientific notation.
Author: Jake
*/
/**************************************************************************/
float numToFloat(char* number){
    short index = 0, decimalIndex = 0;
  boolean decimal = false, e = false, negative = false, one = false;
  float result = 0, temp = 0;

  while(number[index] != NULL){
    if(number[index] == 'e'){
      e = true;
    } else if (number[index] == '.'){
      decimal = true;
    } else if (number[index] == '-'){
      negative = true;
    } else if (number[index] == '+'){
      ;
    } else {
      temp = charToFloat(number[index]);
      if(e){
        result *= (float)pow(10,temp);
      } else if (decimal) {
        temp *= pow(10,(-1*(decimalIndex+1)));
        result += temp;
        decimalIndex++;
      } else {
        if(one){
          result *= 10;
        } else {
           one = true;
        }
        result += temp;
      }
    }
    index++;
  }
  if(negative){
      result *= -1;
  }
  return result;
} //END numToFloat();
/*********************END FUNCTION DEFINITIONS*********************/
