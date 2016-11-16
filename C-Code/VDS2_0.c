#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <math.h>

#include "hashTagDefines.h" //All the VDS settings and constants are here
#include "RCR_Bmp180.h"     //Our own version of the pressure sensor library
#include "MatrixMath.h"

struct stateStruct {
	float alt;
	float vel;
	float accel;
	unsigned long time;
};

/********************BEGIN GLOBAL VARIABLES********************/


/*BMP180 Variables*/
long padAlt;                        //The sea level (SL) altitude of the launchpad. (mm)
bool bmp180_init = false;			      //used to inform user that the bmp180 was not initialized succesfully

/*BNO055 Variables*/
bool bno055_init = false;			      //used to inform user that the bno055 was not initialized succesfully

/*GUI Variables*/
char response;                      //Holds the most recent char response from Serial

/*Kalman variables*/
float q_k[3][3] = {
	{ 1, 0, 0 },
	{ 0, 0.02, 0 },
	{ 0, 0, 0.2 }
};
float r_k[3][3] = {
	{ 0.5, 0, 0 },
	{ 0, 4, 0 },
	{ 0, 0, 7 }
};

//encoder
volatile uint8_t encPos = 0;

/*********************END GLOBAL VARIABLES*********************/

//temporary!!
//float *state;// = (float*)malloc(12); //how do I allocate 3*32 bits of space for this variable?
//float z_k_1[3] = { 0.3974, 6.7814, 102.8645 };
//float z_k_2[3] = { 4.7064,31.5675, 110.7171 };
//float z_k_3[3] = { 15.9803,61.2562, 138.1404 };
struct stateStruct filteredState_test;
struct stateStruct z_k_1;
struct stateStruct z_k_2;
struct stateStruct z_k_3;

/********************CREATE BMP180 OBJECTS********************/
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
/*********************END BMP180 OBJECTS*********************/

/********************CREATE BMP180 OBJECTS********************/
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();
/*********************END BMP180 OBJECTS*********************/

/********************CREATE FILE IO OBJECTS********************/
File data;                                          //Stores file object
SdFatSdio sd;                                       //Micro SD card object
/**********************END FILE IO OBJECTS*********************/

/********************BEGIN FUNCTION PROTOTYPES********************/
/*General Functions*/
void newFlight(void);                               //Initiates files and variables for a new flight
void initializeArrays(void);                        //Fills arrays with zeros at setup.
void flightMode(void);                              //Begins flightMode sequence.  Dependent on TESTMODE
struct stateStruct getRawState(void);               //Retrieves data from sensors.
float calculateVelocity(struct stateStruct*);       //Calculates velocity using alt from bmp180 and accel from BNO055

/*GUI Functions*/
void handShake(void);                               //Initiates pairing with Java program
void returnResponse(char);                          //Returns received response from Java program with message stating what was received.

/*BMP180 Functions*/
bool altitude_plz(float*);                          //Checks if Bmp180 has a reading ready, retrieves reading and requests a new readings
                                                    //if yes, returns false if not ready yet
long getAltitude(void);                             //Finds current altitude using bmp180 sensor
long getPadAlt(void);                               //Finds pad altitude using bmp180 sensor

/*BNO055 Functions*/
float getAcceleration(void);                        //TODO----FINISH THIS FUNCTION

/*Kalman Functions*/
struct stateStruct kalman(int16_t, struct stateStruct);    //Filters the state of the vehicle

/*File IO Functions*/
void writeToFile(struct stateStruct, struct stateStruct);  //Writes data to file
struct stateStruct readFromFile(void);              //Retrieves past flight data for tests.  Replaces sensor functions
void resetNumber(char*);                            //Resets (char)number array to NULL values.
float charToFloat(char);                            //Converts a char number to a floating point value
float numToFloat(char*);                            //Converts a char array representing a number into a floating point value.
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

	pinMode(LED, OUTPUT);
	
	//temporary!!
	z_k_1.accel = 102.8645;
	z_k_1.vel = 6.7814;
	z_k_1.alt = 0.3974;
	z_k_1.time = 128;
	z_k_2.accel = 110.7171;
	z_k_2.vel = 31.5675;
	z_k_2.alt = 4.7064;
	z_k_2.time = 128+ 226;
	z_k_3.accel = 138.1404;
	z_k_3.vel = 61.2562;
	z_k_3.alt = 15.9803;
	z_k_3.time = 128 + 226 + 245;

	// start serial port at //115200 bps:
	Serial.begin(38400);

	//Confirm connection with Java program
	//handShake();  // send a byte to establish contact until receiver responds

	//Initialize BMP180
	if (!bmp.begin()) {
		Serial.println("NO Bmp180 DETECTED!");
	}
	else {
		bmp180_init = true;
	}

	//Initialize BNO055
	if (!bno.begin()) {
		Serial.println("NO Bno055 DETECTED!");
	}
	else {
		bno055_init = true;
		bno.setExtCrystalUse(true);
	}

  //Initialize SD card
  if(!sd.begin()){
    Serial.println("SD card initialization failed!");
  }
  	
	newFlight();
	digitalWrite(LED, HIGH); //turn on on-board LED

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

	Serial.println("-------Menu-------");
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
void loop(void) {
	if (Serial.available() > 0) {
		// response = Serial.read();

		//returnResponse(response);
		switch (Serial.read()) {
		case 'B':
			Serial.println("Case B;");
			eatYourBreakfast();
			break;
		case 'e':
			Serial.println("Case e;");
			eatYourBreakfast();
			break;
		case 'k':
			Serial.println("kalmaning");
			filteredState_test = kalman(0, z_k_1);
			printState(filteredState_test, "test 1");
			filteredState_test = kalman(0, z_k_2);
			printState(filteredState_test, "test 2");
			filteredState_test = kalman(0, z_k_3);
			printState(filteredState_test, "test 3");
			eatYourBreakfast();
			break;
		case 'S':
			break;
		case 'f':
			eatYourBreakfast();
			if ((!bmp180_init || !bno055_init) && !TEST_MODE) {
				Serial.println("Cannot enter flight mode. A sensor is not initialized.");
			}
			else {
				Serial.println("Entering Flight Mode");
				flightMode();
			}
			break;
		default:
			Serial.println("Unkown code received");
			Serial.println(response);
			break;
		}
		Serial.println("-------Menu-------");
	}
}
/*********************END LOOP FUNCTION*********************/




/********************BEGIN FUNCTION DEFINITIONS********************/
/* 
  ______ _ _       _     _     __  __           _        ______                _   _                 
 |  ____| (_)     | |   | |   |  \/  |         | |      |  ____|              | | (_)                
 | |__  | |_  __ _| |__ | |_  | \  / | ___   __| | ___  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___ 
 |  __| | | |/ _` | '_ \| __| | |\/| |/ _ \ / _` |/ _ \ |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
 | |    | | | (_| | | | | |_  | |  | | (_) | (_| |  __/ | |  | |_| | | | | (__| |_| | (_) | | | \__ \
 |_|    |_|_|\__, |_| |_|\__| |_|  |_|\___/ \__,_|\___| |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
              __/ |                                                                                  
             |___/ */

/**************************************************************************/
/*!
@brief  Launch and test sequence.
Author: Jake
*/
/**************************************************************************/
void flightMode(void) {
	struct stateStruct rawState, filteredState;

	while (Serial.available() == 0) {

#if TEST_MODE 
		Serial.println("");
		Serial.println("TEST_MODE!");
		Serial.println("");
#endif
		//just in case
		if (!bmp180_init) {
			Serial.println("WARNING: bmp180 not initialized!");
		}
		if (!bno055_init) {
			Serial.println("WARNING: bno055 not initialized!");
		}

		rawState = getRawState();
		filteredState = kalman(encPos, rawState);
		writeToFile(rawState, filteredState)
	}
	//if some serial input ~= to the standdown code or 1 second passes, call flightmode again...  need to discuss
} //END flightMode(void)


  /**************************************************************************/
  /*!
  @brief  Gathers data from the desired source (Sensors or file).  Dependent on TEST_MODE
  Author: Jake
  */
  /**************************************************************************/
struct stateStruct getRawState(void) {
	static struct stateStruct rawState; //needs to be static because of the altitude reading (ask Ben)
#if TEST_MODE
//testMode code
rawState = readFromFile();

#else

//get raw altitude
	altitude_plz(&rawState.alt);
	rawState.alt -= padAlt;

	//get time
	rawState.time = millis();

	//get raw acceleration
	//rawState.rawAccel = acceleration_plz;

	printState(rawState, "raw state");

#endif

	//calculate velocity
	calculateVelocity(&rawState);

	return rawState;
}


/**************************************************************************/
/*!
@brief  Calculates a velocity value using altitude data from BMP180 and acceleration data fromm BNO055.
Author: Jake & Ben
- Algorithm developed by Ben Stringer, function written by Jacob Cassady
*/
/**************************************************************************/
void calculateVelocity(struct stateStruct *rawState) { //TODO: fix units and variable types
													   //VARIABLES NEEDED FOR CALULATION
	static struct stateStruct pastRawStates[ALT_N];
	long sumBMPTimes = 0;
	long sumBMPTimes2 = 0;
	long sumAlt = 0;
	long sumAltTimes = 0;
	float leftSide = 0;
	float rightSide = 0;

	//shift new readings into arrays	 
	for (uint8_t i = ALT_N; i > 0; i--) {
		pastRawStates[i] = pastRawStates[i - 1];
	}
	pastRawStates[0] = *rawState;

	//FIND SUMS FOR BMP
	for (unsigned int i = 0; i < ALT_N; i++) {
		sumBMPTimes += pastRawStates[i].time;
		sumBMPTimes2 += pow(pastRawStates[i].time, 2);
		sumAlt += pastRawStates[i].alt;
		sumAltTimes += (pastRawStates[i].alt * pastRawStates[i].alt);
	}

	//CALCULATE LEFT SIDE OF EQUATION
	leftSide = (((float)sumBMPTimes * (float)sumAlt) - ((float)ALT_N * (float)sumAltTimes)) / ((float)(pow(sumBMPTimes, 2)) - ((float)ALT_N * (float)sumBMPTimes2));

	//CALCULATE RIGHT SIDE OF EQUATION
	for (unsigned int i = (ACCEL_N / 2); i < (ACCEL_N - 1); i++) {
		rightSide += (.5*(pastRawStates[i].accel + pastRawStates[i + 1].accel)*((float)pastRawStates[i + 1].time - (float)pastRawStates[i].time));
	}

	rawState->vel = (leftSide + rightSide);
}// END calculateVelocity()



  /**************************************************************************/
  /*!
  @brief  Prepares varaibles for new launch
  Author: Jake
  */
  /**************************************************************************/
void newFlight(void) {
  sd.remove("VDSv2FlightData.dat");

  File data = sd.open("VDSv2FlightData.dat", FILE_WRITE);
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
  @brief  WILL REPLACE WITH THE INITIALIZATION OF DENNYS LISTS
  Author: Jake
  */
  /**************************************************************************/
  //void initializeArrays(void){
  //  for(unsigned int i = 0; i < ALT_N; i++){
  //    alts[i] = 0;
  //    altTimes[i] = 0;
  //  }
  //
  //  for(unsigned int i = 0; i < ACCEL_N; i++){
  //    accel[i] = 0;
  //    accelTimes[i] = 0;
  //  }
  //} //END initializeArrays()


 




 /*/$$$$$$  /$$   /$$ /$$$$$$       /$$$$$$$$                              /$$     /$$
 / $$__  $$| $$  | $$|_  $$_/      | $$_____/                             | $$    |__/
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
@brief  prints the state struct
Author: Ben
*/
/**************************************************************************/
void printState(struct stateStruct state, String label) {
	Serial.println();
	Serial.println(label);
	Serial.println(state.alt);
	Serial.println(state.vel);
	Serial.println(state.accel);
}

/**************************************************************************/
/*!
@brief  Returns a received response from the Java program to ensure successful delivery
Author: Jake
*/
/**************************************************************************/
void returnResponse(char response) {
	if (response == '~') {
		;
	}
	else {
		Serial.print(response);
		Serial.print(" RECEIVED;");
		Serial.flush();
	}
} //END returnResponse()

  /*
  _  __     _                         ______                _   _
  | |/ /    | |                       |  ____|              | | (_)
  | ' / __ _| |_ __ ___   __ _ _ __   | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
  |  < / _` | | '_ ` _ \ / _` | '_ \  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
  | . \ (_| | | | | | | | (_| | | | | | |  | |_| | | | | (__| |_| | (_) | | | \__ \
  |_|\_\__,_|_|_| |_| |_|\__,_|_| |_| |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
  */
  /**************************************************************************/
  /*!
  @brief  Filters the state of the vehicle
  Author: Ben, Denny, and Lydia
  */
  /**************************************************************************/
struct stateStruct kalman(int16_t encPos, struct stateStruct rawState) {
	//static float x_k[3] = { 0, 0, 0 };
	static struct stateStruct filteredState;
	static unsigned long lastTime;
	uint16_t delta_t;
	float x_k[3];
	float z_k[3];
	static float p_k[3][3] = {
		{ 0, 0, 0 },
		{ 0, 0, 0 },
		{ 0, 0, 0 }
	};
	float k_gain[3][3];
	float placeHolder_3_1[3];
	float placeHolder_3_3_1[3][3];
	float placeHolder_3_3_2[3][3];
	float placeHolder_3_3_3[3][3];
	float u_k;
	float c_d;
	float area;
	float q;
	float b_k[3];
	float f_k[3][3] = {  
		{ 1, 0, 0 },
		{ 0, 1, 0 },
		{ 0, 0, 0 }
	};

	x_k[0] = filteredState.alt;
	x_k[1] = filteredState.vel;
	x_k[2] = filteredState.accel;

	z_k[0] = rawState.alt;
	z_k[1] = rawState.vel;
	z_k[2] = rawState.accel;

	delta_t = rawState.time - lastTime;
	lastTime = rawState.time;

	b_k[0] = (float)delta_t*delta_t;
	b_k[0] = b_k[0] / 2000000;
	b_k[1] = (float)delta_t / 1000;
	b_k[2] = 1;

	f_k[0][1] = (float)delta_t / 1000;

#if DEBUG_ALPHA
	Serial.println("Kalman");
	Serial.print("delta_t = "); //temp
	Serial.println(delta_t); //temp
	Matrix.Print(x_k, 3, 1, "x_k"); //temp
	Matrix.Print((float*)f_k, 3, 3, "f_k"); //temp
	Matrix.Print(b_k, 3, 1, "b_k"); //temp
#endif

	c_d = CD_R*(1 - encPos / ENC_RANGE) + CD_B / ENC_RANGE;
	area = A_R*(1 - encPos / ENC_RANGE) + A_B / ENC_RANGE;
	q = RHO * (z_k[1]) * (z_k[1]) / 2;
	u_k = -9.81 - c_d * area * q / POST_BURN_MASS;
	// if acceleration > 10m/s^2 the motor is probably burning and we should add that in to u_k
	if (z_k[2] > 10) {
		u_k += AVG_MOTOR_THRUST / POST_BURN_MASS;
	}

#if DEBUG_ALPHA
	Serial.print("u_k = ");
	Serial.println(u_k);
	//Serial.print("c_d = "); //temp
	//Serial.println(c_d); //temp
	//Serial.print("area = "); //temp
	//Serial.println(area); //temp
	//Serial.print("q = "); //temp
	//Serial.println(q); //temp
#endif

	//Predict----------------------------
	//x_k = u_k*b_k + f_k*x_k
	Matrix.Scale((float*)b_k, 3, 1, u_k);
	Matrix.Multiply((float *)f_k, (float *)x_k, 3, 3, 1, placeHolder_3_1);
	Matrix.Add((float*)b_k, (float*)placeHolder_3_1, 3, 1, (float*)x_k);

#if DEBUG_ALPHA
	//Matrix.Print(b_k, 3, 1, "u_k*b_k"); 
	//Matrix.Print(placeHolder_3_1, 3, 1, "f_k*x_k"); 
	Matrix.Print(x_k, 3, 1, "x_k predict");
#endif

	//p_k = q_k + f_k*p_k*T(f_k)
	Matrix.Multiply((float*)f_k, (float*)p_k, 3, 3, 3, (float*)placeHolder_3_3_1);
	Matrix.Transpose((float*)f_k, 3, 3, (float*)placeHolder_3_3_2);
	Matrix.Multiply((float*)placeHolder_3_3_1, (float*)placeHolder_3_3_2, 3, 3, 3, (float*)placeHolder_3_3_3);
	Matrix.Add((float*)placeHolder_3_3_3, (float*)q_k, 3, 3, (float*)p_k);

#if DEBUG_ALPHA
	Matrix.Print((float*)p_k, 3, 3, "p_k predict");
#endif

	//Kalman Gain------------------------
	//p_k*T(h_k) / (r_k + h_k * p_k * T(h_k)) ==
	//p_k / (r_k + p_k)    ..When h_k = eye(3)
	Matrix.Add((float*)r_k, (float*)p_k, 3, 3, (float*)placeHolder_3_3_1);
	Matrix.Invert((float*)placeHolder_3_3_1, 3);
	Matrix.Multiply((float*)p_k, (float*)placeHolder_3_3_1, 3, 3, 3, (float*)k_gain);

#if DEBUG_ALPHA
	Matrix.Print((float*)k_gain, 3, 3, "kalman gain");
	//Matrix.Print((float*)placeHolder_3_3_1, 3, 3, "1 / (r_k + p_k)");
#endif

	//Update-----------------------------
	//x_k = k_gain * (z_k - x_k) + x_k
	Matrix.Subtract((float*)z_k, (float*)x_k, 3, 1, (float*)placeHolder_3_3_1);
	Matrix.Multiply((float*)k_gain, (float*)placeHolder_3_3_1, 3, 3, 1, (float*)placeHolder_3_3_2);
	Matrix.Add((float*)x_k, (float*)placeHolder_3_3_2, 3, 1, x_k);

	//p_k = p_k - k_gain * p_k
	Matrix.Multiply((float*)k_gain, (float*)p_k, 3, 3, 1, (float*)placeHolder_3_3_1);
	Matrix.Subtract((float*)p_k, (float*)placeHolder_3_3_1, 3, 1, (float*)p_k);

	filteredState.alt = x_k[0];
	filteredState.vel = x_k[1];
	filteredState.accel = x_k[2];
	filteredState.time = rawState.time;

	return filteredState;
}


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
//void updateTimesAlts(void){
//  for (unsigned i = ALT_N-1; i>0; i--){
//    alts[i] = alts[i-1];
//    altTimes[i] = altTimes[i-1];
//  }
//  alts[0] = getAltitude() - padAlt;
//  altTimes[0] = millis();
//} //END updateTimesAlts()




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
float getAcceleration(void) {

	return 0;
}
//TO DO:::: getAcceleration()
/*********************END FUNCTION DEFINITIONS*********************/


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
void writeToFile(struct stateStruct sensorData, struct stateStruct kalmanData){
  File myFile = sd.open("VDSv2FlightData.dat", FILE_WRITE);

  if(myFile) {
    Serial.print("Writing data to test.dat;");
  } else {
    Serial.print("Unable to open test.dat;");
  }
    myFile.print(sensorData.time);
    myFile.print(",");
    myFile.print(sensorData.alt);
    myFile.print(",");
    myFile.print(sensorData.vel);
    myFile.print(",");
    myFile.print(sensorData.accel);
    myFile.print(",");
    myFile.print(kalmanData.alt);
    myFile.print(",");
    myFile.print(kalmanData.vel);
    myFile.print(",");
    myFile.print(kalmanData.accel);
    myFile.println("");
    myFile.close();
} //END writeToFile()


/**************************************************************************/
/*!
@brief  Retrieves past flight data for tests.  Replaces sensor functions
Author: Jake
*/
/**************************************************************************/
struct stateStruct readFromFile(void){
  File myFile = sd.open("8_6_16_test.dat", FILE_READ);
  char place = '\n';
  char number[20] = {NULL};
  short numPlace = 0, numCount = 0;
  float value = 0;
  int lineCount = 0;
  static int linePlaceHolder = 0;

  struct stateStruct = fileData;

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
          fileData.time = (value*1000);
          break;
          
        case 2:
          fileData.alt = value;
          break;
        }
        resetNumber(number);
      } else {
        value = numToFloat(number);
        numCount++;
        numPlace = -1;
        resetNumber(number);
        lineCount++;
        fileData.accel = value;
      }

      if(numCount == 0) {
        
      } else {
        if((numCount % ((linePlaceHolder+1)*3)) == 0){
          Serial.print("Time: ");
          Serial.print(fileData.time);
          Serial.print(";");
          Serial.print("Altitude: ");
          Serial.print(fileData.alt);
          Serial.print(";");
          Serial.print("Acceleration: ");
          Serial.print(fileData.accel);
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
